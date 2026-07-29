# Changelog

Detailed log of what changed and how to use it. Git commit messages stay short
and imperative; the full description + usage of each change lives here. Add an
entry **before each commit**, newest on top.

Entry template:
```
## <short title> — YYYY-MM-DD
### What
- bullet points of the change
### Why
- the reason / problem solved
### How to use
- new APIs, commands, config knobs, test steps
### Notes / follow-ups
- caveats, tracked debts
```

---

## CAN hardening: bus-off auto-recovery + hardware node filter — 2026-07-29
### What
- **Bus-off auto-recovery ON** (`CAN_AUTO_BUS_OFF_RECOVERY 1` in `bsp/can.c`):
  after a bus-off the controller rejoins the bus by itself (hardware ABOM
  sequence, 128 x 11 recessive bits) instead of staying off-bus until reboot.
- **New `can_bus_set_node_filter(node_id)` in `bsp/can`** — programs two
  32-bit mask filters: this node (`ID = node<<5 | cmd`, cmd bits don't-care)
  and the broadcast node 0, standard frames only. Replaces the accept-all
  filter installed by `can_bus_init`.
- **`can_proto` applies it**: in `can_proto_init`, and re-applies live in
  `can_proto_poll` when `set can_id N` changes the address — re-addressing a
  node no longer needs a reboot.
- **Pre-commit review fixes**: `CAN_NODE_BCAST` was 1 (dropped every node-0
  broadcast frame in software after the hardware filter had accepted it) — now
  0, so broadcast commands actually reach the dispatcher. An unknown SET_MODE
  byte no longer arms the command watchdog against motion it did not start.
- Version bumped 0.2.6 → 0.2.7 (PATCH: logic changed, stored params kept).
### Why
- The two follow-ups from the protocol-layer entry: a bus-off previously
  stranded the node until re-init, and the accept-all filter would burn
  CPU/ring slots on every foreign frame once on a busy multi-node bus.
### How to use
- Nothing to do — both are automatic. Verify on the bench: `canproto` commands
  work as before; frames addressed to other node IDs no longer show up in
  `canstat`; shorting/bus-loading the bus into bus-off now recovers on its own
  (watch TEC/REC in `canstat` return to 0).
### Notes / follow-ups
- Trade-off: `canstat` can no longer sniff the whole bus (foreign frames are
  dropped in hardware now). To sniff, comment out the
  `can_bus_set_node_filter` call in `can_proto_init` (the BSP init stays
  accept-all) and rebuild.
- MIT impedance control (0x10) remains reserved/unimplemented.
- Also recorded here: `SHELL_BUFFER_SIZE` raised 128 -> 512 in
  `app/shell_port.c` — letter-shell splits the buffer across the edit line plus
  5 history slots (`shell.c: bufferSize = size / (HISTORY+1)`), so 128 left only
  ~20 usable chars per command (too short for e.g. `cansend 23 10 27 00 00`);
  512 gives ~84.

---

## CAN protocol layer: command + telemetry over CAN — 2026-07-28
### What
- **New `app/can_proto.{c,h}` — CAN application protocol.** Sits on top of the
  `bsp/can` frame transport and translates CAN frames <-> the FOC control API.
  Owns no control logic: command frames call the same `foc_set_*` setters the
  letter-shell uses, so CAN and the console are peer front-ends onto one motor.
  Serviced from the 1 ms main loop via `can_proto_poll()`.
- **Addressing (ODrive-style split of the 11-bit standard ID):**
  `ID = (node_id << 5) | cmd_id`, node_id 1..63 (0 = broadcast), cmd_id 0..31.
  The node address is the new `can_id` parameter, so a board is re-addressable
  without a recompile. Node filtering is done in software (the hardware filter
  stays accept-all so `canstat` can still watch the whole bus for debugging).
- **Command frames (host -> board):** 0x01 SET_MODE (b0 = mode enum), 0x02
  SET_CURRENT (int16 iq_mA, int16 id_mA), 0x03 SET_SPEED (int32 rpm*100), 0x04
  SET_POSITION (int32 rad*10000), 0x05 ESTOP, 0x06 CLEAR_FAULT, 0x08 SET_TELEM
  (b0 = 0/1). All little-endian, fixed-point so a CAN analyzer can drive it by
  hand. 0x10 MIT_CONTROL is reserved (impedance control) — NACKed for now, ID
  kept so the map is stable when it lands.
- **Telemetry frames (board -> host):** off by default (quiet bus for debug);
  enable with SET_TELEM. Two frames interleaved at 50 Hz each (one every 10 ms):
  0x00 HEARTBEAT (state, active-fault u32, flags) and 0x07 FEEDBACK (int16 rpm,
  iq_mA, pos*100, vbus*10).
- **Command watchdog:** `can_wdog_ms` parameter; if >0 and the host goes silent
  for that long while driving, the motor auto-stops to IDLE. **Default 0 (off)**
  — current use is bench debug with hand-sent frames, where a timeout would keep
  stopping the motor. Turn on for autonomous host control.
- **New params** `can_id` (default 1) and `can_wdog` (default 0), stored in the
  KVDB like every other parameter (added `PT_U16` to the field-table type set for
  the 16-bit watchdog). Set live with `set can_id N` / `set can_wdog <ms>`.
- **New shell command `canproto`** shows node id, telemetry/watchdog state, and
  rx/tx counters. `cansend`/`canstat` are kept for low-level bus debugging.
### Why
- The CAN BSP (2026-07-23) opened the transport but carried no semantics. This
  is the "command/monitor the drive over CAN" layer that was deferred then.
### How to use
- Node 1 (default) IDs = 0x20 | cmd_id: SET_MODE 0x21, SET_CURRENT 0x22,
  SET_SPEED 0x23, SET_POSITION 0x24, ESTOP 0x25, CLEAR_FAULT 0x26,
  SET_TELEM 0x28; board sends HEARTBEAT 0x20, FEEDBACK 0x27.
- Spin at 100 rpm from a CAN analyzer. **Mode first, then setpoint** — entering
  a mode zeros that mode's target (anti-lurch), so a setpoint sent before the
  mode is wiped:
  1. `cansend 21 02` — SET_MODE to speed (mode enum 2).
  2. `cansend 23 10 27 00 00` — SET_SPEED, int32 = 10000 (100 rpm * 100 = 0x2710 LE).
  3. `cansend 25` — ESTOP back to IDLE.
- Push +1.0 A iq: `cansend 21 01` (current mode) first, then `cansend 22 E8 03 00 00`
  (iq = 1000 mA = 0x03E8 LE, id = 0).
- Telemetry: `cansend 28 01` to start; heartbeat (0x20) and feedback (0x27)
  then appear every 10 ms on the analyzer or in `canstat`. `cansend 28 00` stops.
- `canproto` prints the live protocol state; `set can_id 2` moves the node
  (auto-saves in IDLE); `set can_wdog 100` arms the 100 ms command watchdog.
### Notes / follow-ups
- Hardware RX filter still accept-all; tighten to node+broadcast once on a busy
  multi-node bus. Bus-off auto-recovery still off in the BSP.
- MIT impedance control (0x10) reserved but not implemented — it needs a torque
  synthesis path (pos+vel+Kp/Kd+τff -> iq) in the FOC core, deferred.
- Version bumped 0.2.5 → 0.2.6 (PATCH: two params added, stored params kept).

---

## CAN BSP: CAN0 frame transport (PA11/PA12, 1 Mbps) — 2026-07-23
### What
- **New `bsp/can.{c,h}` — CAN0 frame transport.** GPIO PA11=RX / PA12=TX
  (default mapping, per `doc/GD32F303CCT6_pinmap.xlsx` sheet1; the sheet2
  "PB8/PB9 remap" line is only a remark about the alternative and was NOT
  used — PB8 is the WS2812B LED). Bit timing from APB1 60 MHz: prescaler 4,
  BS1 11 tq, BS2 3 tq (15 tq/bit, 80% sample point) → 1 Mbps; compile-time
  macros, with the 500 kbps alternative noted next to them.
- **Frame API** (`can_bus_*`, because the GD32 SPL owns the `can_*` prefix):
  `can_bus_init/send/recv/get_stats`. RX is interrupt-driven (FIFO0 → 16-frame
  lock-free SPSC ring, drops counted when full); TX queues into a hardware
  mailbox and returns 0 when all are busy. Accept-all filter for now. CAN is a
  frame device — deliberately NOT an `io_stream` backend.
- **Debug shell hooks** `cansend <id_hex> [b0..b7]` and `canstat` (TEC/REC,
  rx_dropped, drains and prints received frames). Bring-up only; the real
  protocol layer will replace them.
- `CAN_SELFTEST_LOOPBACK` switch in `bsp/can.c` selects the internal loopback
  mode so the TX→RX path can be bench-tested with no transceiver attached.
### Why
- First step toward commanding/monitoring the drive over CAN. Scope was
  deliberately limited to the BSP; the application protocol is a separate
  follow-up.
### How to use
- No transceiver: set `CAN_SELFTEST_LOOPBACK 1`, flash, then
  `cansend 123 11 22 33` → `canstat` shows the same frame received.
- On a real bus (1 Mbps, transceiver required): `cansend 1AB DE AD BE EF`
  should appear on a CAN analyzer; frames sent to the board show up in
  `canstat`. With no bus/ACK, `cansend` starts returning "no free mailbox"
  after 3 frames and TEC climbs — that is the hardware reporting no ACK.
### Notes / follow-ups
- Bus-off auto-recovery is OFF (`CAN_AUTO_BUS_OFF_RECOVERY 0`): after a
  bus-off the node stays off-bus until re-init. Recovery policy and comm-loss
  protection were explicitly deferred to the protocol phase.
- Also recorded here: `bsp/uart.c` is now **115200 baud with 4 KB ring
  buffers** (was 3 Mbaud / 1 KB) — the "3 Mbaud VT100 terminal" note in the
  2026-06-04 entry is superseded.
- Version bumped 0.2.4 → 0.2.5 (PATCH: feature added, stored params kept;
  also covers the anti-cogging KV key which missed its bump).

## Drop automatic encoder alignment on boot — 2026-06-12
### What
- **Removed the power-up encoder zero-alignment step from `main()`** (the
  `foc_start_calibration(&g_motor, 1)` that ran whenever `cali_valid == 0`).
- The power-up **current-offset** calibration (`foc_init` → `foc_start_calibration(_, 0)`,
  no rotor motion) is unchanged and still runs every boot.
### Why
- After a flash erase + reflash, `cali_valid` is 0, so the old code auto-ran the
  alignment on first power-up — twitching the rotor on every fresh flash, an
  unexpected jolt and a small risk. Aligning is now a deliberate user action.
### How to use
- After erasing/reflashing, run `autocal` from the shell once to align the encoder
  (+ identify Rs/Ld/Lq/flux/J/B and deploy gains); the alignment is persisted via
  the CALI→IDLE edge, so subsequent boots need no re-alignment. Until then,
  closed-loop modes use the compile-time default `zero_offset` and may not run
  correctly — by design.

## Extract parameter identification into foc_ident — 2026-06-12
### What
- **Moved the electrical (`FOC_STATE_IDENT`) and mechanical (`FOC_STATE_IMECH`)
  identification sub-state-machines out of `foc_core.c/.h` into `foc/foc_ident.c/.h`**
  and wired `foc_ident.c` into the MDK build (`f303_foc.uvprojx`).
- `foc_core.h` now `#include`s `foc_ident.h` for `ident_ctx_t` / `imech_ctx_t`
  (composed into `motor_t`); the duplicate type/enum/`#define` blocks and the
  `foc_start_ident*` declarations were removed from `foc_core.h`.
- `foc_control_loop` now calls the extracted hooks `foc_ident_run()` /
  `foc_imech_run()` in the IDENT/IMECH switch cases, and the locked-rotor angle
  override uses the public `foc_ident_locks_rotor()` instead of an inline phase test.
### Why
- A prior restore left **two copies** of the ident code: the live one inlined in
  `foc_core`, plus an orphaned (unbuilt, un-included) `foc_ident.c/.h`. Adding
  `foc_ident.c` to the build as-is would have caused duplicate `foc_start_ident*`
  symbols and, in any TU including both headers, duplicate `ident_ctx_t` etc.
  This finishes the split so identification lives in one module, mirroring `foc_acog`.
### Notes / follow-ups
- Pure refactor — verified function-by-function that the moved logic is byte-identical
  to the old `foc_core` version (only comments referencing the new function names
  differ). No control behaviour change. Dropped the unused `ident_is_rs_phase` helper.

## Anti-cogging map (step-and-settle calibration) — 2026-06-09
### What
- **New `FOC_STATE_ACOG` calibration that learns a per-mechanical-angle
  feed-forward torque-current map** (`foc/foc_acog.c/.h`, 1024-bin int8 table).
  At run time the map is added to the iq command in CURRENT/SPEED/POSITION modes
  to cancel cogging (`foc_acog_lookup`, two loads + a lerp in the 20 kHz ISR).
- **The sweep REUSES the position→speed→current cascade**, it does NOT use a
  position→current spring. It steps the position controller one bin at a time,
  lets the rotor **settle to rest**, then measures; the speed-loop **integral**
  holds each angle and `max_rpm` caps the hop speed (smooth, no noise
  amplification). One bin per step → the cascade always reaches it in a short
  fixed dwell, so **no per-point arrival check** (the gate that made the old
  spring take an hour). 2·1024 steps is a deterministic ~3.5 min.
- **Why measure at rest (DC), not while moving**: this motor has 168 cogging
  cycles/rev, so even a slow continuous creep puts the cogging frequency far
  above the soft speed-loop bandwidth — the loop can't track it and the measured
  iq comes out attenuated + phase-lagged (a weak, misaligned map that doesn't
  cancel cogging; the first continuous-creep cut measured only ~0.25 A peak and
  did not smooth low-speed running). At rest the holding torque is DC where the
  integrator has full authority, so iq == the true cogging torque at that angle.
- **Measurement**: the held iq is averaged over a 10 ms window after a 40 ms
  settle and accumulated into the rest-angle bin; forward+reverse over the same
  bins cancel static friction. Finalise removes the DC component (cogging is
  zero-mean; the offset is friction that must not be fed forward), clamps `max_ff`.
- **Persistence**: stored as a ~1 KB int8+scale blob KV (`"cogging"`) in the
  existing FlashDB KVDB, loaded on boot (`params_capture_acog` /
  `load_acog_from_flash`). Auto-saved on a successful sweep; `acog on/off`
  persists the enable flag; `acog clear` deletes it.
- **New shell command** `acog <cal [rpm]|on|off|clear|info>`. `foc_acog.c` added
  to the Keil project (`MDK-ARM/f303_foc.uvprojx`).
### Why
- The previous attempt drove calibration with a stiff position→current spring,
  which (no integrator) could not both hold a precise angle and break a cogging
  detent: it stick-slipped, oscillated when stiffened, and the per-point arrival
  gate made a sweep take an hour. The reference firmware (`Firmware_app`) creeps
  smoothly because it keeps the integrator-backed speed loop in the loop and
  velocity-limits the motion — this change adopts that approach on top of this
  project's existing cascade.
### How to use
- Prerequisites: encoder aligned + current offsets valid (`autocal` once). Then:
  `acog cal` (optionally `acog cal <rpm>` to override the ~10 rpm inter-bin hop
  cap; it only sets hop speed, not accuracy — measurement is at rest). Keep the
  rotor clear; watch `status` (state ACOG → IDLE, ~3.5 min). On success the map
  auto-enables and saves (~3.5 min); `acog info` shows `valid/enabled/peak_A`.
  Verify with `acog off` vs `acog on` at a low `speed`.
- Tuning with no scope: if the rotor can't reach/hold a bin, raise the speed
  integral (`set spd_ki …`); raising the hop cap just speeds the sweep. Do **not**
  raise the position landing gain for stiffness.
### Notes / follow-ups
- Calibration statics add ~7 KB SRAM (map 4 KB + counts 2 KB + blob ~1 KB); the
  counts are cal-only. Watch the linker map if SRAM gets tight.
- `motor_t` is now the tagged `struct motor_s` (guarded forward-decl) so
  `foc_acog.h` can compose `acog_ctx_t` into it the same way the orphan
  `foc_ident.h` intended; no behavioural change to existing code.
- A `stop` mid-sweep discards the partial map (only `ACOG_DONE` finalises), but
  the in-RAM map was already cleared at sweep start — reboot restores the stored
  copy.

## One-shot `autocal` + command/cleanup pass — 2026-06-06
### What
- **New `autocal` command runs the entire commissioning sequence end-to-end**:
  current-offset cal + encoder alignment → Rs/Ld/Lq/flux ident → deploy
  current-loop gains (1 kHz) → J/B/Tc ident → deploy speed-loop gains (10 Hz),
  auto-saved. A **1 s settle delay between stages** lets residual current/motion
  decay so one measurement cannot corrupt the next (notably: the flux spin fully
  stops before the J/B accel test). Orchestrated from the main loop
  (`autocal_start/abort/active` in `main.c`); `stop` aborts it; a fault aborts
  it; `status` shows `autocal: RUNNING`. Watch the control state to follow
  progress (CALI → IDENT → IMECH → IDLE).
- **Removed the now-redundant standalone calibration commands**: `cali`, `align`,
  `ident`, `identmech` (all subsumed by `autocal`; boot still auto-cals/aligns).
- **Kept** `tune [bw]` / `tunespd [bw]` as re-deploy utilities (recompute gains
  from stored params at a different bandwidth without re-identifying) and
  `identinfo` for inspection. The gain math is now shared via
  `params_tune_current_loop()` / `params_tune_speed_loop()` (used by both the
  shell and autocal).
- **Dead-code cleanup**: removed unused `pid_calc` / `pid_calc_incremental`
  (only `pi_calc` is used), `motor_hw_calibrate_current_offset` (+`CALIBRATION_
  SAMPLES`), and `foc_calibration_done` — all confirmed to have no callers.
### Why
- Six-parameter identification + deployment is now a single command instead of
  6+ separate steps; far less error-prone and removes the command sprawl that
  had become confusing.
### How to use
- From IDLE: `autocal`. Rotor twitches (align) → holds (Rs/Ld/Lq) → spins
  (flux, J/B); ~45 s; keep clear. When `status` returns to IDLE, `get` /
  `identinfo` show the six parameters and the deployed gains (already saved).
- Re-tune a loop at a different bandwidth without re-identifying: `tune 1500` /
  `tunespd 8`.
### Notes / follow-ups
- Bandwidths inside `autocal` are fixed (1 kHz current / 10 Hz speed); use
  `tune`/`tunespd` to change them afterwards.
- Version bumped 0.2.3 → 0.2.4 (PATCH: logic/commands changed, stored params
  kept).

## Position loop: expose landing gain + deadband for tuning — 2026-06-06
### What
- **The position controller's near-target P gain and deadband are now runtime-
  tunable** (`pos_kp`, `pos_db`), moved from the compile-time `POS_LANDING_KP` /
  `POS_DEADBAND_RAD` macros into `pos_controller_t` (`landing_kp`, `deadband`)
  and the flash parameter set. The profile limits (`pos_rpm/acc/jerk`) were
  already tunable; this exposes the remaining knobs.
- **Default landing gain raised 5 → 12 rad/s** (~0.8 Hz → ~1.9 Hz). The position
  loop is a trajectory profile + P-landing cascade on top of the (now tuned)
  speed loop; for moves smaller than the cruise region the landing term sets the
  whole response, so 5 was sluggish. 12 is ~1/5 of the 10 Hz speed loop.
### Why
- Position tuning is mostly choosing motion limits within the physical (J) and
  cascade (speed-loop bandwidth) bounds, plus the landing gain for crisp, no-
  overshoot settling. Those knobs need to be adjustable live, not hardcoded.
### How to use
- `mode position; pos <deg>` to command a move (degrees, rotor side). Tune live:
  `set pos_kp <val>` (raise toward ~20 for crisper, lower if it buzzes/overshoots
  — keep below ~speed_bw/3 ≈ 20 rad/s), `set pos_db <rad>` to kill final hunting,
  `set pos_acc` / `set pos_jerk` for the accel/decel feel. `save` persists.
- J gives the achievable-accel ceiling: `a_max = Kt·iq_max/J ≈ 0.0809·5/4.8e-5 ≈
  8400 rad/s²`, so `pos_acc` (500) and the jerk-implied accel (~2094) have wide
  margin — raise them for snappier moves if desired.
### Notes / follow-ups
- Encoder is rotor-side: position control is precise at the rotor; the 1:8
  gearbox backlash adds ±lash at the output. High-accuracy *output* positioning
  needs an output encoder or backlash compensation.
- Optional next: acceleration feed-forward (Te_ff = J·dω_ref/dt) into the speed/
  current loop to cut following error during fast moves.
- Version bumped 0.2.2 → 0.2.3 (PATCH: parameters added, stored params kept).

## Speed loop: deploy J/B-derived gains + tunespd — 2026-06-06
### What
- **Speed-loop PI gains deployed from the identified J/B** at a **10 Hz**
  bandwidth by pole-zero cancellation. Plant `ω/iq = Kt/(J·s + B)`; gains
  `Kp = (J·ω_s/Kt)·(2π/60)`, `Ki = (B·ω_s/Kt)·(2π/60)` (A per rpm, `ω_s=2π·10`).
  New defaults `spd_kp=0.0039, spd_ki=0.0227` (were 0.006 / 0.1).
- **New `tunespd [bw_hz]` shell command** recomputes/applies the speed gains from
  the stored J/B/flux (default 10 Hz) — mirrors `tune` for the current loop.
- **No controller change**: the speed loop was already a standard cascade PI
  (rpm error → iq) and already uses the explicit-dt form from the current-loop
  refactor (Ts = 1 ms). Only the gains changed.
### Why
- `Ki/Kp` now equals `B/J` (integral zero on the mechanical pole, time constant
  ≈ J/B ≈ 173 ms) instead of the previous hand-tuned `Ki/Kp = 16.7` whose zero
  sat ~3× above the plant pole (over-aggressive integral → overshoot). Kp now
  targets a defined 10 Hz bandwidth, comfortably under the PLL speed-feedback
  bandwidth (~40 Hz, the binding constraint for the speed loop) and the current
  loop. Completes the chain: all six parameters measured and fed into control.
### How to use
- After `identmech` (J/B) with a valid flux: `tunespd` (or `tunespd 5` for a
  gentler loop). Then test: `mode speed; speed 200` and watch for a clean,
  well-damped response. The shipped defaults already encode the 10 Hz gains.
### Notes / follow-ups
- Speed bandwidth is capped by the PLL (~40 Hz); going much above ~10 Hz needs a
  faster PLL. Optional future work: acceleration feed-forward (Te_ff=J·dω_ref/dt)
  and/or friction feed-forward using the measured B/Tc.
- Version bumped 0.2.1 → 0.2.2 (PATCH: defaults tweaked, command added).

## Motor parameter identification — phase 4: J/B/Tc (mechanical) — 2026-06-06
### What
- **New `FOC_STATE_IMECH` + `identmech [iq]` shell command** — identifies the
  rotor-side **inertia J, viscous friction B, and Coulomb friction Tc**, all in
  **torque (current) mode** (no speed loop), the industry-standard approach
  (TI SpinTAC does inertia ID as the *first* step, before the speed controller).
- **Method — bidirectional ±torque accel/decel across speed gates.** With
  `Id=0`, apply `+iq_test` to ramp the rotor up through gates at 100…600 rotor
  rpm (timestamping each crossing), then `−iq_test` to actively brake back down.
  Per band: `J = 2·Te/(a_up + a_dn)` — the friction torque cancels because both
  directions traverse the same speed (`Te = Kt·iq`, `Kt = 1.5·p·flux` from the
  flux ident). The residual `f = J·(a_dn − a_up)/2` is the friction torque at
  each band midpoint; a least-squares fit of `f` vs ω gives `B` (slope) and `Tc`
  (intercept). J is averaged over bands.
- **PLL speed is sufficient**: per-band timing is the *difference* of two gate
  crossings, so a roughly-constant PLL speed lag cancels out.
- **Results** stored in display-friendly units: `J` in g·cm² (1e7·kg·m²), `B` in
  µNm·s/rad, `Tc` in mNm. New KV keys `inertia`/`visc_b`/`fric_tc`/`mech_ok`.
  Read with `get` or `identinfo`.
- **Safety**: speeds capped at 600 rpm gates, hard abort at 1000 rpm, per-phase
  timeouts, active braking keeps the rotor bounded; the slow-monitor overspeed/
  i²t backstops stay armed (current is modest, ~1 A). Hard overcurrent active.
### Why
- J and B are needed to physically derive the **speed-loop** gains (as R/L gave
  the current-loop gains). Measuring them in torque mode avoids the chicken-and-
  egg of needing a tuned speed loop to tune the speed loop.
### How to use
- After a successful electrical `ident` (flux gives Kt): `identmech` (or
  `identmech 1.5` to set the test current). **The rotor spins up then brakes** —
  keep clear. Wait for IDLE, then `get` / `identinfo` for J/B/Tc.
- If a band's timing is too short (fast ramp → noisy), lower the test current;
  if it can't reach 600 rpm, raise it.
### Notes / follow-ups
- Measures **rotor-side, no-load** J/B (rotor + gearbox-input inertia). The 1:8
  gearbox backlash slightly perturbs the accel↔decel reversal; multi-band
  averaging absorbs it. Reflected load inertia (÷N²) is small — see earlier note.
- **Not yet used by control.** Next optional step: derive the **speed-loop** PI
  gains from J/B and deploy (mirroring the current-loop `tune`).
- Version bumped 0.2.0 → 0.2.1 (PATCH: parameters added, stored params kept).

## Current loop: explicit-dt PI + deploy identified gains — 2026-06-06
### What
- **PI controller refactored to the explicit-Ts form** (matching VESC / ODrive /
  SimpleFOC): `pid_t` gains a `ts` field, `pid_init()` takes it, and the
  integrator is now `integral += ki·error·ts` (was `integral += ki·error` with Ts
  folded into Ki, the ST/TI style). `ki` is therefore now the **continuous-domain
  gain**, so the physics formula maps directly with no hidden ×Ts. Each loop
  carries its own `ts` (current loops 50 µs, speed loop 1 ms), which removes the
  multi-rate footgun. `pid_calc`/`pid_calc_incremental` updated for consistency
  (`ki` term ×ts, `kd` term ÷ts).
- **Current-loop gains deployed from the identified parameters** at a **1 kHz**
  bandwidth by pole-zero cancellation (`Kp = L·ω_bw`, `Ki = R·ω_bw`,
  `ω_bw = 2π·1000`). New defaults: `id_kp=0.69, iq_kp=1.00` (Lq>Ld so the q-axis
  gain is higher — previously d/q shared one gain), `id_ki=iq_ki=1527`. Speed
  loop Ki rescaled to its continuous value (`0.0001 → 0.1`); behaviour unchanged.
- **New `tune [bw_hz]` shell command** recomputes and applies the id/iq gains
  from the stored Rs/Ld/Lq at the given bandwidth (default 1000 Hz) — the proper
  way to redeploy after a fresh `ident` or to try another bandwidth.
### Why
- The folded-Ts form is valid but obscures the gain units and is fragile across
  loop rates; the explicit-dt form is the open-source-FOC standard and makes
  "motor parameters → controller gains" a direct calculation. Bandwidth 1 kHz
  chosen for ~63° phase margin against the ~1.5·Ts digital delay while staying
  well above the motor pole (R/L ≈ 352 Hz) and the outer loops.
### How to use
- After flashing: the board auto-aligns on first boot (params were reset, see
  below). Run `ident` to repopulate Rs/Ld/Lq/flux, then `tune` (or `tune 1500`)
  to recompute gains from the fresh measurement; `get` to inspect. The shipped
  defaults already encode the 1 kHz gains for this motor.
### Notes / follow-ups
- **Version bumped 0.1.1 → 0.2.0 (MINOR): the meaning/units of the stored Ki
  changed, so stored params auto-reset to defaults on first boot** — re-`align`
  (automatic on boot) and re-`ident` as needed.
- Mechanical J/B (phase 4) remain future work.

## Motor parameter identification — phase 3: flux linkage — 2026-06-06
### What
- **Flux linkage added to `FOC_STATE_IDENT`**, completing the electrical set:
  `ident` now runs Rs → Ld → Lq → flux. The first three lock the rotor; the flux
  phase **spins the rotor** (no-load) and coasts it.
- **Method — Iq=0 coast-down (VESC "undriven").** Spin up to a target rotor
  speed (`IDENT_FLUX_SPIN_RPM=400`, via the existing current loop with
  `Iq=1.5 A`, `Id=0`), then command `Id=Iq=0` and free-coast. With zero current
  the applied `Vq` equals the back-EMF, so `Vq = omega_e · flux` — independent of
  Rs and Ld. `(Vq, omega_e)` pairs are accumulated over the decaying speed and a
  **least-squares fit through the origin** (`flux = Σ(Vq·ωe)/Σ(ωe²)`) gives the
  flux linkage; weighting by ωe² favours the high-SNR high-speed samples.
- **Stored in milliwebers** (1e3·Wb) for clean display. Reported via `identinfo`
  and `get flux`.
- **`ident_valid` is now set to 1** on completion (flux is measured last, so a
  non-zero flux implies the whole set is present).
- **Safety:** rotor speed is firmware-capped (abort above `IDENT_FLUX_MAX_RPM=800`,
  well under the ~1200 rpm rotor limit); spin-up and coast both have timeouts;
  hard overcurrent remains active throughout.
### Why
- Flux linkage gives the torque constant `Kt = 1.5·p·flux` and feeds the
  back-EMF feed-forward / sensorless observer. It is the last of the four
  load-independent electrical parameters.
### How to use
- `cali` (if needed) → `ident`. **The rotor will spin up** after the locked Rs/L
  phases — keep clear. Wait for `IDLE`, then `get` (`flux` in mWb) or `identinfo`.
- Sanity check: `flux ≈ Vq_peak / omega_e` at the spin speed; `Kt = 1.5·14·flux`
  should match the motor's torque rating order-of-magnitude. Coast must last long
  enough to gather points (low-friction no-load is ideal).
### Notes / follow-ups
- **All four electrical parameters (Rs/Ld/Lq/flux) are now identified but still
  not fed into the control loops** — deployment (current-loop gains from R/L with
  the `Ki·Ts` mapping, Kt from flux) is the next step, on the user's sign-off.
- Mechanical J/B (phase 4) remain future work.
- The spin uses the existing (working) current loop, not R/L-derived gains —
  simpler and proven; the coast measurement is independent of the loop tuning.

## Motor parameter identification — phase 2: Ld/Lq — 2026-06-06
### What
- **Ld/Lq added to `FOC_STATE_IDENT`**, continuing straight after the Rs phases
  (`ident` now runs Rs → Ld → Lq, ~8 s total, rotor stays locked at phase A).
- **Method — locked-rotor square-wave voltage injection.** A closed-loop DC
  d-current (`IDENT_L_I_BIAS=1.5 A`) first locks the rotor; the steady drive
  voltage is **captured from the current loop** and reused as an open-loop bias
  (so it already carries the dead-time compensation — important since
  `Rs·I_bias ≈ 0.36 V` is below the dead-time drop and pure feed-forward would
  inject almost no current). A ±V square wave (~3.3 kHz) is then injected on the
  d-axis (Ld) and q-axis (Lq); the peak-to-peak current ripple is averaged over
  200 cycles. **Two amplitudes** (`V_LOW=1.0`, `V_HIGH=2.0 V`) are used and the
  slope taken — `L = (Vhi−Vlo)·T_half / (PPhi−PPlo)` — so the dead-time offset
  cancels, exactly like the Rs two-point method. Injection current is soft-capped
  at 5 A (hard overcurrent at 14 A remains the backstop).
- **Rs robustness:** the two-point slope now divides by the **measured** average
  Id (not the nominal setpoint), removing any dependence on the loop reaching the
  exact command.
- **`Ld`/`Lq` stored in microhenries** (1e6·H) so they display with the shell's
  3-decimal float printer instead of rounding to `0.000`.
- **New `identinfo` shell command** dumps the raw measurements of the last run
  (Vd_low/high, measured Id_low/high, Rs, the d/q ripple at both amplitudes, Ld,
  Lq) for cross-checking.
### Why
- Ld/Lq are needed for the current-loop gains (`Kp=L·ω_bw`) and dq decoupling;
  having them measured (not guessed) is the second of the four electrical params.
### How to use
- `cali` (if needed) → `ident`. Wait for `IDLE`, then `get` (shows `ld`/`lq` in
  µH) or `identinfo` for the raw data. Auto-saves on return to IDLE.
- Sanity check: expect Ld ≈ Lq for this (likely low-saliency) motor; the implied
  current-loop bandwidth from the existing gains and the measured R/L should be a
  few hundred Hz to ~1 kHz.
### Notes / follow-ups
- The injection amplitudes/frequency (`IDENT_L_*`) are tuned for a ~0.1–0.2 mH,
  0.24 Ω motor; if `ld`/`lq` come back `0.000` the run hit the 5 A soft-cap
  (reduce `IDENT_L_V_HIGH`) or the ripple was too small (raise it).
- Still **not fed into the control loops** (Rs/Ld confirmed first, gains applied
  only after the user signs off). Phase 3 = flux linkage (Iq=0 coast-down).
- `ident_valid` still 0 until flux lands.

## Motor parameter identification — phase 1: Rs — 2026-06-06
### What
- **New `FOC_STATE_IDENT` control state + `ident` shell command** that identifies
  motor electrical parameters autonomously in the 20 kHz control ISR (modelled on
  the existing `FOC_STATE_CALI` sub-state machine). Phase 1 measures **stator
  resistance Rs**; the state machine, params storage and result-capture path are
  scaffolded so Ld/Lq (phase 2) and flux linkage (phase 3) slot in next.
- **Rs method — two-point locked-rotor DC.** The rotor is held at `theta_e=0`
  (phase A) while the id/iq loops drive a DC d-current at two levels
  (`IDENT_RS_I_LOW=1.0 A`, `IDENT_RS_I_HIGH=2.5 A`, both < the 2.9 A rating). The
  applied `Vd` is averaged at each point and `Rs = (Vd_high − Vd_low)/(I_high −
  I_low)`. Taking the slope cancels the inverter dead-time voltage offset
  (`Vd = Rs·Id + Vdeadtime`). One run is ~1.5 s.
- **`motor_actual_t` gains `vd`/`vq`** — the current-loop voltage output is now
  stored back into the motor struct (it was only flowing into the inverse Park).
  Ident reads `Vd` from here; also useful telemetry.
- **`params_t` gains `rs`/`ld`/`lq`/`flux` + `ident_valid`** with KV keys
  `rs`/`ld`/`lq`/`flux`/`ident_ok`, persisted like any other parameter. Results
  are captured on the `IDENT → IDLE` edge (`params_capture_ident`, mirroring the
  calibration-capture path in `main.c`). **Stored for inspection only — not yet
  fed back into the control loops** (see Notes).
### Why
- Lays the groundwork for measuring the six motor parameters (Rs/Ld/Lq/ψf/J/B).
  The approach and ordering follow industry practice (VESC/ST/TI): measure the
  load-independent electrical parameters first, each later stage reusing earlier
  results. Identification is staged + bench-verifiable since the board drives a
  real joint motor.
### How to use
- From the shell, with the motor in `IDLE` and current offsets calibrated:
  `cali` (if needed) → `ident`. Wait for `status` to show `IDLE` again, then
  `get rs` (ohms). The value auto-saves to flash on return to IDLE.
- Expected for this joint motor: Rs ≈ 0.7 Ω. Cross-check against a multimeter
  (phase-to-phase ≈ 2·Rs) before trusting it.
### Notes / follow-ups
- **Not wired into control yet (by design).** The PI integrators do **not**
  multiply by Ts, so the deployment mapping for the current loop is
  `Kp_code = L·ω_bw`, `Ki_code = R·ω_bw·Ts` (Ts = 50 µs) — `Ki` must include the
  ×Ts factor. The existing defaults (`id_kp=0.72, id_ki=0.11`) are consistent
  with R≈0.7 Ω, L≈0.23 mH, ω_bw≈500 Hz. Gains will be recomputed/applied only
  after Ld is measured and the user confirms.
- **Phase 2 (Ld/Lq):** locked-rotor injection in the rotor dq frame.
- **Phase 3 (ψf):** spin up, then Iq=Id=0 free-coast and fit `Vq = ψf·ω_e`
  (VESC "undriven" method — independent of R/L). Needs the R/L-derived current
  loop for the spin-up.
- `ident_valid` stays 0 until all four electrical parameters are measured, so
  nothing keys off a partially-filled set.
- Version bumped `0.1.0 → 0.1.1` (PATCH: parameters added, stored params kept).

## Flash parameter management (FlashDB) — 2026-06-05
### What
- **FlashDB 2.2.0 (KVDB-only)** vendored at `libraries/flashdb` (inc/src + FAL;
  demos/tests removed). On-chip flash port `bsp/flash_port.c` backs the top 8 KB
  of flash (`0x0803_E000`, 4×2 KB sectors) as the KVDB partition; Keil IROM1 was
  shrunk `0x40000 → 0x3E000` so code never overlaps it. No heap (static partition
  table), no TSDB; FlashDB/FAL logs are routed to the UART (`fdb_uart_printf`)
  instead of the unretargeted `printf` (which would hit semihosting and hang).
- **`app/params.{c,h}`** — table-driven KV parameter store. Single runtime source
  of truth `params_t`; `foc_init()` defaults are captured as the factory
  fallback, then flash values override them and are applied through the existing
  foc setters + `fault_set_cfg`. Saves write only **changed** keys (an `s_saved`
  snapshot) to bound sector GC. Stored params are tagged with the firmware
  version (`app/version.h`, starts `0.1.0`); on boot a MAJOR.MINOR mismatch
  discards the stored layout (PATCH ignored). Encoder calibration (zero offset +
  direction) is persisted, so boot skips the rotor-twitching alignment once
  `cali_valid` is set. All flash writes are gated to `IDLE`.
- **Runtime-tunable protection thresholds** — the operational thresholds moved
  from `#define` into `fault_cfg_t` (`fault_set_cfg`/`fault_get_cfg`). VESC-style
  guard: `thr_protect_en` clamps values to the `*_HARD_MIN/MAX` silicon limits.
- **Persistent fault log** — a ring of 4 `fault_status_t` snapshots in a KV blob,
  pushed on the fault-latch edge (outputs already off), survives power-off.
- **Shell**: `get`/`set`/`save`/`load`/`factory`/`thrprot`/`faultlog`; `pid`/`lim`
  now persist through `params_set`.
### Why
- Keep shell-tuned gains, protection thresholds and auto-calibration across power
  cycles; tune in the field without reflashing; read the last faults after a
  power-cycle. A normal Keil download does not erase the KVDB region, so params
  survive firmware updates (only a full chip-erase or a MAJOR.MINOR version bump
  resets them).
### How to use
- `get [name]` lists/shows params; `set <name> <value>` applies live + auto-saves
  in IDLE (clamped to safe limits while `thr_en=1`). `save`/`load`/`factory` need
  IDLE. `thrprot <0|1>` toggles the threshold guard. `faultlog` dumps the stored
  fault history. Param keys: `id_kp iq_kp cur_lim spd_kp spd_lim pos_* thr_*` etc.
- Calibration auto-saves; the first boot (or after `factory`) re-aligns once then
  persists, so later boots no longer twitch.
- Keep Keil's flash setting on **Erase Sectors** (not Full Chip) so downloads
  preserve params.
### Notes / follow-ups
- FlashDB's `set_kv` never skips an unchanged value; writing all keys every save
  bloated the log and forced GC — a reset during a GC erase once made FlashDB
  self-reformat ("All sector header is incorrect", wiping params). Mitigated by
  the changed-keys-only save.
- Firmware version `0.1.0` (`app/version.h`) is the new product version (the
  banner's `3.1.2` is letter-shell's). Bump MINOR when an existing stored
  parameter's type/size/meaning changes (resets stored params); PATCH for
  everything else (params kept). Shown in the shell banner + `status`.
- `pole_pairs` is still compile-time (`HW_MOTOR_POLE_PAIRS`); current-sense
  offsets are re-measured every boot (not stored).
- `app/shell_port.c` is now 534 lines — a split candidate as more commands land.

## Protection/fault system, letter-shell console, HAL decouple — 2026-06-04
### What
- **Protection / fault** (`foc/fault.{c,h}`, new): bitmask fault codes with
  three-layer detection (nFAULT EXTI / per-PWM fast checks / 1 ms background
  monitor), active + latched + first-cause + context snapshot, FWDGT watchdog,
  recovery via `fault_try_clear`.
- **DRV8323 hardening**: OCP latched mode, VDS OCP trip 0.26 V (~29 A hot) and
  SEN OCP 0.25 V (~50 A) tuned for the ASDM40N80 stage, OTW reported on nFAULT,
  nFAULT EXTI hardware emergency stop, full-config SPI read-back verify
  (`FAULT_DRV_COMM`), `drv8323_apply_config()` / `drv8323_parse_faults()`.
- **HAL decouple**: `motor_hw` now exposes `motor_hw_get_vbus()`, the chip-neutral
  `motor_hw_driver_fault_t` with `motor_hw_driver_get_faults() / _fault_pin() /
  _fault_edge() / _reset()`, and `motor_hw_encoder_ok()`. `fault.c` and
  `foc_core.c` no longer include `drv8323.h` / `tle5012b.h` / `pwm_curr.h`; they
  go through `motor_hw`. Added `ARCHITECTURE.md` + this file.
- **Console**: ported letter-shell 3.1.2 (`libraries/letter-shell/`, export-OFF /
  manual command table, no RTOS) over a new `bsp/io_stream.h` transport
  (`uart_stream()` backend); `app/shell_port.{c,h}` provides the glue + command
  table + `scope` streaming. The interim single-char `foc/console.{c,h}` was
  removed. `vofa` gained `vofa_build_justfloat()` (transport-agnostic builder).
- **Live tuning**: `foc_core` gained `foc_set_pid_gains()` and `foc_set_limit()`.
### Why
- Complete protection + diagnosable error reporting; keep the gate driver /
  encoder chips behind the HAL so swapping them never ripples into control or
  protection; and drive/tune the motor interactively over a stable, full-featured
  terminal on a swappable transport (UART today, USB CDC later).
### How to use
- Shell commands (all take args): `mode <idle|current|speed|position|volt|openloop>`,
  `speed <rpm>`, `id <A>`, `iq <A>`, `pos <deg>`, `volt <vd> <vq>`, `cali`, `align`,
  `stop`, `status`, `dump`, `pid <id|iq|spd> <kp> <ki>`, `lim <cur|spd> <value>`,
  `clrfault`, `clrlatch`, `scope`, `reboot`, plus builtin `help` / `cls`.
- Connect a 3 Mbaud **VT100** terminal (MobaXterm/PuTTY/Tera Term); a plain
  "send-box" serial assistant does not send per-key + CR/LF, so commands never
  execute ("Command is too long") — not a firmware bug.
- Position is in DEGREES; entering a mode zeroes that mode's target (POSITION
  holds the current angle). `status` = curated indicators; `dump` = all 23
  telemetry channels as text. `main.c` boots into IDLE; the shell drives the motor.
- Read bus voltage with `motor_hw_get_vbus()`; never call bsp `read_vbus()` above
  the HAL. New chip → change `driver/` + `motor_hw.c` only.
- Thresholds in `fault.h` (24 V bus, ±16.5 A sense range, 2.9 A rated motor).
### Notes / follow-ups
- Build needs include path `..\libraries\letter-shell`; `shell_cmd_list.c` and
  `shell_companion.c` are intentionally NOT compiled (we own the command table).
- Shell and VOFA share the UART (mutually exclusive); `scope` switches into binary
  streaming and any key returns. When USB CDC lands, give VOFA its own io_stream
  for simultaneous use.
- `vofa` telemetry still lives in `foc/` with a control→telemetry push; invert to
  a pull/callback model when telemetry is reworked.
- DRV overtemp + MOSFET-anomaly tests deferred; mid-run encoder disconnect not yet
  caught (only dead-from-bring-up).
