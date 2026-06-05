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
