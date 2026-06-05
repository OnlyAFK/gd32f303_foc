# FOC Firmware — Architecture

Living snapshot of how the code is organized. Update this **only when the
structure/layering changes** (not for every feature — that goes in CHANGELOG.md).
Use the "Periodic review" checklist below as a recurring health check.

---

## Layers (low → high)

A module may depend only on layers **below** it.

```
app/        main, interrupt vectors, app wiring, CLI, telemetry, params/flash
  |
foc/        control + protection + hardware abstraction
  |
driver/     external chips (drv8323 gate driver, tle5012b encoder)
bsp/        on-chip peripherals (uart, pwm_curr/ADC, flash_port, future usb)
  |
libraries/  vendor code (CMSIS, GD32 SPL, lwrb, CMSIS-DSP, letter-shell, flashdb)
```

## Module responsibilities

| Module | Layer | Responsibility |
|--------|-------|----------------|
| `pid`, `pll`, `svpwm`, `filter`, `foc_math` | foc | Pure leaf math; depend only on `foc_math`/CMSIS-DSP |
| `motor_hw` | foc | **HAL** — the *only* module that includes `driver/*` and bsp chip headers |
| `foc_core` | foc | FOC state machine, cascaded loops |
| `fault` | foc | Protection + error handling; consumes `motor_hw` only. Tunable thresholds live in `fault_cfg_t` (`fault_set_cfg`/`_get_cfg`), guarded/clamped to `*_HARD_*` limits |
| `interrupt` | foc | ISR entry points / control-loop scheduling |
| `vofa` | foc* | Telemetry formatting (see Known debts) |
| `drv8323`, `tle5012b` | driver | Chip drivers; self-contained |
| `uart`, `pwm_curr` | bsp | On-chip peripheral setup + access |
| `flash_port` | bsp | FAL on-chip flash device (GD32 FMC) for FlashDB; UART log sink. Uses SPL only (feeds the watchdog via SPL to avoid a foc dep) |
| `io_stream` | bsp | Transport interface (write/read fn ptrs); `uart_stream()` is the backend |
| `letter-shell` | libraries | Third-party command shell (export-OFF, manual command table) |
| `flashdb` | libraries | Third-party KV store (KVDB-only) over FAL; top 8 KB of flash |
| `shell_port` | app | letter-shell glue: io_stream binding, command table, `scope` mode |
| `params` | app | Flash parameter service: KV store of gains/limits/calibration/thresholds + fault log. Single RAM truth, applied via foc setters + `fault_set_cfg`; writes only in IDLE |
| `main`, `gd32f30x_it` | app | Entry, sequencing, interrupt vector table |

## Dependency rules (red lines)

1. **Only `motor_hw` includes chip/bsp headers** (`drv8323.h`, `tle5012b.h`,
   `pwm_curr.h`). `foc_core` and `fault` go through `motor_hw` (e.g.
   `motor_hw_get_vbus()`, `motor_hw_driver_get_faults()`). The ISR-wiring layer
   (`interrupt.c`, `gd32f30x_it.c`) may reference driver ISR hooks.
2. **No mutable globals in headers** (per STYLE.md). Declare the instance in
   `main.c`, pass by pointer.
3. **No magic numbers / SI units internally** (per STYLE.md).
4. Cross-cutting services (protection, telemetry, CLI) consume abstractions,
   not concrete chips — so swapping a chip touches only `driver/` + `motor_hw`.
5. **Flash writes only while outputs are off.** A flash erase/program stalls the
   CPU and masks IRQs, freezing the 20 kHz control loop for tens of ms. `params`
   writes only in `IDLE` (deferred via a dirty flag + `params_poll_save`); the
   fault-log write is the one exception, allowed on the fault-latch edge because
   the outputs are already braked.

## Periodic review (run every few features)

```sh
# 1. File sizes — anything > ~600 lines is a split candidate
wc -l app/*.c bsp/*.c driver/*/*.c foc/*.c | sort -n

# 2. Chip drivers must only appear in motor_hw + ISR wiring
grep -rl "drv8323_\|tle5012b_" foc app bsp | grep -v "motor_hw.c\|interrupt.c\|gd32f30x_it.c"

# 3. bsp inlines (read_vbus) must not be called above the HAL
grep -rn "read_vbus" foc app | grep -v "motor_hw.c"

# 4. No mutable extern in headers
grep -rn "^extern" foc app bsp | grep -v "const"
```
Any non-empty result from 2–4 is a layering leak to fix.

## Known debts

- **Telemetry coupling**: `vofa` lives in `foc/` and `foc_core` *pushes* to it
  each control cycle (control → telemetry inversion). Invert to a registered
  callback / pull model and move to a service dir when telemetry is reworked.
- **Shared UART**: the shell and VOFA streaming share one UART, so they are
  mutually exclusive — the shell `scope` command switches into binary streaming
  and any key returns to the shell. When USB CDC is added, give VOFA its own
  `io_stream` backend so both run at once (no upper-layer changes needed).
