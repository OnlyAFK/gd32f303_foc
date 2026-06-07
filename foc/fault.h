#ifndef FAULT_H
#define FAULT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

// Central protection / fault-handling module for the GD32F303 + DRV8323 stage.
//
// Three response layers feed this module:
//   L1  nFAULT EXTI (driver/drv8323): cuts PWM in hardware, sets a pending flag.
//   L2  fault_check_fast(): called every PWM cycle from the FOC control loop for
//       the protections that must react within microseconds (software
//       overcurrent, bus overvoltage, DRV nFAULT pin).
//   L3  fault_monitor_slow(): called from the ~1 ms background loop for the
//       slower / debounced detectors (DRV register decode, bus undervoltage,
//       I^2 t overload, stall, overspeed, phase loss, encoder).
//
// Faults are a bitmask so several can be reported at once. `active` is the live
// set; `latched` accumulates everything seen since the last successful clear
// (handy for post-mortem over UART). A context snapshot (DRV status registers,
// bus voltage, iq, timestamp) is captured on the no-fault -> faulted edge.

typedef enum {
  FAULT_NONE             = 0u,

  /* --- DRV8323 hardware faults (decoded from Fault Status 1/2 over SPI) --- */
  FAULT_DRV_VDS_OCP      = (1u << 0),   /* VDS overcurrent (MOSFET hard short) */
  FAULT_DRV_SEN_OCP      = (1u << 1),   /* sense-resistor overcurrent */
  FAULT_DRV_GDF          = (1u << 2),   /* gate-drive fault (gate stuck) */
  FAULT_DRV_UVLO         = (1u << 3),   /* VM undervoltage lockout (chip ~5.6 V) */
  FAULT_DRV_CPUV         = (1u << 4),   /* charge-pump undervoltage */
  FAULT_DRV_OTSD         = (1u << 5),   /* overtemperature shutdown */
  FAULT_DRV_OTW          = (1u << 6),   /* overtemperature warning */
  FAULT_DRV_COMM         = (1u << 7),   /* SPI read-back / verify mismatch */

  /* --- MCU software-detected faults --- */
  FAULT_BUS_OV           = (1u << 8),   /* bus overvoltage */
  FAULT_BUS_UV           = (1u << 9),   /* bus undervoltage (battery protect) */
  FAULT_OVERCURRENT_SW   = (1u << 10),  /* software instantaneous overcurrent */
  FAULT_OVERLOAD_I2T     = (1u << 11),  /* I^2 t winding thermal overload */
  FAULT_MOTOR_OVERTEMP   = (1u << 12),  /* motor NTC (interface reserved) */
  FAULT_BOARD_OVERTEMP   = (1u << 13),  /* board NTC (interface reserved) */
  FAULT_PHASE_LOSS       = (1u << 14),  /* open phase / motor disconnected */
  FAULT_STALL            = (1u << 15),  /* rotor stalled under torque */
  FAULT_OVERSPEED        = (1u << 16),  /* speed over mechanical limit */
  FAULT_ENCODER          = (1u << 17),  /* encoder bus / reading fault */
  FAULT_WATCHDOG         = (1u << 18),  /* recovered from a watchdog reset */
} fault_code_t;

// Faults that must NOT auto-recover: they point at a hardware problem and
// require the condition to be physically resolved plus an explicit clear.
#define FAULT_MASK_LATCHING  (FAULT_DRV_VDS_OCP | FAULT_DRV_SEN_OCP |      \
                              FAULT_DRV_GDF     | FAULT_DRV_COMM    |      \
                              FAULT_PHASE_LOSS  | FAULT_ENCODER     |      \
                              FAULT_WATCHDOG)

/* ===================================================================
 * Protection thresholds.
 * Tuned for: 24 V bus, 5 mOhm shunt, 20 V/V CSA (current sense saturates at
 * ~+/-16.5 A), motor rated 2.9 A / stall 10.36 A, MOSFET Rds(on) 7.34 mOhm
 * typ / 9.0 mOhm max. The DRV hardware VDS/SEN OCP thresholds live in
 * driver/drv8323.c; these are the MCU-side software limits.
 * =================================================================== */

/* Bus voltage (V). OV must trip fast (regen kickback); UV is debounced so a
 * stall/inrush sag does not nuisance-trip. */
#define FAULT_VBUS_OV_TRIP    30.0f
#define FAULT_VBUS_OV_WARN    28.0f
#define FAULT_VBUS_UV_TRIP    18.0f
#define FAULT_VBUS_UV_WARN    20.0f

/* Software overcurrent: per-phase peak (A). Below the +/-16.5 A ADC range and
 * above the 10.36 A stall current so a legitimate stall does not nuisance-trip. */
#define FAULT_OC_SW_PEAK      14.0f

/* I^2 t overload: integrate max(0, I^2 - I_cont^2) over time; trip when the
 * accumulated budget is exceeded. I_cont = continuous rating + headroom. The
 * budget allows roughly stall current (10.36 A) for ~2 s before tripping. */
#define FAULT_I_CONT          3.5f
#define FAULT_I2T_BUDGET      200.0f   /* A^2 * s */

/* Stall: large torque current while the rotor is barely moving, sustained. */
#define FAULT_STALL_IQ        9.0f     /* A */
#define FAULT_STALL_RPM       30.0f    /* |rpm| below this counts as "stopped" */
#define FAULT_STALL_MS        1000u

/* Overspeed (mechanical RPM). */
#define FAULT_OVERSPEED_RPM   2500.0f

/* Phase loss: commanding real torque current but the measured phase currents
 * stay near zero -> open winding / disconnected motor. Gated on a meaningful
 * current command so it never trips while idle / holding a no-load position. */
#define FAULT_PHASELOSS_IQCMD 1.0f     /* A, minimum commanded iq to arm the test */
#define FAULT_PHASELOSS_I     0.3f     /* A, measured "near zero" */
#define FAULT_PHASELOSS_MS    500u

/* Debounce windows for the 1 ms background monitor (in ms). */
#define FAULT_UV_DEBOUNCE_MS  200u
#define FAULT_OV_DEBOUNCE_MS  20u

/* Independent watchdog timeout (ms). The background loop must feed it faster
 * than this or the MCU resets (and PWM stops). */
#define FAULT_WDG_TIMEOUT_MS  200u

/* ===================================================================
 * Runtime-tunable thresholds.
 *
 * The subset of thresholds above that an operator may retune at runtime live in
 * `fault_cfg_t` (stored in flash via app/params). The #define values above are
 * their compile-time DEFAULTS; the remaining #defines (timing / debounce /
 * phase-loss / warn levels) stay compile-time only.
 *
 * VESC-style guard: while `thr_protect_en` is set, any value handed to
 * fault_set_cfg() is clamped to the *_HARD_MIN/MAX bounds below — these encode
 * the real silicon limits (ADC saturation, DRV UVLO, MOSFET rating) and are
 * never user-settable. Clearing the flag lets an expert set raw values.
 * =================================================================== */
#define FAULT_OC_SW_HARD_MAX      16.0f    /* current sense saturates at ~+/-16.5 A */
#define FAULT_OC_SW_HARD_MIN      1.0f
#define FAULT_VBUS_OV_HARD_MAX    32.0f    /* above this the stage is at risk */
#define FAULT_VBUS_OV_HARD_MIN    24.0f    /* OV trip must stay above the nominal bus */
#define FAULT_VBUS_UV_HARD_MAX    22.0f    /* UV trip must stay below the nominal bus */
#define FAULT_VBUS_UV_HARD_MIN    6.0f     /* DRV UVLO ~5.6 V */
#define FAULT_I_CONT_HARD_MAX     10.0f
#define FAULT_I_CONT_HARD_MIN     0.5f
#define FAULT_I2T_HARD_MAX        2000.0f
#define FAULT_I2T_HARD_MIN        1.0f
#define FAULT_STALL_IQ_HARD_MAX   14.0f    /* <= software OC peak */
#define FAULT_STALL_IQ_HARD_MIN   1.0f
#define FAULT_STALL_RPM_HARD_MAX  200.0f
#define FAULT_STALL_RPM_HARD_MIN  5.0f
#define FAULT_OVERSPEED_HARD_MAX  5000.0f
#define FAULT_OVERSPEED_HARD_MIN  100.0f

typedef struct {
  uint8_t thr_protect_en;   /* 1: clamp the values below to *_HARD_MIN/MAX */
  float vbus_ov_trip;       /* bus overvoltage trip (V) */
  float vbus_uv_trip;       /* bus undervoltage trip (V) */
  float oc_sw_peak;         /* software per-phase overcurrent peak (A) */
  float i_cont;             /* I^2 t continuous current (A) */
  float i2t_budget;         /* I^2 t budget (A^2 * s) */
  float stall_iq;           /* stall torque-current threshold (A) */
  float stall_rpm;          /* |rpm| below this counts as stopped */
  float overspeed_rpm;      /* mechanical overspeed trip (RPM) */
} fault_cfg_t;

// Replace the live protection thresholds. With thr_protect_en set, values are
// clamped to the *_HARD_MIN/MAX bounds before taking effect. IRQ-safe (the fast
// check reads the config from the control ISR).
void fault_set_cfg(const fault_cfg_t *cfg);

// The live (already-clamped) threshold configuration.
const fault_cfg_t *fault_get_cfg(void);

typedef struct {
  uint32_t active;          /* faults asserted right now */
  uint32_t latched;         /* every fault seen since the last clear */
  uint32_t first;           /* the code(s) of the no-fault -> fault edge (root cause) */
  uint16_t drv_fs1;         /* Fault Status 1 snapshot at the trip */
  uint16_t drv_fs2;         /* Fault Status 2 snapshot at the trip */
  float    vbus_at_fault;
  float    iq_at_fault;
  uint32_t tick_at_fault;
} fault_status_t;

// Initialise the module: record the reset cause (latches FAULT_WATCHDOG if the
// last reset came from the independent watchdog) and zero the fault state. Call
// once at startup, before foc_init(). Does NOT start the watchdog yet — see
// fault_watchdog_start().
void fault_init(void);

// Start the independent watchdog. Call AFTER the blocking power-up calibration
// (which can take >1 s and would otherwise trip the watchdog), just before the
// main control loop begins feeding it.
void fault_watchdog_start(void);

// OR `code` into active+latched (IRQ-safe). Captures a context snapshot the
// first time the system transitions from no-fault to faulted.
void fault_raise(uint32_t code, float vbus, float iq);

// True if any fault is currently asserted.
bool fault_any_active(void);

const fault_status_t *fault_get(void);

// L2 fast check (per PWM cycle). Returns true if a NEW protection tripped this
// call, so the caller can latch the FOC state machine into ERROR. `driving` is
// false when the outputs are off (IDLE/ERROR) to skip current-based checks.
bool fault_check_fast(float iu, float iv, float iw, float vbus, float iq,
                      bool driving);

// L3 slow monitor (~1 ms). Decodes the DRV8323 status registers when an nFAULT
// edge is pending or the pin is still low, runs the debounced bus checks, the
// I^2 t accumulator, and the stall / overspeed / phase-loss / encoder detectors.
// `iq_cmd` is the commanded torque current (used to arm phase-loss detection);
// `tick_ms` is a free-running millisecond counter supplied by the caller.
void fault_monitor_slow(float vbus, float iq, float iq_cmd, float rpm,
                        float iu, float iv, bool driving, uint32_t tick_ms);

// Attempt to clear all faults. Returns true only if every active condition has
// physically cleared (bus back in range, DRV nFAULT released, etc.); latching
// faults additionally require their condition to be gone. On success the DRV
// configuration is re-applied, its latched bits are cleared, and `active` is
// zeroed (the `latched` history is preserved for diagnostics).
bool fault_try_clear(float vbus);

// Clear the latched fault history (diagnostics). Keeps currently-active faults
// reflected in `latched`; does not touch `active`.
void fault_clear_latched(void);

// Feed the independent watchdog. Call from the main loop.
void fault_watchdog_feed(void);

#ifdef __cplusplus
}
#endif

#endif /* FAULT_H */
