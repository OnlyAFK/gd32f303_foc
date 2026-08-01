#ifndef PARAMS_H
#define PARAMS_H

#include <stdint.h>
#include <stdbool.h>
#include "foc_core.h"
#include "fault.h"

/*
 * Persistent parameter storage (FlashDB KVDB, top 8 KB of flash; the 12 KB
 * below it holds the cogging-torque map, see params.c).
 *
 * Single runtime source of truth is `params_t` in RAM. The compile-time
 * DEFAULT_ / HW_ macros in foc_core/motor_hw remain the factory fallback: at
 * boot foc_init() seeds the motor with them, params_init() then captures those
 * as defaults and lets any value found in flash override them.
 *
 * Boot order (main.c):
 *   fault_init() -> foc_init() -> params_init(&motor) [mount+load+apply]
 *
 * Flash is only ever written from the main/background context while the motor
 * is IDLE (see params_poll_save). Calibration results are auto-saved.
 */

typedef struct {
  /* --- calibration (auto-measured, saved on calibration completion) --- */
  uint8_t cali_valid;        /* 1 once the encoder has been aligned + persisted;
                                boot skips the (rotor-twitching) alignment when set */
  float   enc_zero_offset;   /* encoder mech angle (rad) where elec angle = 0 */
  int32_t enc_dir;           /* encoder counting direction: +1 / -1 */

  /* --- current loop gains + limit --- */
  float id_kp, id_ki;
  float iq_kp, iq_ki;
  float cur_limit;           /* symmetric id/iq output + integral clamp (A) */

  /* --- speed loop gains + limit --- */
  float spd_kp, spd_ki;
  float spd_limit;           /* speed-loop output clamp (A) */

  /* --- position profile --- */
  float pos_max_rpm;
  float pos_max_accel;
  float pos_max_jerk;
  float pos_landing_kp;      /* near-target P gain [rad/s per rad] = position bandwidth */
  float pos_deadband;        /* position deadband [rad] */

  /* --- identified motor parameters (measured by FOC_STATE_IDENT) ---
   * Deployed into the current loop via params_tune_current_loop (the `tune`
   * command / autocal). ident_valid is set once the full electrical set is
   * measured. */
  float   rs;                /* stator resistance [ohm] */
  float   ld;                /* d-axis inductance [uH] (1e6*H, for clean display) */
  float   lq;                /* q-axis inductance [uH] */
  float   flux;              /* PM flux linkage [mWb] (1e3*Wb, for clean display) */
  uint8_t ident_valid;       /* 1 once Rs+Ld+Lq+flux have all been identified */

  /* --- identified mechanical parameters (measured by FOC_STATE_IMECH) --- */
  float   inertia_j;         /* rotor-side inertia [g.cm^2] (1e7*kg.m^2) */
  float   visc_b;            /* viscous friction [uNm.s/rad] (1e6*Nm.s/rad) */
  float   friction_tc;       /* Coulomb friction [mNm] (1e3*Nm) */
  uint8_t mech_valid;        /* 1 once J/B/Tc have been identified */

  /* --- protection thresholds (VESC-style guard lives in fault.c) --- */
  fault_cfg_t thr;

  /* --- CAN protocol layer (app/can_proto) --- */
  uint8_t  can_id;           /* this node's CAN address, 1..63 (ID = node<<5 | cmd) */
  uint16_t can_wdog_ms;      /* command-loss watchdog [ms]; 0 = disabled (default) */
} params_t;

/* Mount the KVDB, load stored values over the foc_init() defaults, and push the
 * result into `motor`. Safe to call once, after foc_init(). If the flash mount
 * fails the system keeps running on the in-RAM defaults (no persistence). */
void params_init(motor_t *motor);

/* Access to the live parameter set (shell get/set). Treat as read-only outside
 * params.c — mutations bypass the dirty/save tracking. */
params_t *params_get(void);

/* Flush the entire parameter set RAM -> flash now. Heavy (flash erase/program);
 * call only while the motor is IDLE. Normally reached via params_poll_save. */
void params_save(void);

/* Mark the parameter set changed; the actual flash write is deferred to the
 * next params_poll_save() in IDLE. Call after any shell `set`. */
void params_mark_dirty(void);
bool params_is_dirty(void);

/* Background hook: if dirty and the motor is IDLE, flush to flash. Call from the
 * ~1 ms main loop. Keeps flash writes out of the control ISR and out of any
 * driving state. */
void params_poll_save(motor_t *motor);

/* Capture the motor's current encoder calibration (zero offset + direction)
 * into the parameter set and mark dirty, so a fresh calibration persists. Call
 * on the calibration-done edge. */
void params_capture_calibration(const motor_t *motor);

/* Capture the motor's identified parameters (Rs/Ld/Lq/flux) into the parameter
 * set and mark dirty. Call on the IDENT-done edge (state IDENT -> IDLE). */
void params_capture_ident(const motor_t *motor);

/* Capture the motor's identified mechanical parameters (J/B/Tc) into the
 * parameter set and mark dirty. Call on the IMECH-done edge (IMECH -> IDLE). */
void params_capture_identmech(const motor_t *motor);

/* Persist the cogging-torque map to its dedicated flash pages (or erase them
 * if the map is invalid). Unlike the small fields this writes flash
 * immediately, so call it only with the motor IDLE — e.g. on the COG-done
 * edge (COG -> IDLE) or after the `cog on/off` shell toggle so the enable
 * flag survives a reboot. */
void params_capture_cogging(const motor_t *motor);

/* Persist (or, if the table is invalid, delete) the encoder linearization
 * table as a compact blob KV ("ltu"). Writes flash immediately, so call it
 * only with the motor IDLE — e.g. on the LTU-done edge (LTU -> IDLE) or after
 * the `ltu on/off` shell toggle so the enable flag survives a reboot. */
void params_capture_ltu(const motor_t *motor);

/* Compute and apply the current-loop / speed-loop PI gains from the identified
 * parameters at a closed-loop bandwidth `bw_hz`, persisting them (no-op if the
 * required parameters have not been identified). Current loop uses Rs/Ld/Lq;
 * speed loop uses J/B and the flux-derived torque constant. Shared by the
 * `tune`/`tunespd` shell commands and the autocal orchestrator. */
void params_tune_current_loop(motor_t *motor, float bw_hz);
void params_tune_speed_loop(motor_t *motor, float bw_hz);

/* Restore factory defaults (the foc_init() values captured at params_init),
 * apply them to `motor`, and persist immediately. */
void params_factory_reset(motor_t *motor);

/* Discard unsaved RAM changes: reload every value from flash and re-apply to
 * `motor` (keys absent in flash revert to their default). */
void params_reload(motor_t *motor);

/* Shell get/set by name (the key strings in the field table). `params_set`
 * writes the value, applies it live (PID/limits via foc setters, thresholds via
 * fault_set_cfg with clamping), and marks the set dirty. Both return false if
 * `name` is unknown. Values are exchanged as float (ints/bytes are converted). */
bool params_set(motor_t *motor, const char *name, float value);
bool params_get_value(const char *name, float *out);

/* Iterate the parameter table for a shell listing. */
uint32_t    params_field_count(void);
const char *params_field_name(uint32_t i);
float       params_field_value(uint32_t i);

/* --- Persistent fault log (a small ring of fault snapshots in flash) ---
 *
 * Replaces a TSDB: there is no RTC, so a time-series would carry only a
 * boot-relative timestamp. This keeps the most recent fault context across a
 * power cycle for field post-mortem ("the device tripped, why?").
 *
 * Push is called on the fault-latch edge (state -> ERROR), when the outputs are
 * already off, so the flash write does not disturb an active control loop. */
void params_fault_log_push(const fault_status_t *snap);

/* Number of stored snapshots available (0..ring depth). */
uint32_t params_fault_log_depth(void);

/* Snapshot at position `i`, newest first (0 = most recent). NULL if out of
 * range. */
const fault_status_t *params_fault_log_entry(uint32_t i);

#endif /* PARAMS_H */
