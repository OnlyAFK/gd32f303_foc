#ifndef FOC_ACOG_H
#define FOC_ACOG_H

#include <stdint.h>

/* Cogging-torque (anti-cogging) compensation — a position-indexed feed-forward
 * torque-current map, learned by an offline calibration sweep and added on top
 * of the loop's iq command at run time.
 *
 * Like foc_ident, the calibration is a commissioning-only sub-state-machine that
 * runs inside the FOC control ISR (state FOC_STATE_ACOG) and returns to IDLE
 * when finished; foc_core.h includes this header to compose acog_ctx_t into
 * motor_t. The big lookup/accumulator buffers live as module-private statics in
 * foc_acog.c (not in motor_t) so motor_t stays lean.
 *
 * HOW IT TURNS THE ROTOR. The calibration does NOT bypass the speed loop with a
 * position->current spring (iq = KP*err): a spring has no integrator, so the
 * holding current is tied to the position error — it cannot both hold a precise
 * angle AND develop the steady torque to break a cogging detent, so it
 * stick-slips and, if stiffened, oscillates. Instead it REUSES the existing
 * cascade (position->speed->current), stepping one bin at a time:
 *   - the position loop drives the rotor to each bin angle and the speed-loop
 *     INTEGRAL holds it there; `max_rpm` caps the hop speed so motion stays a
 *     smooth low-speed slew (no runaway/overshoot, no noise amplification);
 *   - the step is one bin, so the cascade always reaches it within a short fixed
 *     dwell — NO per-point arrival check (the gate that made the old spring take
 *     an hour). 2*N bins is a deterministic ~3.5 min total.
 *
 * WHY STEP-AND-SETTLE, NOT A MOVING MEASUREMENT. The held iq is measured only
 * after the rotor SETTLES TO REST at each bin. This motor has 168 cogging
 * cycles/rev, so measuring while moving (even a slow continuous creep) puts the
 * cogging frequency far above the soft speed-loop bandwidth: the loop cannot
 * track it, and the measured iq comes out attenuated and phase-lagged -> a weak,
 * misaligned map that does not cancel cogging. At rest the holding torque is DC,
 * where the integrator has full authority, so iq == the true cogging torque at
 * that angle. Forward then reverse over the SAME bins: approaching a bin from +
 * vs - settles static friction at its two extremes, so averaging the two passes
 * cancels friction and leaves only cogging (a fixed function of mech angle).
 *
 * Why this fits the hardware: the TLE5012B is an *absolute* 15-bit encoder, and
 * cogging is a fixed function of the rotor-vs-stator mechanical angle, so the
 * map is indexed by mech_angle directly (independent of the electrical
 * zero_offset) and stays valid across power cycles.
 *
 * Table size 1024 over 24s/28p (LCM=168 cogging periods/rev) gives ~6 samples
 * per cogging period — enough for the low-speed-smoothness goal with linear
 * interpolation, and small enough (int8+scale blob, ~1 KB) to persist in a
 * single 2 KB-page FlashDB KV ("cogging") alongside the other parameters.
 */

#define FOC_ACOG_TABLE_SIZE   1024u   /* power of two; index = mech_angle bin */

/* See foc_ident.h: guarded to avoid a duplicate typedef when foc_core.h
 * includes both headers. */
#ifndef FOC_MOTOR_T_DEFINED
#define FOC_MOTOR_T_DEFINED
typedef struct motor_s motor_t;   /* full definition lives in foc_core.h */
#endif

/* Calibration phases (only meaningful in FOC_STATE_ACOG). Step-and-settle: the
 * position cascade steps the rotor one bin at a time, lets it come to REST, then
 * averages the held iq (= the static cogging torque at that angle) into the bin.
 * Forward then reverse over the SAME bins so static friction cancels. */
typedef enum {
  ACOG_FWD = 0,   /* step +1 bin/step over one rev, record held iq per bin */
  ACOG_REV,       /* step -1 bin/step back, accumulate into the same bins */
  ACOG_DONE,      /* hand back to IDLE; the map is finalised in the main loop */
} acog_phase_t;

typedef struct {
  acog_phase_t phase;
  uint8_t  enabled;     /* run-time feed-forward on/off (only if valid) */
  uint8_t  valid;       /* map populated (freshly calibrated or loaded) */
  uint16_t idx;         /* current bin/step index 0..FOC_ACOG_TABLE_SIZE-1 */
  float    creep_rpm;   /* inter-bin hop speed cap [rpm] (tunable via `acog cal <rpm>`) */
  float    max_ff;      /* feed-forward clamp [A] */
  float    start_pos;   /* pos.current_pos at the FWD start (step base) */
  float    rev_base;    /* pos.current_pos captured at the FWD->REV switch */
  float    saved_max_rpm;    /* pos controller max_rpm to restore when done */
  float    saved_landing_kp; /* pos controller landing_kp to restore when done */
  float    meas_sum;    /* iq accumulated over the current step's measure window */
  uint16_t meas_cnt;    /* samples in meas_sum */
  float    peak;        /* |map| peak after compute [A] (info / quantiser scale) */
  uint32_t dwell;       /* ticks elapsed in the current step (settle + measure) */
} acog_ctx_t;

/* Start the calibration. creep_rpm > 0 overrides the sweep speed [rpm] for this
 * run (else the default) — the one knob to tune from the shell with no scope;
 * max_ff <= 0 selects the default feed-forward clamp. Requires valid current
 * offsets and an aligned encoder (the caller checks the latter). Clears the map
 * + per-bin counts, then enters FOC_STATE_ACOG. */
void foc_start_acog(motor_t *motor, float creep_rpm, float max_ff);

/* Per-tick hook called from foc_control_loop BEFORE the position loop while in
 * FOC_STATE_ACOG: drives the creep target for this direction, averages the held
 * iq over the measurement window into the live-angle bin, flips forward->reverse
 * once a full rev of bins is covered, and hands back to IDLE when both passes
 * are done. */
void foc_acog_run(motor_t *motor);

/* Finalise the map: divide each covered bin by its sample count, remove the DC
 * component over the covered bins (cogging is zero-mean over a rev; the offset
 * is friction that must NOT be fed forward), leave uncovered bins at zero, clamp
 * to max_ff. Runs in the main loop on the ACOG->IDLE edge (too heavy for the
 * ISR). Returns 1 on success (good coverage), 0 if the sweep failed. */
uint8_t foc_acog_finish(motor_t *motor);

/* Run-time feed-forward lookup: position-indexed torque current [A], linearly
 * interpolated. Returns 0 unless the map is valid AND enabled. Cheap enough for
 * the 20 kHz ISR (two loads + one lerp). */
float foc_acog_lookup(const motor_t *motor);

/* Run-time controls. */
void    foc_acog_set_enable(motor_t *motor, uint8_t on);
void    foc_acog_clear(motor_t *motor);
uint8_t foc_acog_is_valid(const motor_t *motor);
uint8_t foc_acog_is_enabled(const motor_t *motor);
float   foc_acog_peak(const motor_t *motor);

/* Persistence seam (used by app/params). The map is serialised to a compact
 * int8+scale blob owned by foc_acog; params just stores/loads the opaque bytes.
 *  - foc_acog_blob()         packs the live map and returns a pointer + length
 *                            (NULL/len 0 if the map is invalid -> caller deletes).
 *  - foc_acog_load_buffer()  exposes that same internal buffer for the loader to
 *                            read a stored blob into (avoids a ~1 KB stack copy).
 *  - foc_acog_load_commit()  unpacks the just-read buffer into the live map. */
const void *foc_acog_blob(const motor_t *motor, uint32_t *len);
void       *foc_acog_load_buffer(uint32_t *maxlen);
void        foc_acog_load_commit(motor_t *motor, uint32_t len);

#endif /* FOC_ACOG_H */
