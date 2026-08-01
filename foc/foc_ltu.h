#ifndef FOC_LTU_H
#define FOC_LTU_H

#include <stdint.h>

/* Encoder linearization (LTU) — a mechanical-angle-indexed lookup table that
 * cancels the TLE5012B's residual non-linearity (magnet eccentricity / sensor
 * INL, typically a few tenths of a degree), learned by an offline calibration
 * and applied to the raw encoder count at run time.
 *
 * Like foc_ident, the calibration is a commissioning-only sub-state
 * machine that runs inside the FOC control ISR (state FOC_STATE_LTU) and
 * returns to IDLE when finished; foc_core.h includes this header to compose
 * ltu_ctx_t into motor_t. The sample buffer and the live table are module-
 * private statics in foc_ltu.c (kept out of motor_t so it stays lean).
 *
 * HOW THE ERROR IS MEASURED (ported from the dgm reference firmware). The
 * rotor is dragged OPEN-LOOP, stepper-style: a fixed-magnitude voltage vector
 * whose electrical angle theta sweeps at a constant rate, so the rotor tracks
 * a perfectly uniform rotation. The encoder's raw count is compared against
 * the ideal count implied by theta — the difference is the sensor error at
 * that angle. One full mechanical rev CW, then one rev CCW over the SAME
 * points: the open-loop drag angle (constant torque lag) has opposite sign in
 * the two directions, so averaging the two passes cancels it and leaves only
 * the true sensor error. A moving-average FIR one electrical period wide then
 * smooths noise while keeping the per-rev (magnet) error shape.
 *
 * DIRECTION NORMALISATION (the one subtlety). The FOC pipeline applies the
 * encoder direction in the float domain (mech = dir * raw * k), but the LUT
 * must be built and indexed in ONE canonical direction so it stays valid
 * regardless of the direction the alignment happened to detect. Both the
 * calibration and the run-time lookup therefore first map the raw count to
 * the normalised domain:
 *     raw_n = (dir < 0) ? (CPR - raw) mod CPR : raw
 * The table stores the error in normalised counts indexed by raw_n; run time
 * converts the interpolated correction back with  raw_corr = dir * off  and
 * subtracts it from the raw count before the float pipeline sees it.
 *
 * ZERO OFFSET IS UNTOUCHED. The global mean of the measured error equals the
 * (arbitrary) angle between theta=0 and raw_n=0 at calibration start; it is
 * removed so the LUT is zero-mean — the electrical zero_offset from the
 * alignment calibration keeps doing its own job.
 */

/* Table geometry: 128 entries over 32768 counts (15-bit TLE5012B) -> 256
 * counts per entry, indexed by raw_n >> FOC_LTU_SHIFT with the low bits as
 * the interpolation fraction. */
#define FOC_LTU_TABLE_SIZE   128u
#define FOC_LTU_SHIFT        8u

/* See foc_ident.h: guarded to avoid a duplicate typedef when foc_core.h
 * includes both headers. */
#ifndef FOC_MOTOR_T_DEFINED
#define FOC_MOTOR_T_DEFINED
typedef struct motor_s motor_t;   /* full definition lives in foc_core.h */
#endif

/* Calibration phases (only meaningful in FOC_STATE_LTU). The rotor is dragged
 * by an open-loop rotating voltage vector: ramp up, sweep one mechanical rev
 * forward, one rev back over the same sample points, done. */
typedef enum {
  LTU_RAMP = 0,   /* ramp the drag voltage up at theta = 0 (no sampling) */
  LTU_CW,         /* theta sweeps up one mechanical rev, fill the error array */
  LTU_CCW,        /* theta sweeps back down, average into the same points */
  LTU_DONE,       /* hand back to IDLE; the table is built in the main loop */
} ltu_phase_t;

typedef struct {
  ltu_phase_t phase;
  uint8_t  enabled;      /* run-time correction on/off (only if valid) */
  uint8_t  valid;        /* table populated (freshly calibrated or loaded) */
  float    theta;        /* commanded electrical angle (unbounded, rad) */
  float    volts;        /* open-loop drag voltage magnitude [V] */
  float    time;         /* phase-local time base for the sample schedule [s] */
  float    next_sample;  /* next scheduled sample time [s] */
  int32_t  sample_idx;   /* current sample index 0..LTU_TOTAL-1 (CW up/CCW down) */
  uint32_t ramp_cnt;     /* ticks elapsed in LTU_RAMP */
  int16_t  peak;         /* max |table entry| after compute [counts] (info) */
} ltu_ctx_t;

/* Start the calibration. volts > 0 overrides the open-loop drag voltage for
 * this run (else the default); the caller sizes it from the identified Rs
 * (V ~= 3 A * Rs * 1.5). Requires valid current offsets and an aligned
 * encoder (the caller checks the latter — direction must be known before the
 * sweep so the normalised count domain is fixed). Enters FOC_STATE_LTU. */
void foc_start_ltu(motor_t *motor, float volts);

/* Per-tick hook called from foc_control_loop's LTU switch case: advances the
 * commanded angle, outputs the open-loop drag voltage via the vd/vq out-params,
 * and records/averages the raw-count error at each scheduled sample point.
 * Hands back to IDLE (phase LTU_DONE) when both sweeps are complete. */
void foc_ltu_run(motor_t *motor, float *vd_out, float *vq_out);

/* Finalise the table: subtract the global mean (the theta/raw start offset —
 * NOT a sensor property), apply the one-electrical-period FIR, decimate into
 * the 128-entry table and rotate it so entry k aligns with raw_n bin k.
 * Runs in the main loop on the LTU -> IDLE edge (too heavy for the ISR).
 * Returns 1 on success, 0 if the sweep looks invalid (rotor did not follow). */
uint8_t foc_ltu_finish(motor_t *motor);

/* Run-time correction, in raw counts, to SUBTRACT from the next raw encoder
 * word (called by foc_encoder_callback before the angle conversion). Indexes
 * the table with the last processed raw count (one tick stale — under one
 * table bin even at top speed, and the interpolation smooths the rest).
 * Returns 0 unless the table is valid AND enabled, and always 0 during the
 * calibration itself so the old table cannot contaminate the new samples.
 * Pure integer math — cheap enough for the 20 kHz ISR. */
int32_t foc_ltu_correction(const motor_t *motor);

/* Run-time controls. */
void    foc_ltu_set_enable(motor_t *motor, uint8_t on);
void    foc_ltu_clear(motor_t *motor);
uint8_t foc_ltu_is_valid(const motor_t *motor);
uint8_t foc_ltu_is_enabled(const motor_t *motor);
int16_t foc_ltu_peak(const motor_t *motor);

/* Persistence seam (used by app/params). The table is serialised to a compact
 * int16 blob owned by foc_ltu; params just stores/loads the opaque bytes.
 *  - foc_ltu_blob()         packs the live table and returns a pointer + length
 *                           (NULL/len 0 if invalid -> caller deletes the KV).
 *  - foc_ltu_load_buffer()  exposes that same internal buffer for the loader
 *                           to read a stored blob into (no stack copy).
 *  - foc_ltu_load_commit()  unpacks the just-read buffer into the live table. */
const void *foc_ltu_blob(const motor_t *motor, uint32_t *len);
void       *foc_ltu_load_buffer(uint32_t *maxlen);
void        foc_ltu_load_commit(motor_t *motor, uint32_t len);

#endif /* FOC_LTU_H */
