#include "foc_acog.h"
#include "foc_core.h"
#include <math.h>

/* Step-and-settle calibration via the cascade. See foc_acog.h for the full
 * rationale. The earlier *continuous creep* measured the held iq while the rotor
 * was moving — which fails on this 168-cogging-cycle/rev motor: at any usable
 * creep speed the cogging frequency (≈17 Hz at 6 rpm) sits far above the soft
 * speed-loop bandwidth, so the loop cannot track it and the measured iq is a
 * heavily ATTENUATED, PHASE-LAGGED copy of the true cogging torque -> a weak,
 * misaligned map that does not cancel cogging at run time.
 *
 * The fix is to measure at DC, exactly like the reference firmware: command the
 * position cascade to step to each mechanical-angle bin, let the rotor SETTLE TO
 * REST, then average the held iq. With the rotor stopped the speed-loop integral
 * has no frequency content to chase — it converges to the true static holding
 * torque, so iq == cogging torque at that angle (no attenuation, no phase lag).
 * Forward then reverse: approaching a bin from + vs - puts static friction at
 * its two extremes, so averaging the two passes cancels it (cogging does not).
 * The step is one bin wide, so the cascade reaches each point in tens of ms and
 * a fixed dwell needs no arrival check (that check is what made the old spring
 * take an hour). 2*N steps * STEP_TICKS is a deterministic ~3.5 min total.
 *
 * Tuning (no scope): `creep_rpm` only caps the inter-bin HOP speed (measurement
 * is at rest, so it does not affect accuracy). If the rotor cannot reach/hold a
 * bin, raise the speed integral gain (`set spd_ki ...`); raising creep_rpm just
 * speeds the hops. Do NOT raise the position landing gain for "stiffness". */
#define ACOG_DEFAULT_CREEP_RPM  10.0f      /* inter-bin hop speed cap [rpm mech] */
#define ACOG_DEFAULT_MAXFF      2.0f       /* feed-forward clamp [A] */
#define ACOG_APPROACH_KP        40.0f      /* position landing gain during cal [rad/s per rad].
                                            * Boosted from the ~12 run-time value so the rotor
                                            * settles into a bin's deadband fast (tau=1/kp~25 ms);
                                            * only acts during the hop — at rest the speed
                                            * integral holds, so the higher gain cannot
                                            * oscillate the map. */
#define ACOG_SETTLE_TICKS       1600u      /* ~80 ms hop+settle to rest per step (>3*tau) */
#define ACOG_MEASURE_TICKS      400u       /* ~20 ms iq averaging at rest */
#define ACOG_STEP_TICKS         (ACOG_SETTLE_TICKS + ACOG_MEASURE_TICKS)
#define ACOG_MIN_COVERAGE       (FOC_ACOG_TABLE_SIZE * 3u / 4u)  /* bins that must be hit */
#define ACOG_STEP_RAD           (FOC_2PI / (float)FOC_ACOG_TABLE_SIZE)  /* one bin */
#define ACOG_IDX_SCALE          ((float)FOC_ACOG_TABLE_SIZE / FOC_2PI)
#define ACOG_IDX_MASK           (FOC_ACOG_TABLE_SIZE - 1u)

/* The lookup table and the per-bin sample counts are module-private statics
 * (kept out of motor_t so it stays lean). During calibration g_acog_map doubles
 * as the per-bin iq ACCUMULATOR (summed in place); foc_acog_finish then divides
 * each bin by its count to get the mean, so no separate sum array is needed —
 * that is what keeps the calibration SRAM peak to map(4 KB)+cnt(2 KB). */
static float    g_acog_map[FOC_ACOG_TABLE_SIZE];   /* sum during cal -> map after */
static uint16_t g_acog_cnt[FOC_ACOG_TABLE_SIZE];   /* samples accumulated per bin */

/* Compact persisted form: int8 quantised map + a float scale (map = q*scale).
 * Single static buffer reused for both save (pack) and load (the loader reads
 * into it, then commit unpacks) so neither path needs a ~1 KB stack copy. */
typedef struct {
  uint8_t  valid;
  uint8_t  enabled;
  uint16_t count;                      /* FOC_ACOG_TABLE_SIZE at save time */
  float    scale;                      /* map[i] = q[i] * scale  [A] */
  int8_t   q[FOC_ACOG_TABLE_SIZE];
} acog_blob_t;
static acog_blob_t s_blob;

/* Mechanical-angle bin of the live encoder reading. mech_angle is in [0,2pi), so
 * the masked index is always in range. Same formula as foc_acog_lookup, so the
 * calibration writes a bin exactly where run-time reads it. */
static inline uint32_t acog_bin_of(float mech_angle) {
  float u = mech_angle * ACOG_IDX_SCALE;
  return (uint32_t)u & ACOG_IDX_MASK;
}

void foc_start_acog(motor_t *motor, float creep_rpm, float max_ff) {
  acog_ctx_t *a = &motor->acog;
  uint32_t i;

  /* Clear the accumulator + counts while still in IDLE (the ISR ignores ACOG
   * until the state flips below), so no interrupt masking is needed here. */
  for (i = 0; i < FOC_ACOG_TABLE_SIZE; i++) {
    g_acog_map[i] = 0.0f;
    g_acog_cnt[i] = 0u;
  }

  a->creep_rpm = (creep_rpm > 0.0f) ? creep_rpm : ACOG_DEFAULT_CREEP_RPM;
  a->max_ff    = (max_ff > 0.0f) ? max_ff : ACOG_DEFAULT_MAXFF;
  a->idx       = 0u;
  a->dwell     = 0u;
  a->meas_sum  = 0.0f;
  a->meas_cnt  = 0u;
  a->enabled   = 0u;                 /* off while measuring (no self-compensation) */
  a->phase     = ACOG_FWD;

  /* Walk forward from the live continuous position, one bin per step. rev_base is
   * captured at the FWD->REV switch (where the rotor actually ends up). */
  a->start_pos      = motor->ctrl.pos.current_pos;
  a->rev_base       = a->start_pos;

  /* Save then boost the position-loop profile for fast per-bin settling. max_rpm
   * is re-asserted every tick by foc_acog_run; landing_kp is set once here and
   * restored (with max_rpm) by foc_set_state when ACOG exits. */
  a->saved_max_rpm     = motor->ctrl.pos.max_rpm;
  a->saved_landing_kp  = motor->ctrl.pos.landing_kp;
  motor->ctrl.pos.landing_kp = ACOG_APPROACH_KP;

  foc_set_state(motor, FOC_STATE_ACOG);
}

void foc_acog_run(motor_t *motor) {
  acog_ctx_t *a = &motor->acog;
  pos_controller_t *pos = &motor->ctrl.pos;

  motor->target.id = 0.0f;            /* pure torque current, no d-axis */

  if (a->phase == ACOG_DONE) {
    pos->max_rpm    = a->saved_max_rpm;   /* restore the normal speed profile */
    pos->target_pos = pos->current_pos;   /* hold here; main loop finalises */
    foc_set_state(motor, FOC_STATE_IDLE);
    return;
  }

  /* Command the cascade to this step's bin angle and cap the hop speed. The
   * position loop drives the rotor there and the speed-loop integral then holds
   * it AT REST against cogging — the held iq we measure below is therefore the
   * static (DC) holding torque, free of the bandwidth attenuation that wrecked
   * the moving measurement. One bin per step, so the hop is tiny and fast. */
  pos->max_rpm = a->creep_rpm;
  float base = (a->phase == ACOG_FWD) ? a->start_pos : a->rev_base;
  float dir  = (a->phase == ACOG_FWD) ? 1.0f : -1.0f;
  pos->target_pos = base + dir * (float)a->idx * ACOG_STEP_RAD;

  /* Fixed dwell: hop+settle to rest, then average the held iq over the measure
   * window. No arrival check needed — the step is one bin, the cascade always
   * reaches it within the settle, and the dwell is deterministic. */
  a->dwell++;
  if (a->dwell > ACOG_SETTLE_TICKS) {
    a->meas_sum += motor->actual.iq;
    a->meas_cnt++;
  }
  if (a->dwell < ACOG_STEP_TICKS) {
    return;
  }

  /* Step complete: accumulate the averaged held iq into the bin at the rotor's
   * settled angle (FWD and REV add into the same bins; finish divides by the
   * count, so the two passes average and friction cancels). */
  float avg = (a->meas_cnt > 0u) ? a->meas_sum / (float)a->meas_cnt : motor->actual.iq;
  uint32_t bin = acog_bin_of(motor->sensor.encoder.mech_angle);
  g_acog_map[bin] += avg;
  g_acog_cnt[bin]++;

  a->dwell    = 0u;
  a->meas_sum = 0.0f;
  a->meas_cnt = 0u;
  a->idx++;

  if (a->idx >= FOC_ACOG_TABLE_SIZE) {
    if (a->phase == ACOG_FWD) {
      a->phase    = ACOG_REV;
      a->idx      = 0u;
      a->rev_base = pos->current_pos;   /* walk back from where FWD ended */
    } else {
      a->phase = ACOG_DONE;
    }
  }
}

uint8_t foc_acog_finish(motor_t *motor) {
  acog_ctx_t *a = &motor->acog;
  float mean = 0.0f;
  float peak = 0.0f;
  uint32_t covered = 0u;
  uint32_t i;

  /* Per-bin mean of the accumulated held iq (sum / sample count), over covered
   * bins only (uncovered bins must not drag the DC estimate). */
  for (i = 0; i < FOC_ACOG_TABLE_SIZE; i++) {
    if (g_acog_cnt[i] > 0u) {
      g_acog_map[i] = g_acog_map[i] / (float)g_acog_cnt[i];
      mean += g_acog_map[i];
      covered++;
    } else {
      g_acog_map[i] = 0.0f;
    }
  }
  if (covered == 0u) {
    a->peak = 0.0f;
    a->valid = 0u;
    a->enabled = 0u;
    return 0u;
  }
  mean /= (float)covered;

  /* Remove the DC component (cogging is zero-mean over a revolution; any offset
   * is friction/load that must NOT be fed forward) and clamp. Uncovered bins map
   * to the mean so they centre to ~0 (no feed-forward where there is no data). */
  for (i = 0; i < FOC_ACOG_TABLE_SIZE; i++) {
    float v = (g_acog_cnt[i] > 0u ? g_acog_map[i] : mean) - mean;
    if (v >  a->max_ff) { v =  a->max_ff; }
    if (v < -a->max_ff) { v = -a->max_ff; }
    g_acog_map[i] = v;
    float ab = fabsf(v);
    if (ab > peak) { peak = ab; }
  }

  a->peak = peak;
  if (covered < ACOG_MIN_COVERAGE || peak < 1.0e-4f) {
    a->valid = 0u;            /* sweep did not really turn the rotor */
    a->enabled = 0u;
    return 0u;
  }
  a->valid = 1u;
  a->enabled = 1u;            /* auto-enable a fresh, good map */
  return 1u;
}

float foc_acog_lookup(const motor_t *motor) {
  const acog_ctx_t *a = &motor->acog;
  if (!a->enabled || !a->valid) {
    return 0.0f;
  }
  float u = motor->sensor.encoder.mech_angle * ACOG_IDX_SCALE;
  uint32_t i0 = (uint32_t)u & ACOG_IDX_MASK;
  uint32_t i1 = (i0 + 1u) & ACOG_IDX_MASK;
  float frac = u - (float)(uint32_t)u;
  return g_acog_map[i0] + frac * (g_acog_map[i1] - g_acog_map[i0]);
}

void foc_acog_set_enable(motor_t *motor, uint8_t on) {
  if (motor->acog.valid) {
    motor->acog.enabled = on ? 1u : 0u;
  }
}

void foc_acog_clear(motor_t *motor) {
  motor->acog.valid = 0u;
  motor->acog.enabled = 0u;
  motor->acog.peak = 0.0f;
}

uint8_t foc_acog_is_valid(const motor_t *motor)   { return motor->acog.valid; }
uint8_t foc_acog_is_enabled(const motor_t *motor) { return motor->acog.enabled; }
float   foc_acog_peak(const motor_t *motor)       { return motor->acog.peak; }

const void *foc_acog_blob(const motor_t *motor, uint32_t *len) {
  const acog_ctx_t *a = &motor->acog;
  uint32_t i;

  if (!a->valid) {
    *len = 0u;
    return 0;
  }
  float peak = (a->peak > 1.0e-6f) ? a->peak : 1.0e-6f;
  float scale = peak / 127.0f;

  s_blob.valid   = 1u;
  s_blob.enabled = a->enabled;
  s_blob.count   = (uint16_t)FOC_ACOG_TABLE_SIZE;
  s_blob.scale   = scale;
  for (i = 0; i < FOC_ACOG_TABLE_SIZE; i++) {
    float r = g_acog_map[i] / scale;
    int v = (int)(r + (r >= 0.0f ? 0.5f : -0.5f));
    if (v >  127) { v =  127; }
    if (v < -127) { v = -127; }
    s_blob.q[i] = (int8_t)v;
  }
  *len = (uint32_t)sizeof(s_blob);
  return &s_blob;
}

void *foc_acog_load_buffer(uint32_t *maxlen) {
  *maxlen = (uint32_t)sizeof(s_blob);
  return &s_blob;
}

void foc_acog_load_commit(motor_t *motor, uint32_t len) {
  acog_ctx_t *a = &motor->acog;
  uint32_t i;
  float peak = 0.0f;

  if (len < sizeof(acog_blob_t) || !s_blob.valid ||
      s_blob.count != (uint16_t)FOC_ACOG_TABLE_SIZE) {
    a->valid = 0u;
    a->enabled = 0u;
    return;
  }
  for (i = 0; i < FOC_ACOG_TABLE_SIZE; i++) {
    float v = (float)s_blob.q[i] * s_blob.scale;
    g_acog_map[i] = v;
    float ab = fabsf(v);
    if (ab > peak) { peak = ab; }
  }
  a->peak     = peak;
  a->valid    = 1u;
  a->enabled  = s_blob.enabled;
  a->max_ff   = ACOG_DEFAULT_MAXFF;
}
