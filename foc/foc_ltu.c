#include "foc_ltu.h"
#include "foc_core.h"

/* Open-loop sweep linearization, ported from the dgm reference firmware — see
 * foc_ltu.h for the measurement principle and the direction-normalisation
 * scheme. The rotor is dragged one mechanical rev each way at a constant
 * electrical rate, so each pass takes  2*pi*pp / LTU_PHASE_VEL  seconds
 * (28 s per direction at the defaults with 14 pole pairs). */

/* Electrical sweep rate. Slower = smaller drag angle and cleaner tracking,
 * but the CW/CCW average cancels the drag anyway; pi rad/s is the value the
 * reference firmware proves in production. */
#define LTU_PHASE_VEL        FOC_PI                 /* elec rad/s */
#define LTU_SAMPLES_PER_PPAIR 128u                  /* samples per elec rev */
#define LTU_TOTAL_SAMPLES    ((int32_t)LTU_SAMPLES_PER_PPAIR * (int32_t)HW_MOTOR_POLE_PAIRS)
#define LTU_SAMPLE_PERIOD_S  (FOC_2PI / ((float)LTU_SAMPLES_PER_PPAIR * LTU_PHASE_VEL))
#define LTU_FIR_WINDOW       ((int32_t)LTU_SAMPLES_PER_PPAIR)  /* one elec rev */
#define LTU_RAMP_TICKS       10000u                 /* ~0.5 s drag-voltage ramp */
#define LTU_DEFAULT_VOLTS    2.0f                   /* drag voltage fallback */

/* Sample buffer: raw-count error per sample point, CW pass first, then the
 * CCW pass averages into the same slots (cancels the open-loop drag angle).
 * 128 * 14 int32 = 7 KB static — the calibration-only SRAM cost. */
static int32_t s_err[LTU_TOTAL_SAMPLES];

/* The live table: normalised-count error per 256-count bin (zero-mean). */
static int16_t s_lut[FOC_LTU_TABLE_SIZE];

/* Persisted form, one static buffer shared by the save (pack) and load
 * (commit unpacks what the loader read into it) paths — no stack copies. */
typedef struct {
  uint8_t  valid;
  uint8_t  enabled;
  uint16_t count;                      /* FOC_LTU_TABLE_SIZE at save time */
  int16_t  peak;                       /* max |entry| [counts] */
  int16_t  lut[FOC_LTU_TABLE_SIZE];
} ltu_blob_t;
static ltu_blob_t s_blob;

/* Raw count mapped into the canonical (direction-normalised) domain — the
 * ONLY domain the table is built and indexed in, so a direction flip from a
 * later re-alignment cannot silently reverse the table. */
static int32_t ltu_normalised_raw(const motor_t *motor) {
  const encoder_data_t *e = &motor->sensor.encoder;
  if (e->direction < 0) {
    return ((int32_t)HW_ENCODER_CPR - (int32_t)e->raw_angle) &
           (int32_t)(HW_ENCODER_CPR - 1u);
  }
  return (int32_t)e->raw_angle;
}

void foc_start_ltu(motor_t *motor, float volts) {
  ltu_ctx_t *c = &motor->ltu;

  /* The error array needs no clearing: the CW pass overwrites every slot
   * before the CCW pass reads it. The old table stays valid/enabled — the
   * correction bypasses itself while in FOC_STATE_LTU, so the new samples
   * are taken against the true raw count, and an aborted sweep keeps the
   * previous table usable. */
  c->volts      = (volts > 0.0f) ? volts : LTU_DEFAULT_VOLTS;
  c->theta      = 0.0f;
  c->time       = 0.0f;
  c->next_sample = 0.0f;
  c->sample_idx = 0;
  c->ramp_cnt   = 0u;
  c->phase      = LTU_RAMP;

  foc_set_state(motor, FOC_STATE_LTU);
}

void foc_ltu_run(motor_t *motor, float *vd_out, float *vq_out) {
  ltu_ctx_t *c = &motor->ltu;

  switch (c->phase) {
  case LTU_RAMP:
    /* Ease the drag voltage up at theta = 0 so the rotor settles onto the
     * start angle instead of snapping to it under full voltage (same idea as
     * the alignment current ramp in CALI). No sampling here. */
    c->ramp_cnt++;
    *vq_out = c->volts * (float)c->ramp_cnt / (float)LTU_RAMP_TICKS;
    if (c->ramp_cnt >= LTU_RAMP_TICKS) {
      c->phase       = LTU_CW;
      c->sample_idx  = 0;
      c->time        = 0.0f;
      c->next_sample = 0.0f;   /* first sample on the first CW tick */
    }
    break;

  case LTU_CW:
  case LTU_CCW: {
    *vq_out = c->volts;
    float sgn = (c->phase == LTU_CW) ? 1.0f : -1.0f;
    c->time  += HW_CONTROL_PERIOD_S;
    c->theta += sgn * LTU_PHASE_VEL * HW_CONTROL_PERIOD_S;

    if (c->time < c->next_sample) {
      break;
    }
    c->next_sample += LTU_SAMPLE_PERIOD_S;

    /* Ideal count implied by the commanded angle vs the measured one. theta
     * sweeps exactly pp electrical revs = one mechanical rev per pass, so
     * count_ref spans 0..CPR; folding the difference to [0, CPR) removes the
     * wrap and the (arbitrary) start-angle offset survives as a constant. */
    int32_t count_ref = (int32_t)(c->theta * (float)HW_ENCODER_CPR /
                                  (FOC_2PI * (float)HW_MOTOR_POLE_PAIRS));
    int32_t err = ltu_normalised_raw(motor) - count_ref;
    if (err < 0) {
      err += (int32_t)HW_ENCODER_CPR;
    }

    if (c->phase == LTU_CW) {
      s_err[c->sample_idx] = err;
      c->sample_idx++;
      if (c->sample_idx >= LTU_TOTAL_SAMPLES) {
        /* Reverse: retrace the same points top-down, averaging in place. */
        c->phase       = LTU_CCW;
        c->sample_idx  = LTU_TOTAL_SAMPLES - 1;
        c->time        = 0.0f;
        c->next_sample = 0.0f;
      }
    } else {
      s_err[c->sample_idx] = (s_err[c->sample_idx] + err) / 2;
      if (c->sample_idx == 0) {
        c->phase = LTU_DONE;
      } else {
        c->sample_idx--;
      }
    }
    break;
  }

  case LTU_DONE:
  default:
    /* Leave the vd/vq outs at 0 (outputs already off) and hand back. */
    foc_set_state(motor, FOC_STATE_IDLE);
    break;
  }
}

uint8_t foc_ltu_finish(motor_t *motor) {
  ltu_ctx_t *c = &motor->ltu;
  int32_t i, j;

  /* Sanity: if the rotor stalled, err = (const - count_ref) mod CPR — a
   * full-scale sawtooth. A genuine measurement spans only the sensor's real
   * error (hundreds of counts at most), so reject a near-full-scale spread. */
  int32_t emin = s_err[0];
  int32_t emax = s_err[0];
  int64_t sum = 0;
  for (i = 0; i < LTU_TOTAL_SAMPLES; i++) {
    if (s_err[i] < emin) { emin = s_err[i]; }
    if (s_err[i] > emax) { emax = s_err[i]; }
    sum += s_err[i];
  }
  if ((emax - emin) > (int32_t)(HW_ENCODER_CPR / 2u)) {
    c->valid   = 0u;
    c->enabled = 0u;
    return 0u;
  }

  /* The global mean is the angle between theta=0 and raw_n=0 at the sweep
   * start — a property of the procedure, not of the sensor. Remove it so the
   * table is zero-mean and the alignment zero_offset keeps its own meaning. */
  int32_t mean = (int32_t)(sum / LTU_TOTAL_SAMPLES);

  /* FIR one electrical period wide (keeps the per-rev magnet error, smooths
   * commutation-frequency noise), decimate TOTAL -> TABLE entries, and rotate
   * the table by the dominant (sample-0) offset so the entry indexed by
   * (raw_n >> SHIFT) holds the error measured at that raw position. */
  int32_t lut_offset =
      (int32_t)((int64_t)s_err[0] * (int32_t)FOC_LTU_TABLE_SIZE / (int32_t)HW_ENCODER_CPR);
  int16_t peak = 0;
  for (i = 0; i < (int32_t)FOC_LTU_TABLE_SIZE; i++) {
    int64_t acc = 0;
    for (j = -LTU_FIR_WINDOW / 2; j < LTU_FIR_WINDOW / 2; j++) {
      int32_t idx = i * (LTU_TOTAL_SAMPLES / (int32_t)FOC_LTU_TABLE_SIZE) + j;
      if (idx < 0) {
        idx += LTU_TOTAL_SAMPLES;
      } else if (idx >= LTU_TOTAL_SAMPLES) {
        idx -= LTU_TOTAL_SAMPLES;
      }
      acc += s_err[idx];
    }
    int32_t v = (int32_t)(acc / LTU_FIR_WINDOW) - mean;
    if (v >  32767) { v =  32767; }
    if (v < -32768) { v = -32768; }
    int32_t li = lut_offset + i;
    if (li >= (int32_t)FOC_LTU_TABLE_SIZE) {
      li -= (int32_t)FOC_LTU_TABLE_SIZE;
    }
    s_lut[li] = (int16_t)v;
    int32_t av = (v >= 0) ? v : -v;
    if (av > (int32_t)peak) {
      peak = (int16_t)av;
    }
  }

  c->peak    = peak;
  c->valid   = 1u;
  c->enabled = 1u;            /* auto-enable a fresh table */
  return 1u;
}

int32_t foc_ltu_correction(const motor_t *motor) {
  const ltu_ctx_t *c = &motor->ltu;
  if (!c->valid || !c->enabled || motor->state == FOC_STATE_LTU) {
    return 0;
  }

  /* Two table reads + one lerp in the normalised domain, then map the
   * correction back to the raw domain: with raw = dir*raw_n (mod CPR) the
   * raw-domain delta of a normalised-domain correction is dir * off. */
  int32_t raw_n = ltu_normalised_raw(motor);
  uint32_t i0 = (uint32_t)raw_n >> FOC_LTU_SHIFT;
  uint32_t i1 = (i0 + 1u) & (FOC_LTU_TABLE_SIZE - 1u);
  int32_t frac = raw_n & ((1 << FOC_LTU_SHIFT) - 1);
  int32_t off = (int32_t)s_lut[i0] +
                (((int32_t)s_lut[i1] - (int32_t)s_lut[i0]) * frac >> FOC_LTU_SHIFT);
  return (motor->sensor.encoder.direction < 0) ? -off : off;
}

void foc_ltu_set_enable(motor_t *motor, uint8_t on) {
  if (motor->ltu.valid) {
    motor->ltu.enabled = on ? 1u : 0u;
  }
}

void foc_ltu_clear(motor_t *motor) {
  motor->ltu.valid   = 0u;
  motor->ltu.enabled = 0u;
  motor->ltu.peak    = 0;
}

uint8_t foc_ltu_is_valid(const motor_t *motor)   { return motor->ltu.valid; }
uint8_t foc_ltu_is_enabled(const motor_t *motor) { return motor->ltu.enabled; }
int16_t foc_ltu_peak(const motor_t *motor)       { return motor->ltu.peak; }

const void *foc_ltu_blob(const motor_t *motor, uint32_t *len) {
  const ltu_ctx_t *c = &motor->ltu;
  uint32_t i;

  if (!c->valid) {
    *len = 0u;
    return 0;
  }
  s_blob.valid   = 1u;
  s_blob.enabled = c->enabled;
  s_blob.count   = (uint16_t)FOC_LTU_TABLE_SIZE;
  s_blob.peak    = c->peak;
  for (i = 0u; i < FOC_LTU_TABLE_SIZE; i++) {
    s_blob.lut[i] = s_lut[i];
  }
  *len = (uint32_t)sizeof(s_blob);
  return &s_blob;
}

void *foc_ltu_load_buffer(uint32_t *maxlen) {
  *maxlen = (uint32_t)sizeof(s_blob);
  return &s_blob;
}

void foc_ltu_load_commit(motor_t *motor, uint32_t len) {
  ltu_ctx_t *c = &motor->ltu;
  uint32_t i;

  if (len < sizeof(ltu_blob_t) || !s_blob.valid ||
      s_blob.count != (uint16_t)FOC_LTU_TABLE_SIZE) {
    c->valid   = 0u;
    c->enabled = 0u;
    return;
  }
  for (i = 0u; i < FOC_LTU_TABLE_SIZE; i++) {
    s_lut[i] = s_blob.lut[i];
  }
  c->peak    = s_blob.peak;
  c->valid   = 1u;
  c->enabled = s_blob.enabled;
}
