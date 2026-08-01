#include "foc_cogging.h"
#include "foc_core.h"
#include "filter.h"
#include <math.h>

/* Two-pass iterative slow-march calibration, evolved from the dgm reference
 * firmware (Firmware_app, Source/anticogging.c) — see foc_cogging.h for the
 * table-geometry rationale. Three improvements over the first continuous-march
 * cut, all aimed at the dominant error source of this joint actuator (stiff
 * cogging detents + planetary-gearbox stiction = stick-slip):
 *
 * 1. THE MARCH RUNS AT 1.5 rpm, NOT 0.5 rpm. At 0.5 rpm the rotor sat deep in
 *    the Stribeck (negative-slope friction) region where stick-slip is worst;
 *    industrial drives learn their cogging tables at 1-5 rpm for exactly this
 *    reason (Kollmorgen recommends 1 rpm, Siemens faults above 5 rpm). The
 *    validity condition is unchanged: at 1.5 rpm the 168 cogging periods/rev
 *    produce a ~4.2 Hz disturbance, still below the ~10 Hz speed-loop
 *    bandwidth, so the held torque command tracks the true cogging torque with
 *    no attenuation. The creep cap default drops 10 -> 2 rpm so a stiction
 *    break-loose can never fling the rotor fast (bounded inertia torque).
 *
 * 2. THE SAMPLED QUANTITY IS THE SPEED-LOOP OUTPUT (target.iq), NOT THE
 *    MEASURED iq. With feed-forward off (pass 1) the PI output IS the loop's
 *    disturbance estimate — the same quantity ODrive samples
 *    (vel_integrator_torque) — and it is far cleaner than 20 kHz measured
 *    current. More importantly it is the ONLY correct thing to sample in
 *    pass 2: physics demands the same actual current whether or not FF is
 *    active, so measured iq cannot see the residual — but the speed-loop
 *    output shrinks to exactly the uncompensated remainder once FF carries
 *    the cogging.
 *
 * 3. TWO PASSES WITH LIVE FEED-FORWARD (iterative learning, like ODrive's
 *    "iteratively builds a map"). Pass 1 (CW+CCW) builds a rough map the same
 *    way as before (CW writes, CCW averages -> direction-symmetric friction
 *    cancels). COG_MID then removes its DC component chunk-wise and ENABLES it
 *    as live feed-forward. Pass 2 repeats both directions with the detents
 *    already pre-cancelled: the rotor sticks far less, and the residual loop
 *    output is added into the map in place (+residual/2 per direction, so
 *    friction still cancels and the residual is added exactly once). A `stop`
 *    during pass 2 loses nothing: the pass-1 map was committed valid at MID.
 *
 * Samples are still binned by the ACTUAL angle (never the commanded one), so
 * tracking lag and lurches can never misalign a sample, and each bin keeps
 * the freshest (most settled) value seen inside it. A VELOCITY GATE drops
 * samples taken while the rotor moves faster than 2x the march rate or moves
 * backwards — those windows carry inertia + P-term torque, not cogging. The
 * gate rarely fires at the default 2 rpm creep cap (the cap itself bounds the
 * lurch); it protects runs where the user raises `cog cal <rpm>` for a
 * stickier gearbox.
 *
 * Total time ~3 min: 4 sampling revs x 40 s + 4 short warm-ups. Tuning (no
 * scope): `creep_rpm` is only the catch-up speed cap; if the rotor cannot
 * follow the march at all, raise the drag current (`cog cal <rpm> <drag_A>`)
 * and/or the speed integral gain (`set spd_ki ...`).
 *
 * DRAG FEED-FORWARD (why the march does not stall). A position->speed cascade
 * with modest gains breaks gearbox stiction only by integrator wind-up — at
 * the deployed spd_ki that is ~0.045 A/s, and with 168 detents/rev the rotor
 * parked at nearly every one: a nominally 3 min sweep ran past 30 min on the
 * bench. The fix is a direction-signed bias current added to the speed-loop
 * output (target.iq) for the whole sweep: the drag does the heavy lifting
 * against friction so the loop only trims, and the rotor follows the march
 * continuously instead of stick-slipping. It does NOT corrupt the map: the
 * sampled quantity (target.iq) includes the drag, so pass 1 still measures
 * cogging+friction+drag and pass 2 the residual+drag, and being direction-
 * signed the drag cancels in the CW/CCW average exactly like friction. The
 * leash and velocity gate below remain as safety nets for a drag setting
 * that is still too small. */
#define COG_DEFAULT_CREEP_RPM  2.0f       /* rotor speed cap during the march [rpm] */
#define COG_DEFAULT_MAXFF      2.0f       /* feed-forward clamp [A] */
#define COG_DEFAULT_DRAG_A     0.3f       /* march drag feed-forward [A] (`cog cal <rpm> <drag_A>`) */
#define COG_MAX_DRAG_A         2.0f       /* drag clamp [A] */
#define COG_APPROACH_KP        40.0f      /* position landing gain during cal [rad/s per rad] */
#define COG_MARCH_RPM          1.5f       /* march rate [rpm]: 40 s/rev at 20 kHz */
#define COG_REV_TICKS          800000u    /* march duration per rev: 40 s at 20 kHz (= 1.5 rpm) */
#define COG_MARCH_TICKS        (COG_REV_TICKS / FOC_COG_TABLE_SIZE)  /* ticks per bin (~195) */
#define COG_BIN_RATE           (FOC_2PI / (float)COG_REV_TICKS)      /* target advance per tick [rad] */
#define COG_WARMUP_BINS        100u       /* un-sampled march bins at each direction change (~1 s) */
#define COG_MIN_COVERAGE       (FOC_COG_TABLE_SIZE * 3u / 4u)  /* bins that must be hit (final) */
#define COG_MID_MIN_COVERAGE   (FOC_COG_TABLE_SIZE / 2u)     /* pass-1 bins needed to be worth a pass 2 */
#define COG_MID_CHUNK          512u       /* bins processed per tick in COG_MID */
#define COG_GATE_FAST_MULT     2.0f       /* drop samples above 2x the march rate */
#define COG_GATE_REV_MULT      0.5f       /* drop samples moving backwards above 0.5x march */
#define COG_LEASH_BINS         30u        /* march waits when the rotor lags this many bins */
#define COG_LEASH_RAD          ((float)COG_LEASH_BINS * (FOC_2PI / (float)FOC_COG_TABLE_SIZE))
#define COG_IDX_SCALE          ((float)FOC_COG_TABLE_SIZE / FOC_2PI)
#define COG_IDX_MASK           (FOC_COG_TABLE_SIZE - 1u)
#define COG_COV_BYTES          (FOC_COG_TABLE_SIZE / 8u)
#define COG_IQ_LPF_ALPHA       0.005f     /* cmd pre-filter, ~10 ms at 20 kHz (dgm uses 0.01) */

/* The map is int16 fixed point at FOC_COG_SCALE counts/A (same quantisation as
 * the reference firmware): 8 KB static. The coverage bitmap marks bins written
 * by the first CW pass (so the first CCW pass can average into them and
 * finish() can skip unwritten bins). */
static int16_t g_cog_map[FOC_COG_TABLE_SIZE];
static uint8_t g_cog_cov[COG_COV_BYTES];
static lpf_t   g_iq_lpf;

/* Mechanical-angle bin of a reading, nearest point — the SAME rounding is used
 * by the calibration write and the run-time lookup, so a bin is read exactly
 * where it was written. mech_angle is in [0,2pi), so the masked index is
 * always in range. */
static inline uint32_t cog_bin_of(float mech_angle) {
  return (uint32_t)(mech_angle * COG_IDX_SCALE + 0.5f) & COG_IDX_MASK;
}

static inline void cov_set(uint32_t bin) { g_cog_cov[bin >> 3] |= (uint8_t)(1u << (bin & 7u)); }
static inline uint8_t cov_get(uint32_t bin) {
  return (uint8_t)((g_cog_cov[bin >> 3] >> (bin & 7u)) & 1u);
}

/* Phase classification. */
static inline uint8_t phase_is_sampling(cog_phase_t p) {
  return (uint8_t)(p == COG_CW1 || p == COG_CCW1 || p == COG_CW2 || p == COG_CCW2);
}
static inline float phase_dir(cog_phase_t p) {
  return (p == COG_WARMUP_CW || p == COG_CW1 || p == COG_WARMUP_CW2 || p == COG_CW2)
             ? 1.0f : -1.0f;
}

/* Bins marched in the current phase (warm-up vs full-rev sampling). */
static inline uint16_t phase_bins(cog_phase_t p) {
  return phase_is_sampling(p) ? (uint16_t)FOC_COG_TABLE_SIZE : (uint16_t)COG_WARMUP_BINS;
}

/* Commit one bin's sample (the freshest filtered loop output seen inside it).
 * Pass 1: CW writes the bin and marks it covered; CCW averages into covered
 * bins (the two passes settle friction at opposite signs, so the mean cancels
 * it) or writes the bins CW missed. Pass 2 (feed-forward live): the sample is
 * the RESIDUAL the loop still fights, so it is added into the map in place —
 * half per direction, so friction cancels the same way and the residual lands
 * exactly once. */
static void cog_commit(cog_ctx_t *c, uint32_t bin, float iq) {
  int32_t tmp = (int32_t)(iq * FOC_COG_SCALE + (iq >= 0.0f ? 0.5f : -0.5f));
  if (tmp >  32767) { tmp =  32767; }
  if (tmp < -32768) { tmp = -32768; }
  if (c->phase == COG_CW1) {
    g_cog_map[bin] = (int16_t)tmp;
    cov_set(bin);
  } else if (c->phase == COG_CCW1) {
    g_cog_map[bin] = cov_get(bin)
        ? (int16_t)(((int32_t)g_cog_map[bin] + tmp) / 2)
        : (int16_t)tmp;
  } else {
    int32_t v = (int32_t)g_cog_map[bin] + tmp / 2;
    if (v >  32767) { v =  32767; }
    if (v < -32768) { v = -32768; }
    g_cog_map[bin] = (int16_t)v;
    cov_set(bin);
  }
}

/* COG_MID, one chunk per tick (512 bins -> 8 ticks per sub-step, too heavy to
 * do whole-table in the 20 kHz ISR). Sub-step 0 accumulates the pass-1 mean
 * over covered bins; sub-step 1 removes it and clamps, then enables the rough
 * map as live feed-forward for pass 2. A pass-1 sweep that covered too little
 * is not worth iterating on: jump straight to COG_DONE and let finish() judge
 * it (it will apply the same 3/4 coverage rule as before). */
static void cog_mid_chunk(cog_ctx_t *c) {
  uint32_t end = (uint32_t)c->idx + COG_MID_CHUNK;
  uint32_t i;
  if (end > FOC_COG_TABLE_SIZE) { end = FOC_COG_TABLE_SIZE; }

  if (c->mid_step == 0u) {
    for (i = c->idx; i < end; i++) {
      if (cov_get(i)) {
        c->mid_mean += (float)g_cog_map[i];
        c->mid_count++;
      }
    }
    c->idx = (uint16_t)end;
    if (end >= FOC_COG_TABLE_SIZE) {
      if (c->mid_count < COG_MID_MIN_COVERAGE) {
        c->phase = COG_DONE;
        return;
      }
      c->mid_mean /= (float)c->mid_count;
      c->mid_step = 1u;
      c->idx = 0u;
    }
  } else {
    float max_ff_q = c->max_ff * FOC_COG_SCALE;
    for (i = c->idx; i < end; i++) {
      if (cov_get(i)) {
        float v = (float)g_cog_map[i] - c->mid_mean;
        if (v >  max_ff_q) { v =  max_ff_q; }
        if (v < -max_ff_q) { v = -max_ff_q; }
        g_cog_map[i] = (int16_t)(v + (v >= 0.0f ? 0.5f : -0.5f));
      }
    }
    c->idx = (uint16_t)end;
    if (end >= FOC_COG_TABLE_SIZE) {
      c->valid   = 1u;               /* rough map is good enough to feed forward */
      c->enabled = 1u;
      c->idx  = 0u;
      c->tick = 0u;
      c->phase = COG_WARMUP_CW2;
    }
  }
}

void foc_start_cogging(motor_t *motor, float creep_rpm, float max_ff, float drag_a) {
  cog_ctx_t *c = &motor->cog;
  uint32_t i;

  /* Clear the map + coverage while still in IDLE (the ISR ignores COG until
   * the state flips below), so no interrupt masking is needed here. */
  for (i = 0; i < FOC_COG_TABLE_SIZE; i++) {
    g_cog_map[i] = 0;
  }
  for (i = 0; i < COG_COV_BYTES; i++) {
    g_cog_cov[i] = 0u;
  }

  c->creep_rpm = (creep_rpm > 0.0f) ? creep_rpm : COG_DEFAULT_CREEP_RPM;
  c->max_ff    = (max_ff > 0.0f) ? max_ff : COG_DEFAULT_MAXFF;
  c->drag_a    = (drag_a > 0.0f) ? drag_a : COG_DEFAULT_DRAG_A;
  if (c->drag_a > COG_MAX_DRAG_A) { c->drag_a = COG_MAX_DRAG_A; }
  c->drag_ff   = 0.0f;
  c->idx       = 0u;
  c->tick      = 0u;
  c->pend_iq   = 0.0f;
  c->last_bin  = 0u;
  c->have_pend = 0u;
  c->mid_step  = 0u;
  c->mid_mean  = 0.0f;
  c->mid_count = 0u;
  c->enabled   = 0u;                 /* off during pass 1 (no self-compensation) */
  c->phase     = COG_WARMUP_CW;

  /* March from the live continuous position; the running target advances at a
   * constant slow rate from here. */
  c->target = motor->ctrl.pos.current_pos;

  /* Prime the cmd pre-filter so the first samples are not dragged toward 0. */
  lpf_init(&g_iq_lpf, COG_IQ_LPF_ALPHA);
  g_iq_lpf.out_prev = motor->target.iq;

  /* Save then adapt the position-loop profile: boosted landing gain for
   * tracking stiffness, deadband zeroed (it is wider than one bin and would
   * stall the march inside the band). max_rpm is re-asserted every tick by
   * foc_cogging_run; all three are restored by foc_set_state when COG exits. */
  c->saved_max_rpm     = motor->ctrl.pos.max_rpm;
  c->saved_landing_kp  = motor->ctrl.pos.landing_kp;
  c->saved_deadband    = motor->ctrl.pos.deadband;
  motor->ctrl.pos.landing_kp = COG_APPROACH_KP;
  motor->ctrl.pos.deadband   = 0.0f;

  foc_set_state(motor, FOC_STATE_COG);
}

void foc_cogging_run(motor_t *motor) {
  cog_ctx_t *c = &motor->cog;
  pos_controller_t *pos = &motor->ctrl.pos;

  motor->target.id = 0.0f;            /* pure torque current, no d-axis */

  if (c->phase == COG_DONE) {
    c->drag_ff      = 0.0f;
    pos->max_rpm    = c->saved_max_rpm;   /* restore the normal speed profile */
    pos->target_pos = pos->current_pos;   /* hold here; main loop finalises */
    foc_set_state(motor, FOC_STATE_IDLE);
    return;
  }

  /* Drive the cascade with the marching target; max_rpm only caps how fast
   * the rotor may catch up after a stiction break (the march itself is
   * 1.5 rpm). The drag feed-forward (added to target.iq in foc_core.c) is
   * direction-signed with the march and does the heavy lifting against
   * gearbox stiction; it is zero while parked at COG_MID. */
  pos->max_rpm = c->creep_rpm;
  pos->target_pos = c->target;
  c->drag_ff = (c->phase == COG_MID) ? 0.0f : phase_dir(c->phase) * c->drag_a;

  /* Pass boundary between the two sweeps: finalise the rough pass-1 map
   * chunk by chunk (position held), then enable it as live feed-forward. */
  if (c->phase == COG_MID) {
    cog_mid_chunk(c);
    return;
  }

  /* Sample: filtered speed-loop output, binned by the ACTUAL angle. Each
   * continuous visit of a bin keeps only the freshest value (the most settled
   * one); the previous bin is committed when the rotor leaves it — even during
   * a gated lurch, so a settled pre-lurch value is never lost. Samples taken
   * while moving too fast or backwards are dropped: those windows carry
   * inertia + P-term torque, not cogging. Warm-up phases keep the pre-filter
   * tracking (result discarded) so the first sampled bins after a direction
   * change are not dragged toward the previous phase's level. */
  if (phase_is_sampling(c->phase)) {
    float iq_f = lpf_apply(&g_iq_lpf, motor->target.iq);
    uint32_t bin = cog_bin_of(motor->sensor.encoder.mech_angle);
    if (c->have_pend && bin != (uint32_t)c->last_bin) {
      cog_commit(c, (uint32_t)c->last_bin, c->pend_iq);
      c->have_pend = 0u;
    }
    float dir = phase_dir(c->phase);
    float rpm = motor->actual.rpm;
    if (fabsf(rpm) <= COG_GATE_FAST_MULT * COG_MARCH_RPM &&
        dir * rpm >= -COG_GATE_REV_MULT * COG_MARCH_RPM) {
      c->pend_iq  = iq_f;
      c->last_bin = (uint16_t)bin;
      c->have_pend = 1u;
    }
  } else {
    (void)lpf_apply(&g_iq_lpf, motor->target.iq);   /* warm-up: keep filter fresh */
  }

  /* March the target at a constant slow rate (one bin per COG_MARCH_TICKS) —
   * but only while the rotor keeps up. If stiction parks it more than
   * COG_LEASH_RAD behind the target, the march (and the phase clock) WAITS:
   * without this leash a stuck rotor lets the target run far away, and the
   * eventual break-loose becomes a long capped-speed chase (2 rpm cap vs
   * 1.5 rpm march = 0.5 rpm closing rate) whose samples are all P-term /
   * integrator transient, not cogging — observed on the bench as the rotor
   * sitting ~100 deg behind the marching target after a direction reversal.
   * Paused ticks still SAMPLE: a rotor parked in a bin with the integrator
   * winding up is a settled held-torque measurement (stiction included — the
   * CW/CCW average cancels it). */
  float lag = c->target - pos->current_pos;
  if (fabsf(lag) < COG_LEASH_RAD) {
    c->target += phase_dir(c->phase) * COG_BIN_RATE;
    c->tick++;
    if (c->tick >= COG_MARCH_TICKS) {
      c->tick = 0u;
      c->idx++;
    }
  }

  if (c->idx >= phase_bins(c->phase)) {
    /* Commit the bin the rotor sits in at the phase boundary. */
    if (c->have_pend && phase_is_sampling(c->phase)) {
      cog_commit(c, (uint32_t)c->last_bin, c->pend_iq);
    }
    c->have_pend = 0u;
    c->idx  = 0u;
    c->tick = 0u;
    c->phase = (cog_phase_t)((uint8_t)c->phase + 1u);   /* ... -> COG_MID/DONE */
  }
}

uint8_t foc_cogging_finish(motor_t *motor) {
  cog_ctx_t *c = &motor->cog;
  float mean = 0.0f;
  float peak = 0.0f;
  float max_ff_q = c->max_ff * FOC_COG_SCALE;
  uint32_t covered = 0u;
  uint32_t i;

  /* Mean over covered bins only (uncovered bins must not drag the DC
   * estimate). */
  for (i = 0; i < FOC_COG_TABLE_SIZE; i++) {
    if (cov_get(i)) {
      mean += (float)g_cog_map[i];
      covered++;
    }
  }
  c->covered = (uint16_t)covered;      /* info: shown by `cog info` */
  if (covered == 0u) {
    c->peak = 0.0f;
    c->valid = 0u;
    c->enabled = 0u;
    return 0u;
  }
  mean /= (float)covered;

  /* Remove the DC component (cogging is zero-mean over a revolution; any
   * offset is friction/load that must NOT be fed forward) and clamp.
   * Uncovered bins go to zero (no feed-forward where there is no data). */
  for (i = 0; i < FOC_COG_TABLE_SIZE; i++) {
    if (cov_get(i)) {
      float v = (float)g_cog_map[i] - mean;
      if (v >  max_ff_q) { v =  max_ff_q; }
      if (v < -max_ff_q) { v = -max_ff_q; }
      g_cog_map[i] = (int16_t)(v + (v >= 0.0f ? 0.5f : -0.5f));
      float ab = fabsf(v);
      if (ab > peak) { peak = ab; }
    } else {
      g_cog_map[i] = 0;
    }
  }
  peak /= FOC_COG_SCALE;

  c->peak = peak;
  if (covered < COG_MIN_COVERAGE || peak < 1.0e-4f) {
    c->valid = 0u;            /* sweep did not really turn the rotor */
    c->enabled = 0u;
    return 0u;
  }
  c->valid = 1u;
  c->enabled = 1u;            /* auto-enable a fresh, good map */
  return 1u;
}

float foc_cogging_lookup(const motor_t *motor) {
  const cog_ctx_t *c = &motor->cog;
  if (!c->enabled || !c->valid) {
    return 0.0f;
  }
  return (float)g_cog_map[cog_bin_of(motor->sensor.encoder.mech_angle)] / FOC_COG_SCALE;
}

void foc_cogging_set_enable(motor_t *motor, uint8_t on) {
  if (motor->cog.valid) {
    motor->cog.enabled = on ? 1u : 0u;
  }
}

void foc_cogging_clear(motor_t *motor) {
  motor->cog.valid = 0u;
  motor->cog.enabled = 0u;
  motor->cog.peak = 0.0f;
  motor->cog.covered = 0u;
}

uint8_t foc_cogging_is_valid(const motor_t *motor)   { return motor->cog.valid; }
uint8_t foc_cogging_is_enabled(const motor_t *motor) { return motor->cog.enabled; }
float   foc_cogging_peak(const motor_t *motor)       { return motor->cog.peak; }
uint16_t foc_cogging_coverage(const motor_t *motor)  { return motor->cog.covered; }

const int16_t *foc_cogging_table(const motor_t *motor) {
  return motor->cog.valid ? g_cog_map : 0;
}

void foc_cogging_load_table(motor_t *motor, const int16_t *table, uint8_t enabled) {
  cog_ctx_t *c = &motor->cog;
  float peak = 0.0f;
  uint32_t i;

  for (i = 0; i < FOC_COG_TABLE_SIZE; i++) {
    g_cog_map[i] = table[i];
    float ab = fabsf((float)table[i]);
    if (ab > peak) { peak = ab; }
  }
  c->peak    = peak / FOC_COG_SCALE;
  c->max_ff  = COG_DEFAULT_MAXFF;
  c->covered = (uint16_t)FOC_COG_TABLE_SIZE;
  c->valid   = 1u;
  c->enabled = enabled ? 1u : 0u;
}
