#include "foc_ident.h"
#include "foc_core.h"
#include <math.h>

/* Control tick period (alias of the HW rate) used for all ident timing math. */
#define CONTROL_DT              HW_CONTROL_PERIOD_S

/* Parameter identification — Rs (locked rotor at theta_e=0, two-point DC).
 * Two current points let us take the slope dVd/dId, which cancels the inverter
 * dead-time voltage drop (Vd = Rs*Id + Vdeadtime). Both points stay below the
 * motor's rated current. Timing is in ticks at the 20 kHz control rate. */
#define IDENT_RS_I_LOW          1.0f    /* A — first d-current point */
#define IDENT_RS_I_HIGH         2.5f    /* A — second point (< rated 2.9 A) */
#define IDENT_RS_RAMP_TICKS     4000u   /* ~200 ms to ramp between current points */
#define IDENT_RS_SETTLE_TICKS   4000u   /* ~200 ms settle before averaging */
#define IDENT_RS_AVG_TICKS      4000u   /* ~200 ms of Vd averaging */

/* Parameter identification — Ld/Lq (locked rotor, square-wave voltage injection).
 * A DC d-current first locks the rotor; the steady drive voltage is captured
 * from the current loop (so it already compensates dead-time) and reused as an
 * open-loop bias. A symmetric square wave is then injected on the d-axis (Ld) or
 * q-axis (Lq) and the peak-to-peak current ripple is measured. Two amplitudes
 * are used so the slope dRipple/dV cancels the dead-time offset, exactly like
 * Rs:  L = (Vhi - Vlo) * T_half / (PPhi - PPlo).  These values may need tuning
 * per motor; the current is soft-capped at IDENT_L_I_ABORT. */
#define IDENT_L_I_BIAS          1.5f    /* A — DC d-current that locks the rotor */
#define IDENT_L_V_LOW           1.0f    /* V — low square-wave amplitude */
#define IDENT_L_V_HIGH          2.0f    /* V — high square-wave amplitude */
#define IDENT_L_HALF_TICKS      3u      /* injection half-period (ticks) -> ~3.3 kHz */
#define IDENT_L_SETTLE_TICKS    4000u   /* settle the DC bias before injecting */
#define IDENT_L_CYCLES          200u    /* injection cycles averaged per amplitude */
#define IDENT_L_SKIP_CYCLES     16u     /* transient cycles skipped before averaging */
#define IDENT_L_I_ABORT         5.0f    /* A — abort injection if |i| exceeds this */

/* Parameter identification — flux linkage (Iq=0 coast-down, VESC "undriven").
 * Spin up with a small Iq using the existing current loop, then command
 * Id=Iq=0 and let the rotor free-coast. With zero current the applied Vq equals
 * the back-EMF, so Vq = omega_e * flux (independent of Rs/Ld). Sampling Vq vs
 * omega_e over the decaying speed and least-squares fitting through the origin
 * gives the flux linkage. Speeds are rotor-mechanical rpm (encoder is on the
 * rotor); well below the motor's ~1200 rpm rotor limit. */
#define IDENT_FLUX_SPIN_IQ      1.5f      /* A — q-current used to spin up */
#define IDENT_FLUX_SPIN_RPM     400.0f    /* rotor rpm to reach before coasting */
#define IDENT_FLUX_SPIN_TIMEOUT 60000u    /* ticks (~3 s) max spin-up time */
#define IDENT_FLUX_MAX_RPM      800.0f    /* abort if the rotor exceeds this */
#define IDENT_FLUX_COAST_SETTLE 2000u     /* ticks (~100 ms) for Iq to reach 0 */
#define IDENT_FLUX_COAST_MIN_RPM 40.0f    /* stop fitting below this rotor rpm */
#define IDENT_FLUX_COAST_TIMEOUT 200000u  /* ticks (~10 s) max coast */

/* Mechanical identification (J/B/Tc) — see foc_ident.h for the method
 * overview. J: spin up to IMECH_J_TOP with +Iq (settle guard for the current
 * rise), cut iq, then time the free coast through [IMECH_COAST_LO, HI] rpm —
 * J = f(w_mid)*dt/dw with f from the B/Tc fit below (computed in
 * imech_compute). B/Tc: steady-state speed steps at imech_spd_targets_rpm[];
 * a conservative local PI speed loop holds each target and the tail of the
 * settle window is averaged into (w, iq) — steady-state fit Kt*iq = B*w + Tc.
 * All speeds are far below the ~1200 rpm rotor / ~1000 rpm abort limit. */
#define IMECH_J_TOP             500.0f    /* spin-up target before coasting */
#define IMECH_COAST_HI          400.0f    /* coast timing window [rpm] */
#define IMECH_COAST_LO          200.0f
#define IMECH_J_START_SETTLE_TICKS 20u    /* ~1 ms current rise before timing */
#define IMECH_ACCEL_TIMEOUT     80000u    /* ticks (~4 s) max spin-up */
#define IMECH_COAST_TIMEOUT     80000u    /* ticks (~4 s) max coast */
#define IMECH_ABORT_RPM         1000.0f   /* safety: abort if rotor exceeds this */
#define IMECH_SPD_SETTLE_TICKS  50000u    /* 2.5 s settle per target speed */
#define IMECH_SPD_RECORD_TICKS  10000u    /* last 0.5 s averaged */
#define IMECH_SPD_IQ_MAX        2.0f      /* local speed-PI output clamp [A] */
#define IMECH_SPD_KP            0.005f    /* conservative P gain (A/rpm) */
#define IMECH_SPD_KI            0.02f     /* A/(rpm*s) — slow integrator */

static const float imech_spd_targets_rpm[IMECH_SPD_N_TARGETS] = {
  200.0f, 300.0f, 400.0f, 500.0f
};

/* Phase helpers — range-test the contiguous ident_phase_t order. */

// Injection phases apply open-loop square-wave voltage (no current loop).
static inline uint8_t ident_is_inj_phase(ident_phase_t p) {
  return (p >= IDENT_LD_INJ_LOW) && (p <= IDENT_LQ_INJ_HIGH);
}

// Rs + Ld/Lq phases lock the rotor at theta_e = 0 (phase A).
static inline uint8_t ident_is_locked_phase(ident_phase_t p) {
  return (p <= IDENT_LQ_INJ_HIGH);
}

// Flux phases spin the rotor and run on the measured electrical angle.
static inline uint8_t ident_is_flux_phase(ident_phase_t p) {
  return (p == IDENT_FLUX_SPINUP) || (p == IDENT_FLUX_COAST);
}

/* Reset the square-wave injection accumulators before an injection phase. */
static inline void ident_inject_reset(ident_ctx_t *c) {
  c->inj_sign = 1;
  c->inj_tick = 0;
  c->half_count = 0;
  c->i_top = -1.0e9f;
  c->i_bot =  1.0e9f;
  c->ripple_sum = 0.0f;
  c->ripple_n = 0;
  c->ripple_skip = IDENT_L_SKIP_CYCLES;
}

/* Drive one tick of the square-wave injection on the chosen axis (0 = d, 1 = q).
 * Tracks the running max/min of the current over each full injection period and
 * accumulates the peak-to-peak ripple. Using period extrema (not a fixed-tick
 * sample) makes the measurement immune to the 1-cycle ADC pipeline delay between
 * setting a voltage and reading its current. Returns the averaged peak-to-peak
 * ripple once IDENT_L_CYCLES periods have accumulated, else a negative value.
 * The DC d-bias (c->vd_bias) is always applied. */
static float ident_inject_step(motor_t *motor, uint8_t axis, float v_amp) {
  ident_ctx_t *c = &motor->ident;
  float v = (c->inj_sign > 0) ? v_amp : -v_amp;
  if (axis == 0u) {
    c->vd_cmd = c->vd_bias + v;
    c->vq_cmd = 0.0f;
  } else {
    c->vd_cmd = c->vd_bias;
    c->vq_cmd = v;
  }

  float i = (axis == 0u) ? motor->actual.id : motor->actual.iq;
  if (i > c->i_top) { c->i_top = i; }
  if (i < c->i_bot) { c->i_bot = i; }

  c->inj_tick++;
  if (c->inj_tick >= IDENT_L_HALF_TICKS) {
    c->inj_sign = (int8_t)-c->inj_sign;
    c->inj_tick = 0;
    c->half_count++;
    if (c->half_count >= 2u) {        /* one full square-wave period elapsed */
      c->half_count = 0;
      if (c->ripple_skip > 0u) {
        c->ripple_skip--;             /* let the ripple reach steady state first */
      } else {
        c->ripple_sum += (c->i_top - c->i_bot);
        c->ripple_n++;
        if (c->ripple_n >= IDENT_L_CYCLES) {
          return c->ripple_sum / (float)c->ripple_n;
        }
      }
      c->i_top = -1.0e9f;             /* reset extrema for the next period */
      c->i_bot =  1.0e9f;
    }
  }
  return -1.0f;
}

/* Advance the identification sub-state machine (called while in FOC_STATE_IDENT).
 *   Rs    : two-point locked-rotor DC, slope dVd/dId (cancels dead-time).
 *   Ld/Lq : square-wave voltage injection at theta_e = 0, two amplitudes, slope
 *           dRipple/dV (cancels dead-time).  Flux linkage is phase 3.
 * The d-current / open-loop voltage commands are produced here; foc_ident_run
 * applies them and stores the resulting Vd/Vq in motor->actual (read back here). */
static void ident_update(motor_t *motor) {
  ident_ctx_t *c = &motor->ident;
  float t_half = (float)IDENT_L_HALF_TICKS * CONTROL_DT;

  /* Soft current cap during injection (the open-loop amplitude depends on the
   * unknown L). Hard overcurrent (fault_check_fast) is the real backstop. */
  if (ident_is_inj_phase(c->phase) &&
      (fabsf(motor->actual.id) > IDENT_L_I_ABORT ||
       fabsf(motor->actual.iq) > IDENT_L_I_ABORT)) {
    c->vd_cmd = 0.0f;
    c->vq_cmd = 0.0f;
    c->phase = IDENT_DONE;   /* leaves ld/lq at 0 -> user sees the failure */
  }

  switch (c->phase) {
  case IDENT_RS_RAMP_LOW:
    c->id_ref += IDENT_RS_I_LOW / (float)IDENT_RS_RAMP_TICKS;
    if (c->id_ref >= IDENT_RS_I_LOW) {
      c->id_ref = IDENT_RS_I_LOW;
      c->counter = 0;
      c->vd_sum = 0.0f;
      c->id_sum = 0.0f;
      c->n = 0;
      c->phase = IDENT_RS_MEAS_LOW;
    }
    break;

  case IDENT_RS_MEAS_LOW:
    c->counter++;
    if (c->counter > IDENT_RS_SETTLE_TICKS) {
      c->vd_sum += motor->actual.vd;   /* Vd applied last cycle */
      c->id_sum += motor->actual.id;
      c->n++;
    }
    if (c->counter >= IDENT_RS_SETTLE_TICKS + IDENT_RS_AVG_TICKS) {
      c->vd_low = (c->n > 0u) ? c->vd_sum / (float)c->n : 0.0f;
      c->id_low = (c->n > 0u) ? c->id_sum / (float)c->n : 0.0f;
      c->phase = IDENT_RS_RAMP_HIGH;
    }
    break;

  case IDENT_RS_RAMP_HIGH:
    c->id_ref += (IDENT_RS_I_HIGH - IDENT_RS_I_LOW) / (float)IDENT_RS_RAMP_TICKS;
    if (c->id_ref >= IDENT_RS_I_HIGH) {
      c->id_ref = IDENT_RS_I_HIGH;
      c->counter = 0;
      c->vd_sum = 0.0f;
      c->id_sum = 0.0f;
      c->n = 0;
      c->phase = IDENT_RS_MEAS_HIGH;
    }
    break;

  case IDENT_RS_MEAS_HIGH:
    c->counter++;
    if (c->counter > IDENT_RS_SETTLE_TICKS) {
      c->vd_sum += motor->actual.vd;
      c->id_sum += motor->actual.id;
      c->n++;
    }
    if (c->counter >= IDENT_RS_SETTLE_TICKS + IDENT_RS_AVG_TICKS) {
      c->vd_high = (c->n > 0u) ? c->vd_sum / (float)c->n : 0.0f;
      c->id_high = (c->n > 0u) ? c->id_sum / (float)c->n : 0.0f;
      /* Slope over the measured currents cancels the dead-time offset. */
      float did = c->id_high - c->id_low;
      c->rs = (did > 0.01f) ? (c->vd_high - c->vd_low) / did : 0.0f;
      c->id_ref = 0.0f;
      c->counter = 0;
      c->phase = IDENT_L_BIAS;
    }
    break;

  case IDENT_L_BIAS:
    /* Closed-loop DC d-current to lock the rotor; capture the steady drive
     * voltage (which already includes dead-time compensation) as the open-loop
     * bias for the injection phases. */
    c->id_ref += IDENT_L_I_BIAS / (float)IDENT_RS_RAMP_TICKS;
    if (c->id_ref > IDENT_L_I_BIAS) { c->id_ref = IDENT_L_I_BIAS; }
    c->counter++;
    if (c->counter >= IDENT_RS_RAMP_TICKS + IDENT_L_SETTLE_TICKS) {
      c->vd_bias = motor->actual.vd;
      ident_inject_reset(c);
      c->phase = IDENT_LD_INJ_LOW;
    }
    break;

  case IDENT_LD_INJ_LOW: {
    float pp = ident_inject_step(motor, 0u, IDENT_L_V_LOW);
    if (pp >= 0.0f) {
      c->ld_pp_low = pp;
      ident_inject_reset(c);
      c->phase = IDENT_LD_INJ_HIGH;
    }
    break;
  }

  case IDENT_LD_INJ_HIGH: {
    float pp = ident_inject_step(motor, 0u, IDENT_L_V_HIGH);
    if (pp >= 0.0f) {
      c->ld_pp_high = pp;
      float dpp = c->ld_pp_high - c->ld_pp_low;
      /* L in microhenries (1e6 * H) so it displays with the shell's 3-decimal
       * float printer; convert back to H if ever fed into the control gains. */
      c->ld = (dpp > 0.001f) ? (IDENT_L_V_HIGH - IDENT_L_V_LOW) * t_half / dpp * 1.0e6f : 0.0f;
      ident_inject_reset(c);
      c->phase = IDENT_LQ_INJ_LOW;
    }
    break;
  }

  case IDENT_LQ_INJ_LOW: {
    float pp = ident_inject_step(motor, 1u, IDENT_L_V_LOW);
    if (pp >= 0.0f) {
      c->lq_pp_low = pp;
      ident_inject_reset(c);
      c->phase = IDENT_LQ_INJ_HIGH;
    }
    break;
  }

  case IDENT_LQ_INJ_HIGH: {
    float pp = ident_inject_step(motor, 1u, IDENT_L_V_HIGH);
    if (pp >= 0.0f) {
      c->lq_pp_high = pp;
      float dpp = c->lq_pp_high - c->lq_pp_low;
      c->lq = (dpp > 0.001f) ? (IDENT_L_V_HIGH - IDENT_L_V_LOW) * t_half / dpp * 1.0e6f : 0.0f;
      c->vd_cmd = 0.0f;
      c->vq_cmd = 0.0f;
      /* Hand over to the flux spin-up on the measured angle (closed loop). */
      pid_reset(&motor->ctrl.id);
      pid_reset(&motor->ctrl.iq);
      c->iq_ref = 0.0f;
      c->flux_sum_vw = 0.0f;
      c->flux_sum_ww = 0.0f;
      c->counter = 0;
      c->phase = IDENT_FLUX_SPINUP;
    }
    break;
  }

  case IDENT_FLUX_SPINUP: {
    /* Accelerate with a small Iq (Id=0) until the rotor reaches the target.
     * Compare on |rpm| so the encoder direction convention does not matter. */
    float rpm_mag = fabsf(motor->actual.rpm);
    c->iq_ref = IDENT_FLUX_SPIN_IQ;
    c->counter++;
    if (rpm_mag >= IDENT_FLUX_SPIN_RPM || c->counter >= IDENT_FLUX_SPIN_TIMEOUT) {
      c->iq_ref = 0.0f;          /* cut drive -> coast on Id=Iq=0 */
      c->counter = 0;
      c->phase = IDENT_FLUX_COAST;
    } else if (rpm_mag > IDENT_FLUX_MAX_RPM) {
      c->iq_ref = 0.0f;          /* safety: too fast, bail out */
      c->phase = IDENT_DONE;
    }
    break;
  }

  case IDENT_FLUX_COAST: {
    /* Id=Iq=0: the loop applies Vq = back-EMF = omega_e * flux. Accumulate a
     * least-squares fit through the origin once the current has settled to 0.
     * Vq and omega_e share sign, so the fit is direction-agnostic. */
    float rpm_mag = fabsf(motor->actual.rpm);
    c->iq_ref = 0.0f;
    c->counter++;
    if (c->counter > IDENT_FLUX_COAST_SETTLE && rpm_mag > IDENT_FLUX_COAST_MIN_RPM) {
      float we = motor->actual.rpm * FOC_RPM_TO_RAD * (float)HW_MOTOR_POLE_PAIRS;
      c->flux_sum_vw += motor->actual.vq * we;
      c->flux_sum_ww += we * we;
    }
    if ((c->counter > IDENT_FLUX_COAST_SETTLE && rpm_mag <= IDENT_FLUX_COAST_MIN_RPM) ||
        c->counter >= IDENT_FLUX_COAST_TIMEOUT) {
      /* flux in milliwebers (1e3 * Wb) for a clean shell display. */
      c->flux = (c->flux_sum_ww > 100.0f)
                  ? (c->flux_sum_vw / c->flux_sum_ww) * 1000.0f : 0.0f;
      c->phase = IDENT_DONE;
    }
    break;
  }

  case IDENT_DONE:
  default:
    foc_set_state(motor, FOC_STATE_IDLE);
    break;
  }
}

/* Compute J/B/Tc from the recorded data.
 *  B/Tc first: least-squares fit Kt*iq = B*w + Tc over the settled speed
 *     points — purely steady-state averages, immune to transients by
 *     construction.
 *  J: coast-down 400 -> 200 rpm with iq = 0. With zero current  J*a = f(w),
 *     so  J = f(w_mid) * dt / dw  using the fitted B/Tc. No current reversal,
 *     no sign flip, and the PLL's constant acceleration lag cancels in the
 *     time difference of a steady coast. */
static void imech_compute(motor_t *motor) {
  imech_ctx_t *m = &motor->imech;

  /* --- B/Tc (steady-state least squares through offset Tc) --- */
  float sx = 0.0f, sy = 0.0f, sxx = 0.0f, sxy = 0.0f;
  uint32_t fn = 0u;
  for (uint32_t i = 0u; i < m->n_pts; i++) {
    float w = m->w_pts[i];
    float y = m->kt * m->iq_pts[i];
    sx += w; sy += y; sxx += w * w; sxy += w * y; fn++;
  }
  if (fn >= 2u) {
    float denom = (float)fn * sxx - sx * sx;
    if (denom > 1.0e-9f || denom < -1.0e-9f) {
      float bb = ((float)fn * sxy - sx * sy) / denom;  /* Nm*s/rad */
      float tcb = (sy - bb * sx) / (float)fn;          /* Nm */
      /* Physical sanity: viscous friction B and Coulomb friction Tc cannot be
       * negative. A negative fit is a measurement artefact; clamp to 0 and
       * flag the result so the operator is warned (identinfo) and the deployed
       * speed-loop Ki is not destabilising. */
      m->friction_valid = (bb >= 0.0f && tcb >= 0.0f) ? 1u : 0u;
      m->b  = (bb  >= 0.0f) ? bb  * 1.0e6f : 0.0f;     /* -> uNm*s/rad */
      m->tc = (tcb >= 0.0f) ? tcb * 1.0e3f : 0.0f;     /* -> mNm */
    }
  }

  /* --- J (coast-down, needs the B/Tc fit above) --- */
  float w_mid = (IMECH_COAST_HI + IMECH_COAST_LO) * 0.5f * FOC_RPM_TO_RAD;
  float f_avg = m->b * 1.0e-6f * w_mid + m->tc * 1.0e-3f;   /* Nm */
  float dw_c = (m->rpm_coast_hi - m->rpm_coast_lo) * FOC_RPM_TO_RAD;
  float dt_c = (float)(m->t_coast_lo - m->t_coast_hi) * CONTROL_DT;
  if (dw_c > 0.0f && dt_c > 0.0f && f_avg > 0.0f) {
    m->j = (f_avg * dt_c / dw_c) * 1.0e7f;                 /* kg*m^2 -> g*cm^2 */
  }
}

/* Advance the mechanical-identification state machine (FOC_STATE_IMECH).
 *  J phase: constant +Iq / -Iq, timestamp the 100 and 500 rpm crossings; a
 *  settle guard on each current transition keeps the timestamps clean.
 *  SPD_STEP: a conservative local PI speed loop holds each target speed for a
 *  fixed settle window; the tail of the window is averaged into (w, iq) so the
 *  steady-state fit is immune to the approach transient. iq_ref is consumed by
 *  foc_imech_run's current loop. */
static void imech_update(motor_t *motor) {
  imech_ctx_t *m = &motor->imech;
  float rpm = fabsf(motor->actual.rpm);
  m->tick++;
  m->timeout++;

  switch (m->phase) {
  case IMECH_J_UP:
    m->iq_ref = m->iq_test;                 /* +torque, natural direction */
    if (m->j_settle < IMECH_J_START_SETTLE_TICKS) {
      m->j_settle++;                        /* let the current rise first */
      break;
    }
    if (rpm >= IMECH_J_TOP) {
      m->t_j_hi = m->tick;
      m->j_hi_done = 1u;
      m->iq_ref = 0.0f;                     /* cut current -> free coast */
      m->timeout = 0u;
      m->phase = IMECH_J_COAST;
    } else if (m->timeout >= IMECH_ACCEL_TIMEOUT || rpm > IMECH_ABORT_RPM) {
      m->iq_ref = 0.0f;                      /* could not reach top / too fast */
      m->phase = IMECH_DONE;
    }
    break;

  case IMECH_J_COAST:
    m->iq_ref = 0.0f;                        /* pure coast: J*a = f(w) */
    if (!m->coast_hi_done && rpm <= IMECH_COAST_HI) {
      m->t_coast_hi = m->tick;
      m->rpm_coast_hi = rpm;
      m->coast_hi_done = 1u;
    }
    if (m->coast_hi_done && rpm <= IMECH_COAST_LO) {
      m->t_coast_lo = m->tick;
      m->rpm_coast_lo = rpm;
      m->idx = 0;
      m->target_rpm = imech_spd_targets_rpm[0];
      m->settle_tick = 0u;
      m->sum_w = 0.0f;
      m->sum_iq = 0.0f;
      m->n = 0u;
      m->spd_int = 0.0f;
      m->timeout = 0u;
      m->phase = IMECH_SPD_STEP;
    } else if (m->timeout >= IMECH_COAST_TIMEOUT) {
      m->phase = IMECH_DONE;
    }
    break;

  case IMECH_SPD_STEP: {
    if (m->idx >= (int32_t)IMECH_SPD_N_TARGETS) {
      m->phase = IMECH_DONE;
      imech_compute(motor);
      m->iq_ref = 0.0f;
      break;
    }
    /* Local conservative speed PI (A/rpm) feeding the current loop. */
    {
      float err = m->target_rpm - motor->actual.rpm;
      m->spd_int += m->spd_ki * err * CONTROL_DT;
      if (m->spd_int > IMECH_SPD_IQ_MAX) { m->spd_int = IMECH_SPD_IQ_MAX; }
      if (m->spd_int < -IMECH_SPD_IQ_MAX) { m->spd_int = -IMECH_SPD_IQ_MAX; }
      float iq = m->spd_kp * err + m->spd_int;
      if (iq > IMECH_SPD_IQ_MAX) { iq = IMECH_SPD_IQ_MAX; }
      if (iq < -IMECH_SPD_IQ_MAX) { iq = -IMECH_SPD_IQ_MAX; }
      m->iq_ref = iq;
    }
    /* Record the tail of the settle window (steady state). */
    m->settle_tick++;
    if (m->settle_tick > IMECH_SPD_SETTLE_TICKS - IMECH_SPD_RECORD_TICKS) {
      m->sum_w += motor->actual.rpm * FOC_RPM_TO_RAD;
      m->sum_iq += m->iq_ref;
      m->n++;
    }
    if (m->settle_tick >= IMECH_SPD_SETTLE_TICKS) {
      if (m->n > 0u) {
        m->w_pts[m->idx] = m->sum_w / (float)m->n;
        m->iq_pts[m->idx] = m->sum_iq / (float)m->n;
        m->n_pts++;
      }
      m->idx++;
      if (m->idx < (int32_t)IMECH_SPD_N_TARGETS) {
        m->target_rpm = imech_spd_targets_rpm[m->idx];
        m->settle_tick = 0u;
        m->sum_w = 0.0f;
        m->sum_iq = 0.0f;
        m->n = 0u;
      } else {
        m->phase = IMECH_DONE;
        imech_compute(motor);
        m->iq_ref = 0.0f;
      }
    }
    break;
  }

  case IMECH_DONE:
  default:
    m->iq_ref = 0.0f;
    foc_set_state(motor, FOC_STATE_IDLE);
    break;
  }
}

/* Public API */

void foc_start_ident(motor_t *motor) {
  ident_ctx_t *c = &motor->ident;
  c->phase = IDENT_RS_RAMP_LOW;
  c->counter = 0;
  c->id_ref = 0.0f;
  c->vd_sum = 0.0f;
  c->id_sum = 0.0f;
  c->n = 0;
  c->vd_low = 0.0f;
  c->vd_high = 0.0f;
  c->id_low = 0.0f;
  c->id_high = 0.0f;
  c->vd_cmd = 0.0f;
  c->vq_cmd = 0.0f;
  c->vd_bias = 0.0f;
  c->ld_pp_low = 0.0f;
  c->ld_pp_high = 0.0f;
  c->lq_pp_low = 0.0f;
  c->lq_pp_high = 0.0f;
  c->iq_ref = 0.0f;
  c->flux_sum_vw = 0.0f;
  c->flux_sum_ww = 0.0f;
  c->rs = 0.0f;
  c->ld = 0.0f;
  c->lq = 0.0f;
  c->flux = 0.0f;
  foc_set_state(motor, FOC_STATE_IDENT);
}

void foc_start_identmech(motor_t *motor, float kt, float iq_test) {
  imech_ctx_t *m = &motor->imech;
  m->phase = IMECH_J_UP;
  m->kt = kt;
  m->iq_test = iq_test;
  m->iq_ref = 0.0f;
  m->tick = 0u;
  m->timeout = 0u;
  m->idx = 0;
  m->t_j_hi = 0u;
  m->t_coast_hi = 0u;
  m->t_coast_lo = 0u;
  m->rpm_coast_hi = 0.0f;
  m->rpm_coast_lo = 0.0f;
  m->j_hi_done = 0u;
  m->coast_hi_done = 0u;
  m->j_settle = 0u;
  m->target_rpm = imech_spd_targets_rpm[0];
  m->spd_kp = IMECH_SPD_KP;
  m->spd_ki = IMECH_SPD_KI;
  m->spd_int = 0.0f;
  m->settle_tick = 0u;
  m->sum_w = 0.0f;
  m->sum_iq = 0.0f;
  m->n = 0u;
  m->n_pts = 0u;
  m->j = 0.0f;
  m->b = 0.0f;
  m->tc = 0.0f;
  m->friction_valid = 0u;   /* set by imech_compute after a plausible fit */
  foc_set_state(motor, FOC_STATE_IMECH);
}

uint8_t foc_ident_locks_rotor(const motor_t *motor) {
  return (motor->state == FOC_STATE_IDENT) && ident_is_locked_phase(motor->ident.phase);
}

void foc_ident_run(motor_t *motor, float *vd_out, float *vq_out) {
  ident_ctx_t *c = &motor->ident;
  ident_update(motor);
  if (ident_is_inj_phase(c->phase)) {
    *vd_out = c->vd_cmd;   /* open-loop square-wave injection */
    *vq_out = c->vq_cmd;
  } else if (ident_is_locked_phase(c->phase)) {
    /* Rs phases + L bias: closed current loop on the d-current command. */
    *vd_out = pi_calc(&motor->ctrl.id, c->id_ref, motor->actual.id);
    *vq_out = pi_calc(&motor->ctrl.iq, 0.0f, motor->actual.iq);
  } else if (ident_is_flux_phase(c->phase)) {
    /* Flux spin-up/coast: Id=0, Iq from the spin command (0 while coasting,
     * so Vq settles to the back-EMF). Runs on the measured electrical angle. */
    *vd_out = pi_calc(&motor->ctrl.id, 0.0f, motor->actual.id);
    *vq_out = pi_calc(&motor->ctrl.iq, c->iq_ref, motor->actual.iq);
  }
}

void foc_imech_run(motor_t *motor, float *vd_out, float *vq_out) {
  /* Mechanical ident: torque mode (Id=0, Iq = +/- iq_test) on the measured
   * angle; the state machine sets iq_ref as it ramps the rotor up and down. */
  imech_update(motor);
  *vd_out = pi_calc(&motor->ctrl.id, 0.0f, motor->actual.id);
  *vq_out = pi_calc(&motor->ctrl.iq, motor->imech.iq_ref, motor->actual.iq);
}
