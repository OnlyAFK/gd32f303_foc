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

/* Mechanical identification (J/B/Tc) — bidirectional torque, current mode.
 * Speed gates are rotor mechanical rpm, all far below the ~1200 rpm rotor limit.
 * Per-band timing (time between two gate crossings) is immune to a constant PLL
 * speed lag, so reading the PLL speed is fine. */
#define IMECH_ACCEL_TIMEOUT     80000u    /* ticks (~4 s) max spin-up */
#define IMECH_DECEL_TIMEOUT     80000u    /* ticks (~4 s) max braking */
#define IMECH_ABORT_RPM         1000.0f   /* safety: abort if rotor exceeds this */

static const float imech_gates_rpm[IMECH_N_GATES] = {
  100.0f, 200.0f, 300.0f, 400.0f, 500.0f, 600.0f
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

/* Compute J/B/Tc from the recorded gate-crossing times. Per band [g_i, g_i+1]:
 *   a_up = dw/dt_up,  a_dn = dw/dt_dn  (both magnitudes)
 *   J    = 2*Te/(a_up + a_dn)        -> friction cancels at matched speed
 *   f    = J*(a_dn - a_up)/2         -> friction torque at the band midpoint
 * J is averaged over bands; f(omega) is least-squares fit -> B (slope), Tc. */
static void imech_compute(motor_t *motor) {
  imech_ctx_t *m = &motor->imech;
  float te = m->kt * m->iq_test;

  float j_sum = 0.0f;
  uint32_t j_n = 0u;
  float sx = 0.0f, sy = 0.0f, sxx = 0.0f, sxy = 0.0f;  /* for f = B*w + Tc */
  uint32_t fn = 0u;

  for (uint32_t i = 0u; i + 1u < IMECH_N_GATES; i++) {
    float dw = (imech_gates_rpm[i + 1u] - imech_gates_rpm[i]) * FOC_RPM_TO_RAD;
    float dt_up = (float)(m->t_up[i + 1u] - m->t_up[i]) * CONTROL_DT;
    float dt_dn = (float)(m->t_dn[i] - m->t_dn[i + 1u]) * CONTROL_DT;
    if (dt_up <= 0.0f || dt_dn <= 0.0f) {
      continue;
    }
    float a_up = dw / dt_up;
    float a_dn = dw / dt_dn;
    if ((a_up + a_dn) <= 0.0f) {
      continue;
    }
    float jb = 2.0f * te / (a_up + a_dn);              /* kg*m^2 */
    float wm = (imech_gates_rpm[i + 1u] + imech_gates_rpm[i]) * 0.5f * FOC_RPM_TO_RAD;
    float fb = jb * (a_dn - a_up) * 0.5f;              /* Nm */
    j_sum += jb;
    j_n++;
    sx += wm; sy += fb; sxx += wm * wm; sxy += wm * fb; fn++;
  }

  if (j_n > 0u) {
    m->j = (j_sum / (float)j_n) * 1.0e7f;              /* kg*m^2 -> g*cm^2 */
  }
  if (fn >= 2u) {
    float denom = (float)fn * sxx - sx * sx;
    if (denom > 1.0e-9f || denom < -1.0e-9f) {
      float bb = ((float)fn * sxy - sx * sy) / denom;  /* Nm*s/rad */
      float tcb = (sy - bb * sx) / (float)fn;          /* Nm */
      m->b = bb * 1.0e6f;                              /* -> uNm*s/rad */
      m->tc = tcb * 1.0e3f;                            /* -> mNm */
    }
  }
}

/* Advance the mechanical-identification state machine (FOC_STATE_IMECH). Spins
 * the rotor with +Iq up through the speed gates, then -Iq back down, recording
 * the crossing times; the rate difference between the two directions yields J
 * and the friction model. iq_ref is consumed by foc_imech_run's current loop. */
static void imech_update(motor_t *motor) {
  imech_ctx_t *m = &motor->imech;
  float rpm = fabsf(motor->actual.rpm);
  m->tick++;
  m->timeout++;

  switch (m->phase) {
  case IMECH_ACCEL:
    m->iq_ref = m->iq_test;                 /* +torque, natural direction */
    if (m->idx < (int32_t)IMECH_N_GATES && rpm >= imech_gates_rpm[m->idx]) {
      m->t_up[m->idx] = m->tick;
      m->idx++;
    }
    if (m->idx >= (int32_t)IMECH_N_GATES) { /* crossed the top gate */
      m->idx = (int32_t)IMECH_N_GATES - 1;  /* first downward gate to look for */
      m->timeout = 0u;
      m->phase = IMECH_DECEL;
    } else if (m->timeout >= IMECH_ACCEL_TIMEOUT || rpm > IMECH_ABORT_RPM) {
      m->iq_ref = 0.0f;                      /* could not reach top / too fast */
      m->phase = IMECH_DONE;
    }
    break;

  case IMECH_DECEL:
    m->iq_ref = -m->iq_test;                 /* active braking */
    if (m->idx >= 0 && rpm <= imech_gates_rpm[m->idx]) {
      m->t_dn[m->idx] = m->tick;
      m->idx--;
    }
    if (m->idx < 0) {                        /* crossed the bottom gate */
      m->iq_ref = 0.0f;                      /* stop driving, coast to rest */
      imech_compute(motor);
      m->phase = IMECH_DONE;
    } else if (m->timeout >= IMECH_DECEL_TIMEOUT) {
      m->iq_ref = 0.0f;
      m->phase = IMECH_DONE;
    }
    break;

  case IMECH_DONE:
  default:
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
  m->phase = IMECH_ACCEL;
  m->kt = kt;
  m->iq_test = iq_test;
  m->iq_ref = 0.0f;
  m->tick = 0u;
  m->timeout = 0u;
  m->idx = 0;
  for (uint32_t i = 0u; i < IMECH_N_GATES; i++) {
    m->t_up[i] = 0u;
    m->t_dn[i] = 0u;
  }
  m->j = 0.0f;
  m->b = 0.0f;
  m->tc = 0.0f;
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
