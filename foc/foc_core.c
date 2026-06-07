#include "foc_core.h"
#include <math.h>
#include "fault.h"
#include "vofa.h"

/* Current loop defaults — deployed from the identified motor parameters
 * (Rs=0.243 ohm, Ld=110 uH, Lq=158 uH) at a 1 kHz closed-loop bandwidth via
 * pole-zero cancellation:  Kp = L*w_bw,  Ki = R*w_bw  (w_bw = 2*pi*1000).
 * Ki is the continuous-domain gain; the integrator scales it by the loop Ts.
 * Re-derive after a fresh `ident` with the `tune` shell command. */
#define DEFAULT_ID_KP           0.69f    /* Ld * w_bw */
#define DEFAULT_ID_KI           1527.0f  /* Rs * w_bw */
#define DEFAULT_IQ_KP           1.00f    /* Lq * w_bw (higher than d: Lq > Ld) */
#define DEFAULT_IQ_KI           1527.0f  /* Rs * w_bw */
#define DEFAULT_CURRENT_LIMIT   12.0f

/* Speed loop defaults — deployed from the identified J/B (J=4.82e-5 kg.m^2,
 * B=2.79e-4 Nm.s/rad) at a 10 Hz bandwidth via pole-zero cancellation:
 *   Kp = (J*w_s/Kt)*(2pi/60),  Ki = (B*w_s/Kt)*(2pi/60),  w_s = 2*pi*10.
 * Gains are A per RPM (the loop runs on rpm error); Ts = 1 ms. Ki/Kp = B/J
 * places the integral zero on the mechanical pole. Re-derive after a fresh
 * identmech with the `tunespd` shell command. */
#define DEFAULT_SPD_KP          0.0039f
#define DEFAULT_SPD_KI          0.0227f
#define DEFAULT_SPEED_LIMIT     5.0f

/* Position loop defaults (cascade: position -> speed loop -> current loop)
 * max_rpm     : peak speed during a position move (RPM mechanical).
 * max_accel   : deceleration constant for the brake curve (rad/s^2): v_brake = sqrt(2*a*|error|)
 * max_jerk    : max RPM change per position-loop tick (1 ms) = the acceleration limit.
 * landing_kp  : proportional gain near the target [rad/s per rad]; this IS the
 *               position bandwidth and dominates moves smaller than ~max speed.
 *               Keep below ~1/3 of the speed-loop bandwidth (10 Hz -> ~20 rad/s).
 * deadband    : position deadband [rad] to stop final hunting (~0.1 deg rotor). */
#define DEFAULT_MAX_RPM             2000.0f
#define DEFAULT_MAX_ACCEL           500.0f
#define DEFAULT_MAX_JERK            20.0f
#define DEFAULT_POS_LANDING_KP      12.0f    /* ~1.9 Hz, below speed loop / 3 */
#define DEFAULT_POS_DEADBAND        0.002f
#define POS_CROSSOVER_THRESHOLD     5.0f

/* PLL defaults */
#define DEFAULT_PLL_KP          200.0f
#define DEFAULT_PLL_KI          40000.0f

/* Control periods */
#define CONTROL_DT              0.00005f
#define POS_LOOP_DT             0.001f

/* Calibration timing (ticks at the 20 kHz PWM rate) and alignment current */
#define CALI_OFFSET_SAMPLES     2000u   /* ~100 ms current-offset averaging */
#define CALI_RAMP_TICKS         4000u   /* ~200 ms ramp to alignment current */
#define CALI_SETTLE_TICKS       10000u  /* ~500 ms let the rotor settle */
#define CALI_DIR_TICKS          4000u   /* ~200 ms hold at +90 deg for direction detect */
#define ALIGN_CURRENT           4.0f    /* A — forced Id to align the rotor to phase A */

// Electrical angle used for direction detection: π/2 = one quarter of an
// electrical revolution, small enough not to stress the motor but large enough
// to produce a detectable mechanical movement even through a gearbox.
#define CALI_DIR_THETA          FOC_PI_2

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

/* Internal helpers */

static inline float min_f(float a, float b) {
  return (a < b) ? a : b;
}

static inline float normalize_angle(float angle) {
  while (angle >= FOC_2PI) angle -= FOC_2PI;
  while (angle < 0.0f)     angle += FOC_2PI;
  return angle;
}

// Phases that hold the rotor at theta_e = 0 and drive current.
static inline uint8_t cali_is_align_phase(cali_phase_t p) {
  return (p == CALI_ALIGN_RAMP) || (p == CALI_ALIGN_SETTLE) || (p == CALI_ALIGN_MEASURE);
}

// Phases that drive current at theta_e = CALI_DIR_THETA to detect direction.
static inline uint8_t cali_is_dir_phase(cali_phase_t p) {
  return (p == CALI_DIR_STEP) || (p == CALI_DIR_READ);
}

// Rs phases run the closed current loop on a DC d-current command.
static inline uint8_t ident_is_rs_phase(ident_phase_t p) {
  return (p <= IDENT_RS_MEAS_HIGH);
}

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

static void pos_controller_init(pos_controller_t *pos) {
  pos->target_pos = 0.0f;
  pos->current_pos = 0.0f;
  pos->last_target_pos = 0.0f;
  pos->total_turns = 0;
  pos->last_theta = 0.0f;
  pos->initialized = 0;
  pos->max_rpm = DEFAULT_MAX_RPM;
  pos->max_accel = DEFAULT_MAX_ACCEL;
  pos->max_jerk = DEFAULT_MAX_JERK;
  pos->landing_kp = DEFAULT_POS_LANDING_KP;
  pos->deadband = DEFAULT_POS_DEADBAND;
  pos->last_output_rpm = 0.0f;
}

static void pos_controller_update_sensor(pos_controller_t *pos, float mech_theta) {
  if (!pos->initialized) {
    /* Seed the tracker with the actual encoder angle so the first delta is zero.
     * current_pos stays 0 — the calibration home position is "position 0". */
    pos->last_theta = mech_theta;
    pos->current_pos = 0.0f;
    pos->total_turns = 0;
    pos->initialized = 1;
    return;
  }

  float delta = mech_theta - pos->last_theta;

  /* Unwrap 0↔2π crossings. Threshold must sit above max angle per tick
   * (≈0.21 rad at 2000 RPM / 1 ms) and well below π. */
  if (delta < -POS_CROSSOVER_THRESHOLD) delta += FOC_2PI;
  else if (delta >  POS_CROSSOVER_THRESHOLD) delta -= FOC_2PI;

  pos->current_pos += delta;
  pos->last_theta = mech_theta;
  pos->total_turns = (int32_t)(pos->current_pos / FOC_2PI);
}

static float pos_controller_calc(pos_controller_t *pos) {
  /* Feed-forward velocity */
  float delta_target = pos->target_pos - pos->last_target_pos;
  float ff_velocity_rpm = (delta_target / POS_LOOP_DT) * FOC_RAD_TO_RPM;
  pos->last_target_pos = pos->target_pos;

  /* Position error with deadband */
  float error = pos->target_pos - pos->current_pos;
  float sign = (error > 0.0f) ? 1.0f : -1.0f;
  float abs_err = fabsf(error);
  if (abs_err < pos->deadband) abs_err = 0.0f;

  /* Speed profile: min of cruise, brake, landing */
  float v_cruise = pos->max_rpm * FOC_RPM_TO_RAD;
  float v_brake = sqrtf(2.0f * pos->max_accel * abs_err);
  float v_land = pos->landing_kp * abs_err;

  float fb_rad_s = min_f(v_cruise, min_f(v_brake, v_land));
  float fb_rpm = sign * fb_rad_s * FOC_RAD_TO_RPM;

  /* Superimpose feed-forward */
  float final_rpm = fb_rpm + ff_velocity_rpm;

  /* Jerk limiting */
  float rpm_diff = final_rpm - pos->last_output_rpm;
  if (rpm_diff > pos->max_jerk) {
    final_rpm = pos->last_output_rpm + pos->max_jerk;
  } else if (rpm_diff < -pos->max_jerk) {
    final_rpm = pos->last_output_rpm - pos->max_jerk;
  }
  pos->last_output_rpm = final_rpm;

  /* Speed clamp */
  if (final_rpm > pos->max_rpm)  final_rpm = pos->max_rpm;
  if (final_rpm < -pos->max_rpm) final_rpm = -pos->max_rpm;

  return final_rpm;
}

/* Advance the calibration sub-state machine (called while in FOC_STATE_CALI). */
static void cali_update(motor_t *motor) {
  cali_ctx_t *c = &motor->cali;

  switch (c->phase) {
  case CALI_CURRENT_OFFSET: {
    uint32_t au, av, aw;
    motor_hw_get_current_adc(&au, &av, &aw);
    c->sum_u += au;
    c->sum_v += av;
    c->sum_w += aw;
    c->counter++;
    if (c->counter >= CALI_OFFSET_SAMPLES) {
      motor->sensor.cur_offset.offset_u = (float)c->sum_u / (float)CALI_OFFSET_SAMPLES;
      motor->sensor.cur_offset.offset_v = (float)c->sum_v / (float)CALI_OFFSET_SAMPLES;
      motor->sensor.cur_offset.offset_w = (float)c->sum_w / (float)CALI_OFFSET_SAMPLES;
      motor->sensor.cur_offset.is_calibrated = 1;

      if (c->with_align) {
        c->phase = CALI_ALIGN_RAMP;
        c->counter = 0;
        c->id_ref = 0.0f;
        pid_reset(&motor->ctrl.id);
        pid_reset(&motor->ctrl.iq);
        motor_hw_enable_driver();  /* alignment needs PWM output */
      } else {
        c->phase = CALI_DONE;
      }
    }
    break;
  }

  case CALI_ALIGN_RAMP:
    c->id_ref += ALIGN_CURRENT / (float)CALI_RAMP_TICKS;
    if (c->id_ref >= ALIGN_CURRENT) {
      c->id_ref = ALIGN_CURRENT;
      c->phase = CALI_ALIGN_SETTLE;
      c->counter = 0;
    }
    break;

  case CALI_ALIGN_SETTLE:
    c->counter++;
    if (c->counter >= CALI_SETTLE_TICKS) {
      c->phase = CALI_ALIGN_MEASURE;
    }
    break;

  case CALI_ALIGN_MEASURE:
    /* Rotor settled at theta_e=0. Record zero_offset and the mech_angle for the
     * upcoming direction test, then advance the commanded angle to π/2. */
    motor->sensor.encoder.zero_offset = motor->sensor.encoder.mech_angle;
    c->mech_at_align = motor->sensor.encoder.mech_angle;
    c->id_ref = ALIGN_CURRENT;  /* keep holding during direction step */
    c->counter = 0;
    c->phase = CALI_DIR_STEP;
    break;

  case CALI_DIR_STEP:
    /* Hold at CALI_DIR_THETA and wait for the rotor to follow. */
    c->counter++;
    if (c->counter >= CALI_DIR_TICKS) {
      c->phase = CALI_DIR_READ;
    }
    break;

  case CALI_DIR_READ: {
    /* The rotor has been pulled toward theta_e = CALI_DIR_THETA. If mech_angle
     * increased, the current direction setting is correct. If it decreased,
     * direction must be flipped and zero_offset must be recalculated. */
    float delta = motor->sensor.encoder.mech_angle - c->mech_at_align;
    if (delta >  FOC_PI)  delta -= FOC_2PI;
    if (delta < -FOC_PI)  delta += FOC_2PI;

    if (delta < 0.0f) {
      // Rotor moved opposite to what direction=1 predicts → flip.
      motor->sensor.encoder.direction = -1;
      // Recalculate zero_offset for the new direction.
      // With dir=-1: mech_angle = normalize(-raw * RAW_TO_RAD)
      //            = normalize(2π - raw * RAW_TO_RAD)
      //            = 2π - zero_offset_dir1  (since zero_offset_dir1 = raw * RAW_TO_RAD)
      motor->sensor.encoder.zero_offset =
          normalize_angle(FOC_2PI - c->mech_at_align);
    }
    // else: direction=1 (the HW_ENCODER_DIR default) was already correct.

    c->id_ref = 0.0f;
    c->phase = CALI_DONE;
    break;
  }

  case CALI_DONE:
  default:
    foc_set_state(motor, FOC_STATE_IDLE);
    break;
  }
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
  if (i > c->i_top) c->i_top = i;
  if (i < c->i_bot) c->i_bot = i;

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
 * The d-current / open-loop voltage commands are produced here; foc_control_loop
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
    if (c->id_ref > IDENT_L_I_BIAS) c->id_ref = IDENT_L_I_BIAS;
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
 * and the friction model. iq_ref is consumed by foc_control_loop's IMECH case. */
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

void foc_init(motor_t *motor) {
  motor->state = FOC_STATE_IDLE;

  motor->sensor.encoder.direction = HW_ENCODER_DIR;
  motor->sensor.encoder.zero_offset = HW_ENCODER_ZERO_OFFSET;

  motor->vdc = 12.0f;
  motor->pwm_period = HW_PWM_PERIOD;
  motor->openloop_angle = 0.0f;

  pid_init(&motor->ctrl.id, DEFAULT_ID_KP, DEFAULT_ID_KI, 0.0f, CONTROL_DT,
           DEFAULT_CURRENT_LIMIT, -DEFAULT_CURRENT_LIMIT);
  pid_init(&motor->ctrl.iq, DEFAULT_IQ_KP, DEFAULT_IQ_KI, 0.0f, CONTROL_DT,
           DEFAULT_CURRENT_LIMIT, -DEFAULT_CURRENT_LIMIT);
  pid_init(&motor->ctrl.speed, DEFAULT_SPD_KP, DEFAULT_SPD_KI, 0.0f,
           CONTROL_DT * (float)HW_SPEED_LOOP_DIV,
           DEFAULT_SPEED_LIMIT, -DEFAULT_SPEED_LIMIT);
  pos_controller_init(&motor->ctrl.pos);
  pll_init(&motor->ctrl.speed_pll, DEFAULT_PLL_KP, DEFAULT_PLL_KI);

  motor->xform.svpwm.ts = motor->pwm_period;
  motor->xform.svpwm.udc = motor->vdc;

  motor->speed_loop_cnt = 0;
  motor->pos_loop_cnt = 0;

  motor_hw_init();

  /* Power-up always runs one current-offset calibration (no encoder align). */
  foc_start_calibration(motor, 0);
}

void foc_set_state(motor_t *motor, foc_state_t state) {
  /* Once in ERROR, only an explicit clear (-> IDLE) may leave it. */
  if (motor->state == FOC_STATE_ERROR && state != FOC_STATE_IDLE) {
    return;
  }

  switch (state) {
  case FOC_STATE_OPENLOOP_VOLT:
    pid_reset(&motor->ctrl.id);
    pid_reset(&motor->ctrl.iq);
    motor->openloop_angle = motor->sensor.encoder.elec_angle;
    motor_hw_enable_driver();
    break;

  case FOC_STATE_CLOSEDLOOP_VOLT:
    pid_reset(&motor->ctrl.id);
    pid_reset(&motor->ctrl.iq);
    motor_hw_enable_driver();
    break;

  case FOC_STATE_CURRENT:
  case FOC_STATE_SPEED:
    if (!motor->sensor.cur_offset.is_calibrated) {
      state = FOC_STATE_ERROR;
      motor_hw_disable_driver();
      motor_hw_set_pwm_brake();
      break;
    }
    pid_reset(&motor->ctrl.id);
    pid_reset(&motor->ctrl.iq);
    pid_reset(&motor->ctrl.speed);
    motor_hw_enable_driver();
    break;

  case FOC_STATE_POSITION:
    if (!motor->sensor.cur_offset.is_calibrated) {
      state = FOC_STATE_ERROR;
      motor_hw_disable_driver();
      motor_hw_set_pwm_brake();
      break;
    }
    pid_reset(&motor->ctrl.id);
    pid_reset(&motor->ctrl.iq);
    pid_reset(&motor->ctrl.speed);
    /* Hold the current position on entry — avoids a sudden jump if target and
     * actual are not yet matched. Clear the jerk limiter state too. */
    motor->ctrl.pos.target_pos      = motor->ctrl.pos.current_pos;
    motor->ctrl.pos.last_target_pos = motor->ctrl.pos.current_pos;
    motor->ctrl.pos.last_output_rpm = 0.0f;
    motor->target.pos               = motor->ctrl.pos.current_pos;
    motor_hw_enable_driver();
    break;

  case FOC_STATE_CALI:
    /* Offset phase runs with zero current, so keep the outputs disabled until
     * the (optional) alignment phase enables them. */
    pid_reset(&motor->ctrl.id);
    pid_reset(&motor->ctrl.iq);
    motor_hw_disable_driver();
    break;

  case FOC_STATE_IDENT:
    /* Needs valid current offsets (same as the current loop). The Rs phase
     * drives a DC d-current through the id/iq loops, so enable the outputs. */
    if (!motor->sensor.cur_offset.is_calibrated) {
      state = FOC_STATE_ERROR;
      motor_hw_disable_driver();
      motor_hw_set_pwm_brake();
      break;
    }
    pid_reset(&motor->ctrl.id);
    pid_reset(&motor->ctrl.iq);
    motor_hw_enable_driver();
    break;

  case FOC_STATE_IMECH:
    /* Mechanical ident spins the rotor under current control on the measured
     * angle, so it needs valid offsets (the caller also requires alignment). */
    if (!motor->sensor.cur_offset.is_calibrated) {
      state = FOC_STATE_ERROR;
      motor_hw_disable_driver();
      motor_hw_set_pwm_brake();
      break;
    }
    pid_reset(&motor->ctrl.id);
    pid_reset(&motor->ctrl.iq);
    motor_hw_enable_driver();
    break;

  case FOC_STATE_IDLE:
    motor_hw_disable_driver();
    motor_hw_set_pwm_brake();
    break;

  case FOC_STATE_ERROR:
  default:
    state = FOC_STATE_ERROR;
    motor_hw_disable_driver();
    motor_hw_set_pwm_brake();
    break;
  }

  motor->state = state;
}

foc_state_t foc_get_state(const motor_t *motor) {
  return motor->state;
}

void foc_fault(motor_t *motor) {
  foc_set_state(motor, FOC_STATE_ERROR);
}

void foc_clear_fault(motor_t *motor) {
  if (motor->state != FOC_STATE_ERROR) {
    return;
  }
  /* Only leave ERROR if every active protection condition has physically
   * cleared (bus back in range, DRV nFAULT released, etc.). Latching faults
   * (hard overcurrent, GDF, encoder, phase loss) re-arm and will trip again
   * immediately if the underlying problem is still present. */
  float vbus = motor_hw_get_vbus();
  if (fault_try_clear(vbus)) {
    foc_set_state(motor, FOC_STATE_IDLE);
  }
}

void foc_start_calibration(motor_t *motor, uint8_t with_align) {
  motor->cali.phase = CALI_CURRENT_OFFSET;
  motor->cali.with_align = with_align;
  motor->cali.counter = 0;
  motor->cali.sum_u = 0;
  motor->cali.sum_v = 0;
  motor->cali.sum_w = 0;
  motor->cali.id_ref = 0.0f;
  motor->sensor.cur_offset.is_calibrated = 0;
  foc_set_state(motor, FOC_STATE_CALI);
}

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

void foc_set_voltage(motor_t *motor, float vd, float vq) {
  motor->target.vd = vd;
  motor->target.vq = vq;
}

void foc_set_openloop_speed(motor_t *motor, float rpm) {
  motor->target.rpm = rpm;
}

void foc_set_target_current(motor_t *motor, float id, float iq) {
  motor->target.id = id;
  motor->target.iq = iq;
}

void foc_set_target_speed(motor_t *motor, float rpm) {
  motor->target.rpm = rpm;
}

void foc_set_target_position(motor_t *motor, float pos_rad) {
  motor->target.pos = pos_rad;
  motor->ctrl.pos.target_pos = pos_rad;
}

void foc_set_pid_gains(motor_t *motor, foc_gain_sel_t sel, float kp, float ki) {
  pid_t *p = (sel == FOC_GAIN_ID)   ? &motor->ctrl.id
           : (sel == FOC_GAIN_IQ)   ? &motor->ctrl.iq
                                    : &motor->ctrl.speed;
  p->kp = kp;
  p->ki = ki;
}

static void set_pid_limit(pid_t *p, float lim) {
  p->out_max = lim;
  p->out_min = -lim;
  p->integral_max = lim;
  p->integral_min = -lim;
}

void foc_set_limit(motor_t *motor, foc_limit_sel_t sel, float lim) {
  if (sel == FOC_LIMIT_CURRENT) {
    set_pid_limit(&motor->ctrl.id, lim);
    set_pid_limit(&motor->ctrl.iq, lim);
  } else {
    set_pid_limit(&motor->ctrl.speed, lim);
  }
}

void foc_encoder_callback(motor_t *motor) {
  motor_hw_process_encoder_data(&motor->sensor.encoder);
}

void foc_control_loop(motor_t *motor) {
  motor_hw_debug_pin(1);

  /* Track the real bus voltage for correct SVPWM scaling. */
  float vbus = motor_hw_get_vbus();
  if (vbus > 1.0f) {
    motor->vdc = vbus;
  }

  /* 1. Sample phase currents */
  motor_hw_get_currents(&motor->sensor.currents, &motor->sensor.cur_offset);

  /* L2 fast protection: software overcurrent, bus overvoltage, DRV nFAULT pin.
   * Trip into ERROR (outputs off) the moment any of them fires.
   * Current-based checks require valid offsets: before the current-offset
   * calibration completes, the readings are not real currents (the mid-scale
   * ADC value would look like ~16 A and falsely trip the overcurrent guard). */
  uint8_t driving =
      (motor->state != FOC_STATE_IDLE && motor->state != FOC_STATE_ERROR &&
       motor->sensor.cur_offset.is_calibrated);
  if (fault_check_fast(motor->sensor.currents.iu, motor->sensor.currents.iv,
                       motor->sensor.currents.iw, vbus, motor->actual.iq,
                       driving != 0u)) {
    if (driving) {
      foc_fault(motor);
    }
  }

  /* 2. Start encoder read (async via DMA, processed in foc_encoder_callback) */
  motor_hw_start_encoder_read();

  /* 3. Clarke transform: Iabc → Iαβ */
  motor->xform.clarke.ia = motor->sensor.currents.iu;
  motor->xform.clarke.ib = motor->sensor.currents.iv;
  motor->xform.clarke.ic = motor->sensor.currents.iw;
  clarke_calc(&motor->xform.clarke);

  /* 4. PLL speed estimation */
  pll_update(&motor->ctrl.speed_pll, motor->sensor.encoder.mech_angle, CONTROL_DT);
  motor->actual.rpm = motor->ctrl.speed_pll.speed_rpm;

  /* 5. Select the electrical angle used by this cycle */
  float ctrl_theta;
  if (motor->state == FOC_STATE_OPENLOOP_VOLT) {
    float elec_speed = motor->target.rpm * FOC_RPM_TO_RAD * (float)HW_MOTOR_POLE_PAIRS;
    motor->openloop_angle = normalize_angle(motor->openloop_angle + elec_speed * CONTROL_DT);
    ctrl_theta = motor->openloop_angle;
  } else if (motor->state == FOC_STATE_CALI && cali_is_align_phase(motor->cali.phase)) {
    ctrl_theta = 0.0f;             /* phase A for alignment */
  } else if (motor->state == FOC_STATE_CALI && cali_is_dir_phase(motor->cali.phase)) {
    ctrl_theta = CALI_DIR_THETA;   /* shifted angle for direction detection */
  } else if (motor->state == FOC_STATE_IDENT && ident_is_locked_phase(motor->ident.phase)) {
    ctrl_theta = 0.0f;             /* lock rotor to phase A for Rs / Ld / Lq */
  } else {
    ctrl_theta = motor->sensor.encoder.elec_angle;
  }

  /* 6. Park transform: Iαβ → Idq */
  motor->xform.park.alpha = motor->xform.clarke.alpha;
  motor->xform.park.beta  = motor->xform.clarke.beta;
  motor->xform.park.theta = ctrl_theta;
  park_calc(&motor->xform.park);

  motor->actual.id = motor->xform.park.d;
  motor->actual.iq = motor->xform.park.q;

  /* 7. Position loop (decimated). Track position always; close it in POSITION. */
  motor->pos_loop_cnt++;
  if (motor->pos_loop_cnt >= HW_POS_LOOP_DIV) {
    motor->pos_loop_cnt = 0;

    pos_controller_update_sensor(&motor->ctrl.pos, motor->sensor.encoder.mech_angle);
    motor->actual.pos = motor->ctrl.pos.current_pos;

    if (motor->state == FOC_STATE_POSITION) {
      motor->target.rpm = pos_controller_calc(&motor->ctrl.pos);
    }
  }

  /* 8. Speed loop (decimated) */
  motor->speed_loop_cnt++;
  if (motor->speed_loop_cnt >= HW_SPEED_LOOP_DIV) {
    motor->speed_loop_cnt = 0;

    if (motor->state == FOC_STATE_SPEED || motor->state == FOC_STATE_POSITION) {
      motor->target.iq = pi_calc(&motor->ctrl.speed, motor->target.rpm, motor->actual.rpm);
    }
  }

  /* 9. Produce the dq voltage for this state */
  float vd_out = 0.0f;
  float vq_out = 0.0f;

  switch (motor->state) {
  case FOC_STATE_OPENLOOP_VOLT:
  case FOC_STATE_CLOSEDLOOP_VOLT:
    vd_out = motor->target.vd;
    vq_out = motor->target.vq;
    break;

  case FOC_STATE_CURRENT:
  case FOC_STATE_SPEED:
  case FOC_STATE_POSITION:
    vd_out = pi_calc(&motor->ctrl.id, motor->target.id, motor->actual.id);
    vq_out = pi_calc(&motor->ctrl.iq, motor->target.iq, motor->actual.iq);
    break;

  case FOC_STATE_CALI:
    cali_update(motor);
    if (cali_is_align_phase(motor->cali.phase) || cali_is_dir_phase(motor->cali.phase)) {
      vd_out = pi_calc(&motor->ctrl.id, motor->cali.id_ref, motor->actual.id);
      vq_out = pi_calc(&motor->ctrl.iq, 0.0f, motor->actual.iq);
    }
    break;

  case FOC_STATE_IDENT:
    ident_update(motor);
    if (ident_is_inj_phase(motor->ident.phase)) {
      vd_out = motor->ident.vd_cmd;   /* open-loop square-wave injection */
      vq_out = motor->ident.vq_cmd;
    } else if (ident_is_locked_phase(motor->ident.phase)) {
      /* Rs phases + L bias: closed current loop on the d-current command. */
      vd_out = pi_calc(&motor->ctrl.id, motor->ident.id_ref, motor->actual.id);
      vq_out = pi_calc(&motor->ctrl.iq, 0.0f, motor->actual.iq);
    } else if (ident_is_flux_phase(motor->ident.phase)) {
      /* Flux spin-up/coast: Id=0, Iq from the spin command (0 while coasting,
       * so Vq settles to the back-EMF). Runs on the measured electrical angle. */
      vd_out = pi_calc(&motor->ctrl.id, 0.0f, motor->actual.id);
      vq_out = pi_calc(&motor->ctrl.iq, motor->ident.iq_ref, motor->actual.iq);
    }
    break;

  case FOC_STATE_IMECH:
    /* Mechanical ident: torque mode (Id=0, Iq = +/- iq_test) on the measured
     * angle; the state machine sets iq_ref as it ramps the rotor up and down. */
    imech_update(motor);
    vd_out = pi_calc(&motor->ctrl.id, 0.0f, motor->actual.id);
    vq_out = pi_calc(&motor->ctrl.iq, motor->imech.iq_ref, motor->actual.iq);
    break;

  case FOC_STATE_IDLE:
  case FOC_STATE_ERROR:
  default:
    break;
  }

  /* Record the applied dq voltage (current-loop output) for telemetry and for
   * the parameter-identification routines that read Vd/Vq back. */
  motor->actual.vd = vd_out;
  motor->actual.vq = vq_out;

  /* 10. Inverse Park: Vdq → Vαβ */
  motor->xform.inv_park.d = vd_out;
  motor->xform.inv_park.q = vq_out;
  motor->xform.inv_park.theta = ctrl_theta;
  inv_park_calc(&motor->xform.inv_park);

  /* 11. SVPWM modulation */
  motor->xform.svpwm.alpha = motor->xform.inv_park.alpha;
  motor->xform.svpwm.beta  = motor->xform.inv_park.beta;
  motor->xform.svpwm.udc = motor->vdc;
  motor->xform.svpwm.ts  = motor->pwm_period;
  svpwm_calc(&motor->xform.svpwm);

  /* 12. PWM output */
  motor_hw_set_pwm(motor->xform.svpwm.ccr1, motor->xform.svpwm.ccr2, motor->xform.svpwm.ccr3);

  /* 13. Update telemetry */
  vofa_update_from_motor(motor);

  motor_hw_debug_pin(0);
}
