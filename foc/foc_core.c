#include "foc_core.h"
#include <math.h>
#include "fault.h"
#include "vofa.h"

/* Current loop defaults */
#define DEFAULT_ID_KP           0.72f
#define DEFAULT_ID_KI           0.11f
#define DEFAULT_IQ_KP           0.72f
#define DEFAULT_IQ_KI           0.11f
#define DEFAULT_CURRENT_LIMIT   12.0f

/* Speed loop defaults
 * Speed loop runs at Ts=1ms (HW_SPEED_LOOP_DIV=20 ticks).
 * pi_calc integrates without Ts, so Ki_code = Ki_continuous * 0.001.
 * Kp=0.006 A/RPM: 100 RPM error -> 0.6 A Iq command.
 * Ki=0.0001: integral time ~60 ms, fast enough to reject constant load. */
#define DEFAULT_SPD_KP          0.006f
#define DEFAULT_SPD_KI          0.0001f
#define DEFAULT_SPEED_LIMIT     5.0f

/* Position loop defaults
 * max_rpm  : peak speed during a position move (RPM mechanical).
 * max_accel: deceleration constant for the brake curve (rad/s²): v_brake = sqrt(2 * a * |error|)
 * max_jerk : max RPM change per position-loop tick (1ms) = max acceleration in RPM/ms. */
#define DEFAULT_MAX_RPM             1200.0f
#define DEFAULT_MAX_ACCEL           500.0f
#define DEFAULT_MAX_JERK            20.0f
#define POS_CROSSOVER_THRESHOLD     5.0f
#define POS_DEADBAND_RAD            0.002f
#define POS_LANDING_KP              5.0f

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
  if (abs_err < POS_DEADBAND_RAD) abs_err = 0.0f;

  /* Speed profile: min of cruise, brake, landing */
  float v_cruise = pos->max_rpm * FOC_RPM_TO_RAD;
  float v_brake = sqrtf(2.0f * pos->max_accel * abs_err);
  float v_land = POS_LANDING_KP * abs_err;

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

/* Public API */

void foc_init(motor_t *motor) {
  motor->state = FOC_STATE_IDLE;

  motor->sensor.encoder.direction = HW_ENCODER_DIR;
  motor->sensor.encoder.zero_offset = HW_ENCODER_ZERO_OFFSET;

  motor->vdc = 12.0f;
  motor->pwm_period = HW_PWM_PERIOD;
  motor->openloop_angle = 0.0f;

  pid_init(&motor->ctrl.id, DEFAULT_ID_KP, DEFAULT_ID_KI, 0.0f,
           DEFAULT_CURRENT_LIMIT, -DEFAULT_CURRENT_LIMIT);
  pid_init(&motor->ctrl.iq, DEFAULT_IQ_KP, DEFAULT_IQ_KI, 0.0f,
           DEFAULT_CURRENT_LIMIT, -DEFAULT_CURRENT_LIMIT);
  pid_init(&motor->ctrl.speed, DEFAULT_SPD_KP, DEFAULT_SPD_KI, 0.0f,
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

uint8_t foc_calibration_done(const motor_t *motor) {
  return motor->sensor.cur_offset.is_calibrated;
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

  case FOC_STATE_IDLE:
  case FOC_STATE_ERROR:
  default:
    break;
  }

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
