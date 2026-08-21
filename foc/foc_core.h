#ifndef FOC_CORE_H
#define FOC_CORE_H

#include <stdint.h>
#include "foc_math.h"
#include "pid.h"
#include "pll.h"
#include "svpwm.h"
#include "motor_hw.h"

/* motor_t is composed at the bottom of this header but forward-declared (tagged)
 * here so the sub-state-machine headers can take motor_t* parameters. Guard +
 * `struct motor_s` so the identical forward declarations in foc_ident.h and
 * foc_ltu.h coexist with this one. */
#ifndef FOC_MOTOR_T_DEFINED
#define FOC_MOTOR_T_DEFINED
typedef struct motor_s motor_t;
#endif
#include "foc_ident.h"   /* ident_ctx_t / imech_ctx_t + the ident/imech sub-state machines */
#include "foc_ltu.h"     /* ltu_ctx_t + the encoder-linearization sub-state machine */

/* Top-level control state machine. */
typedef enum {
  FOC_STATE_IDLE = 0,        /* outputs off, no drive */
  FOC_STATE_CALI,            /* current-offset + encoder zero alignment */
  FOC_STATE_OPENLOOP_VOLT,   /* voltage at an internally ramped angle (no encoder) */
  FOC_STATE_CLOSEDLOOP_VOLT, /* voltage in the dq frame at the measured angle */
  FOC_STATE_CURRENT,         /* id/iq current loop */
  FOC_STATE_SPEED,           /* speed loop -> current loop */
  FOC_STATE_POSITION,        /* position loop -> speed -> current loop */
  FOC_STATE_MIT,             /* MIT impedance control: PD + torque FF -> current loop */
  FOC_STATE_IDENT,           /* electrical identification (Rs/Ld/Lq/flux) */
  FOC_STATE_IMECH,           /* mechanical identification (J/B/Tc), spins rotor */
  FOC_STATE_LTU,             /* encoder linearization calibration (open-loop sweep) */
  FOC_STATE_ERROR,           /* fault latched, outputs off */
} foc_state_t;

/* Calibration sub-phases (only meaningful in FOC_STATE_CALI). */
typedef enum {
  CALI_CURRENT_OFFSET = 0,   /* average phase-current ADC at zero current */
  CALI_ALIGN_RAMP,           /* ramp Id at forced angle 0 */
  CALI_ALIGN_SETTLE,         /* hold, let the rotor settle on phase A */
  CALI_ALIGN_MEASURE,        /* read encoder -> store zero_offset, mech_at_align */
  CALI_DIR_STEP,             /* shift to +90 deg elec, wait for rotor to follow */
  CALI_DIR_READ,             /* compare mech_angle change -> determine direction */
  CALI_DONE,
} cali_phase_t;

typedef struct {
  cali_phase_t phase;
  uint8_t with_align;        /* 0 = current offset only, 1 = also align encoder */
  uint32_t counter;          /* per-phase sample/time counter */
  uint32_t sum_u, sum_v, sum_w;
  float id_ref;              /* ramped alignment current */
  float mech_at_align;       /* mech_angle captured at theta_e=0 (before dir step) */
} cali_ctx_t;

/* ident_ctx_t (FOC_STATE_IDENT) and imech_ctx_t (FOC_STATE_IMECH) now live in
 * foc_ident.h, included above, and are composed into motor_t below. */

/* Position controller with feed-forward and S-curve profiling */
typedef struct {
  float target_pos;
  float current_pos;
  float last_target_pos;

  int32_t total_turns;
  float last_theta;
  uint8_t initialized;   /* 0 until first encoder reading — avoids spurious turn count */

  float max_rpm;
  float max_accel;
  float max_jerk;
  float landing_kp;      /* proportional gain near target [rad/s per rad] = position bandwidth */
  float deadband;        /* position deadband [rad] (stops final hunting) */
  float last_output_rpm;
} pos_controller_t;

/* Domain sub-structs — group related data by responsibility */

typedef struct {
  encoder_data_t encoder;
  phase_currents_t currents;
  current_offset_t cur_offset;
} motor_sensor_t;

typedef struct {
  clarke_t clarke;
  park_t park;
  inv_park_t inv_park;
  svpwm_t svpwm;
} motor_xform_t;

typedef struct {
  pid_t id;
  pid_t iq;
  pid_t speed;
  pos_controller_t pos;
  pll_t speed_pll;
} motor_ctrl_t;

typedef struct {
  float vd;       /* voltage command (open/closed-loop voltage modes) */
  float vq;
  float id;
  float iq;
  float rpm;      /* also the open-loop speed command */
  float pos;
} motor_target_t;

typedef struct {
  float id;
  float iq;
  float rpm;
  float pos;
  float vd;       /* applied d-axis voltage (current-loop output) — for ident/telemetry */
  float vq;       /* applied q-axis voltage (current-loop output) */
} motor_actual_t;

/* MIT impedance-control command (Mini Cheetah style). Received via CAN as a
 * single 8-byte frame and consumed by the 20 kHz current-loop block: the PD
 * torque (tau_ff + kp*pos_err + kd*vel_err) is converted to iq by Kt. `active`
 * is cleared on mode entry and set by the first command frame, so the motor
 * holds zero torque until a setpoint actually arrives (soft start). */
typedef struct {
  float pos_ref;      /* desired mechanical position [rad], single-turn [0, 2pi) */
  float vel_ref;      /* desired mechanical velocity [rad/s] */
  float kp;           /* position stiffness [Nm/rad] */
  float kd;           /* velocity damping [Nm*s/rad] */
  float tau_ff;       /* torque feed-forward [Nm] */
  uint8_t active;     /* 1 once a valid MIT command has been received */
} mit_cmd_t;

struct motor_s {
  foc_state_t state;

  motor_sensor_t sensor;
  motor_xform_t xform;
  motor_ctrl_t ctrl;
  motor_target_t target;
  motor_actual_t actual;

  mit_cmd_t mit_cmd;          /* live MIT impedance command (written by CAN) */
  float     mit_kt;           /* user-specified torque constant [Nm/A]; 0 = auto from flux */
  float     mit_effective_kt; /* actual Kt used by the ISR [Nm/A]; 0 = unavailable */
  float     mit_cur_lim;      /* MIT torque-current ceiling [A]; 0 = follow cur_limit */

  cali_ctx_t cali;
  ident_ctx_t ident;
  imech_ctx_t imech;
  ltu_ctx_t ltu;
  float openloop_angle;   /* electrical angle for FOC_STATE_OPENLOOP_VOLT */

  float spd_cmd_rpm;      /* user-commanded speed setpoint (before slew ramp) */
  float spd_accel;        /* speed slew rate [rpm/s]; 0 = instant (backward compatible) */
  float regen_margin;     /* V margin below ov_trip to begin regen limiting */

  /* Feed-forward (P0): accel+friction torque FF into the speed loop and dq
   * cross-coupling decoupling at the current loop. The enable switches are
   * params-driven (ff_en / dcpl_en, default on; inert until the relevant ident
   * has run); prev_rpm_ref and iq_ff are ISR state written by the speed loop. */
  uint8_t ff_en;          /* 1: iq_ff = (J*a_ref + B*w + Tc*sign(w)) / Kt, SPEED/POSITION */
  uint8_t decouple_en;    /* 1: Vd_ff = -we*Lq*Iq, Vq_ff = we*(Ld*Id+psi), closed-loop current */
  float   prev_rpm_ref;   /* target.rpm of the last speed-loop tick (accel diff) */
  float   iq_ff;          /* last feed-forward torque current [A] (telemetry) */

  float vdc;
  uint32_t pwm_period;

  uint8_t speed_loop_cnt;
  uint8_t pos_loop_cnt;
};

void foc_init(motor_t *motor);

/* State machine */
void foc_set_state(motor_t *motor, foc_state_t state);
foc_state_t foc_get_state(const motor_t *motor);
void foc_fault(motor_t *motor);                 /* force ERROR */
void foc_clear_fault(motor_t *motor);           /* ERROR -> IDLE */

/* Trigger an extra calibration. with_align != 0 also re-aligns the encoder
 * zero (the motor will twitch to phase A). */
void foc_start_calibration(motor_t *motor, uint8_t with_align);

/* foc_start_ident() / foc_start_identmech() are declared in foc_ident.h. */

/* Setpoints (take effect in the matching state) */
void foc_set_voltage(motor_t *motor, float vd, float vq);
void foc_set_openloop_speed(motor_t *motor, float rpm);
void foc_set_target_current(motor_t *motor, float id, float iq);
void foc_set_target_speed(motor_t *motor, float rpm);
void foc_set_target_position(motor_t *motor, float pos_rad);

/* MIT impedance control: update the 5 command parameters atomically (IRQ-safe).
 * If the motor is not yet in FOC_STATE_MIT, the caller must transition first. */
void foc_set_mit_command(motor_t *motor, float pos_ref, float vel_ref,
                         float kp, float kd, float tau_ff);

/* Live tuning (used by the shell). Selectors for the gain/limit setters. */
typedef enum { FOC_GAIN_ID = 0, FOC_GAIN_IQ, FOC_GAIN_SPEED } foc_gain_sel_t;
typedef enum { FOC_LIMIT_CURRENT = 0, FOC_LIMIT_SPEED } foc_limit_sel_t;

/* Update PID gains of a loop without resetting its state. */
void foc_set_pid_gains(motor_t *motor, foc_gain_sel_t sel, float kp, float ki);

/* Update the output + integral clamp of the current loops (both id/iq) or the
 * speed loop. `lim` is applied symmetrically (+/-lim). */
void foc_set_limit(motor_t *motor, foc_limit_sel_t sel, float lim);

/* Called every PWM period from the ADC inserted-conversion ISR. */
void foc_control_loop(motor_t *motor);

/* Called at the top of every control tick to convert the raw encoder word
 * (from the DMA read kicked at the end of the previous tick) into mech/elec
 * angle. Runs in the control-loop context only — never from the DMA ISR. */
void foc_encoder_callback(motor_t *motor);

#endif /* FOC_CORE_H */
