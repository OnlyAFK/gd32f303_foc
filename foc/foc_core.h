#ifndef FOC_CORE_H
#define FOC_CORE_H

#include <stdint.h>
#include "foc_math.h"
#include "pid.h"
#include "pll.h"
#include "svpwm.h"
#include "motor_hw.h"

/* Top-level control state machine. */
typedef enum {
  FOC_STATE_IDLE = 0,        /* outputs off, no drive */
  FOC_STATE_CALI,            /* current-offset + encoder zero alignment */
  FOC_STATE_OPENLOOP_VOLT,   /* voltage at an internally ramped angle (no encoder) */
  FOC_STATE_CLOSEDLOOP_VOLT, /* voltage in the dq frame at the measured angle */
  FOC_STATE_CURRENT,         /* id/iq current loop */
  FOC_STATE_SPEED,           /* speed loop -> current loop */
  FOC_STATE_POSITION,        /* position loop -> speed -> current loop */
  FOC_STATE_IDENT,           /* electrical identification (Rs/Ld/Lq/flux) */
  FOC_STATE_IMECH,           /* mechanical identification (J/B/Tc), spins rotor */
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

/* Parameter-identification sub-phases (only meaningful in FOC_STATE_IDENT).
 * Built up in stages: Rs, then Ld/Lq, then flux linkage. Later stages reuse the
 * result of the earlier ones (the L bias voltage uses Rs; flux will use the
 * R/L-derived current loop). All phases below IDENT_DONE lock the rotor at
 * theta_e = 0. Keep the order contiguous — the ident_is_* helpers range-test it. */
typedef enum {
  IDENT_RS_RAMP_LOW = 0,  /* ramp Id to the first measurement current */
  IDENT_RS_MEAS_LOW,      /* settle, then average Vd at the first current */
  IDENT_RS_RAMP_HIGH,     /* ramp Id to the second measurement current */
  IDENT_RS_MEAS_HIGH,     /* settle, average Vd, compute Rs = dVd/dId */
  IDENT_L_BIAS,           /* closed-loop DC d-current; capture the bias voltage */
  IDENT_LD_INJ_LOW,       /* d-axis square wave at the low amplitude  -> ripple */
  IDENT_LD_INJ_HIGH,      /* d-axis square wave at the high amplitude -> Ld */
  IDENT_LQ_INJ_LOW,       /* q-axis square wave at the low amplitude  -> ripple */
  IDENT_LQ_INJ_HIGH,      /* q-axis square wave at the high amplitude -> Lq */
  IDENT_FLUX_SPINUP,      /* spin up with Iq (rotor turns) to a target speed */
  IDENT_FLUX_COAST,       /* Iq=Id=0 free-coast; fit Vq = flux * omega_e */
  IDENT_DONE,
} ident_phase_t;

typedef struct {
  ident_phase_t phase;
  uint32_t counter;       /* per-phase tick counter */

  /* Rs — two-point locked-rotor DC */
  float    id_ref;        /* ramped d-current command (rotor locks to phase A) */
  float    vd_sum;        /* Vd accumulator for averaging */
  float    id_sum;        /* measured-Id accumulator (robust denominator) */
  uint32_t n;             /* samples in the accumulators */
  float    vd_low, vd_high;   /* averaged Vd at the two current points */
  float    id_low, id_high;   /* averaged measured Id at the two points */

  /* Ld/Lq — square-wave voltage injection (two amplitudes per axis) */
  float    vd_cmd, vq_cmd;    /* open-loop voltage applied during injection */
  float    vd_bias;           /* DC d-axis hold voltage captured from the PI */
  int8_t   inj_sign;          /* square-wave polarity (+1/-1) */
  uint32_t inj_tick;          /* ticks within the current half-period */
  uint32_t half_count;        /* half-periods elapsed in the current period */
  float    i_top, i_bot;      /* running max/min of the current over a period */
  uint32_t ripple_skip;       /* transient periods skipped before averaging */
  float    ripple_sum;        /* accumulated peak-to-peak ripple */
  uint32_t ripple_n;          /* periods averaged */
  float    ld_pp_low, ld_pp_high;  /* Id ripple at the two d-axis amplitudes */
  float    lq_pp_low, lq_pp_high;  /* Iq ripple at the two q-axis amplitudes */

  /* Flux linkage — Iq=0 coast-down, least-squares fit Vq = flux * omega_e */
  float    iq_ref;            /* q-current command (spin-up) */
  float    flux_sum_vw;       /* accumulator of Vq * omega_e */
  float    flux_sum_ww;       /* accumulator of omega_e^2 */

  /* Results (copied to flash params on the IDENT -> IDLE edge). */
  float rs;               /* stator resistance [ohm] */
  float ld;               /* d-axis inductance [uH] (1e6*H, for clean display) */
  float lq;               /* q-axis inductance [uH] */
  float flux;             /* PM flux linkage [mWb] (1e3*Wb, for clean display) */
} ident_ctx_t;

/* Mechanical identification (FOC_STATE_IMECH): bidirectional torque accel/decel
 * across a set of speed gates, all in current (torque) mode reading the speed
 * feedback. Per band:  J = 2*Te/(a_up + a_dn)  (friction cancels), and the
 * friction torque  f = J*(a_dn - a_up)/2  is fit across bands -> B, Tc. */
#define IMECH_N_GATES 6u
typedef enum {
  IMECH_ACCEL = 0,   /* +Iq: ramp up, timestamp each gate crossing */
  IMECH_DECEL,       /* -Iq: ramp down, timestamp each gate crossing -> compute */
  IMECH_DONE,
} imech_phase_t;

typedef struct {
  imech_phase_t phase;
  float    kt;            /* torque constant Kt = 1.5*p*flux [Nm/A] (passed in) */
  float    iq_test;       /* test current magnitude [A] */
  float    iq_ref;        /* applied q-current (+iq_test / -iq_test / 0) */
  uint32_t tick;          /* free-running tick counter (timestamps) */
  uint32_t timeout;       /* per-phase watchdog counter */
  int32_t  idx;           /* next gate index */
  uint32_t t_up[IMECH_N_GATES];   /* tick at each upward gate crossing */
  uint32_t t_dn[IMECH_N_GATES];   /* tick at each downward gate crossing */

  /* Results (copied to flash params on the IMECH -> IDLE edge). */
  float j;                /* rotor-side inertia [g.cm^2] (1e7*kg.m^2) */
  float b;                /* viscous friction [uNm.s/rad] (1e6*Nm.s/rad) */
  float tc;               /* Coulomb friction [mNm] (1e3*Nm) */
} imech_ctx_t;

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

typedef struct {
  foc_state_t state;

  motor_sensor_t sensor;
  motor_xform_t xform;
  motor_ctrl_t ctrl;
  motor_target_t target;
  motor_actual_t actual;

  cali_ctx_t cali;
  ident_ctx_t ident;
  imech_ctx_t imech;
  float openloop_angle;   /* electrical angle for FOC_STATE_OPENLOOP_VOLT */

  float vdc;
  uint32_t pwm_period;

  uint8_t speed_loop_cnt;
  uint8_t pos_loop_cnt;
} motor_t;

void foc_init(motor_t *motor);

/* State machine */
void foc_set_state(motor_t *motor, foc_state_t state);
foc_state_t foc_get_state(const motor_t *motor);
void foc_fault(motor_t *motor);                 /* force ERROR */
void foc_clear_fault(motor_t *motor);           /* ERROR -> IDLE */

/* Trigger an extra calibration. with_align != 0 also re-aligns the encoder
 * zero (the motor will twitch to phase A). */
void foc_start_calibration(motor_t *motor, uint8_t with_align);

/* Start electrical identification (Rs/Ld/Lq/flux). Requires current-offset
 * calibration. Runs autonomously in the control ISR and returns to IDLE when
 * done; results land in motor->ident. */
void foc_start_ident(motor_t *motor);

/* Start mechanical identification (J/B/Tc). Spins the rotor with +/- iq_test in
 * current mode; kt = 1.5*p*flux is the torque constant from the flux ident.
 * Requires a calibrated, aligned encoder. Results land in motor->imech. */
void foc_start_identmech(motor_t *motor, float kt, float iq_test);

/* Setpoints (take effect in the matching state) */
void foc_set_voltage(motor_t *motor, float vd, float vq);
void foc_set_openloop_speed(motor_t *motor, float rpm);
void foc_set_target_current(motor_t *motor, float id, float iq);
void foc_set_target_speed(motor_t *motor, float rpm);
void foc_set_target_position(motor_t *motor, float pos_rad);

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

/* Called when the encoder DMA read completes (processes the raw angle). */
void foc_encoder_callback(motor_t *motor);

#endif /* FOC_CORE_H */
