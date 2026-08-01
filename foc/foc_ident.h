#ifndef FOC_IDENT_H
#define FOC_IDENT_H

#include <stdint.h>

/* Motor parameter identification — commissioning-only sub-state-machines that
 * run inside the FOC control ISR (states FOC_STATE_IDENT / FOC_STATE_IMECH) and
 * return to IDLE when finished. Conceptually separate from the steady-state
 * cascaded loops, so they live here rather than in foc_core.
 *
 * foc_core.h includes this header to compose the ctx structs into motor_t; the
 * run/query hooks below are the only seam back into foc_control_loop. */

/* Guarded so foc_ident.h and the other sub-state-machine headers can both
 * forward-declare motor_t when foc_core.h includes them (a bare duplicate
 * typedef is a C99 constraint violation under AC5). */
#ifndef FOC_MOTOR_T_DEFINED
#define FOC_MOTOR_T_DEFINED
typedef struct motor_s motor_t;   /* full definition lives in foc_core.h */
#endif

/* Electrical-identification sub-phases (only meaningful in FOC_STATE_IDENT).
 * Built up in stages: Rs, then Ld/Lq, then flux linkage. Later stages reuse the
 * result of the earlier ones (the L bias voltage uses Rs; flux reuses the
 * current loop). Phases up to IDENT_LQ_INJ_HIGH lock the rotor at theta_e = 0;
 * the flux phases spin it on the measured electrical angle.
 * Keep the order contiguous — the ident phase helpers range-test it. */
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

/* Start electrical identification (Rs/Ld/Lq/flux). Requires current-offset
 * calibration. Runs autonomously in the control ISR and returns to IDLE when
 * done; results land in motor->ident. */
void foc_start_ident(motor_t *motor);

/* Start mechanical identification (J/B/Tc). Spins the rotor with +/- iq_test in
 * current mode; kt = 1.5*p*flux is the torque constant from the flux ident.
 * Requires a calibrated, aligned encoder. Results land in motor->imech. */
void foc_start_identmech(motor_t *motor, float kt, float iq_test);

/* True while electrical ident holds the rotor at theta_e = 0 (Rs/Ld/Lq phases);
 * foc_control_loop forces the control angle to 0 in that case. */
uint8_t foc_ident_locks_rotor(const motor_t *motor);

/* Per-tick hooks called from foc_control_loop's IDENT / IMECH switch cases.
 * Each advances its state machine and writes the dq voltage to apply this cycle
 * (leaving the outputs untouched in the terminal/handover phases). */
void foc_ident_run(motor_t *motor, float *vd_out, float *vq_out);
void foc_imech_run(motor_t *motor, float *vd_out, float *vq_out);

#endif /* FOC_IDENT_H */
