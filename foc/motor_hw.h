#ifndef MOTOR_HW_H
#define MOTOR_HW_H

#include <stdint.h>
#include <stdbool.h>
#include "foc_math.h"
#include "pwm_curr.h"

// Hardware abstraction for the GD32F303 + DRV8323 + TLE5012B power stage.
// PWM/ADC live in bsp/pwm_curr, the gate driver in driver/drv8323, the encoder
// in driver/tle5012b; this layer adapts them to the FOC core.

/* PWM: timer period (center-aligned ARR) comes from pwm_curr. */
#define HW_PWM_PERIOD         HALF_PWM_PERIOD_CYCLES
#define HW_PWM_FREQ_HZ        PWM_FREQUENCY

/* Current sensing — must match the DRV8323 CSA gain set in drv8323.c. */
#define HW_ADC_RESOLUTION     4096.0f
#define HW_ADC_VREF           3.3f
#define HW_OPAMP_GAIN         20.0f
#define HW_SHUNT_RESISTANCE   0.005f
#define HW_CURRENT_SCALE      (HW_ADC_VREF / (HW_OPAMP_GAIN * HW_SHUNT_RESISTANCE * HW_ADC_RESOLUTION))

/* Encoder: TLE5012B reports a 15-bit angle (0..32767 per mechanical turn). */
#define HW_ENCODER_CPR        32768u

// MOTOR-SPECIFIC — must be set/calibrated for the actual joint motor:
//   HW_MOTOR_POLE_PAIRS   : number of pole pairs of the motor
//   HW_ENCODER_ZERO_OFFSET: mechanical angle (rad) at which elec angle = 0,
//                           found by aligning the rotor to phase A.
#define HW_MOTOR_POLE_PAIRS   14u
#define HW_ENCODER_ZERO_OFFSET 0.0f
// Encoder counting direction relative to positive torque. If closed-loop locks
// up / draws current without spinning, flip this to -1 and re-run calibration.
#define HW_ENCODER_DIR        1   // auto-detected during alignment calibration

/* Control timing */
#define HW_CONTROL_PERIOD_S   0.00005f
#define HW_SPEED_LOOP_DIV     20u
#define HW_POS_LOOP_DIV       20u

typedef struct {
  float iu;
  float iv;
  float iw;
} phase_currents_t;

typedef struct {
  uint16_t raw_angle;
  float mech_angle;
  float elec_angle;
  int8_t direction;
  float zero_offset;   // mechanical angle (rad) where elec_angle = 0; set by alignment
} encoder_data_t;

typedef struct {
  float offset_u;
  float offset_v;
  float offset_w;
  uint8_t is_calibrated;
} current_offset_t;

// Chip-neutral view of the gate-driver fault state. Decoded from whatever
// driver this HAL is built on (currently DRV8323); the FOC/fault layers consume
// this instead of the driver's own register layout so swapping the gate driver
// does not ripple upward.
typedef struct {
  bool overcurrent;        // VDS or sense overcurrent
  bool gate_fault;         // gate-drive fault (stuck gate / IDRIVE too low)
  bool undervoltage;       // supply undervoltage lockout
  bool charge_pump_uv;     // charge-pump undervoltage
  bool overtemp_shutdown;  // thermal shutdown
  bool overtemp_warning;   // thermal warning
  uint16_t raw1;           // raw status register 1 (diagnostics)
  uint16_t raw2;           // raw status register 2 (diagnostics)
} motor_hw_driver_fault_t;

void motor_hw_init(void);

void motor_hw_enable_driver(void);

void motor_hw_disable_driver(void);

// Bus voltage (V) via the monitoring ADC. The single entry point so upper
// layers do not reach into bsp directly.
float motor_hw_get_vbus(void);

// True while the gate driver is signalling a fault on its nFAULT line.
bool motor_hw_driver_fault_pin(void);

// Returns (and clears) whether an nFAULT edge has occurred since the last call.
bool motor_hw_driver_fault_edge(void);

// Decode the gate-driver status registers into the chip-neutral struct.
void motor_hw_driver_get_faults(motor_hw_driver_fault_t *f);

// Re-apply the driver configuration and clear its latched faults (used during
// fault recovery; the driver resets its registers on a supply UVLO/sleep).
void motor_hw_driver_reset(void);

// Basic encoder health check (bus alive). False if the encoder link looks dead.
bool motor_hw_encoder_ok(void);

// Live SSC command-phase echo (~0x8020 on a healthy bus, 0x0000/0xFFFF on a
// dead one) — the raw sample behind motor_hw_encoder_ok(), for diagnostics.
uint16_t motor_hw_encoder_cmd_echo(void);

// True when the init-time sensor config read-back confirmed (or corrected to)
// fast filter update + prediction + autocalibration mode 1. False means the
// fitted derivate could not be configured as assumed — the loop runs but with
// degraded angle quality. Surfaced in the shell `status` output.
bool motor_hw_encoder_config_ok(void);

// Raw encoder MOD_1 / MOD_2 config register read-back (post-correction).
void motor_hw_encoder_get_config(uint16_t *mod1, uint16_t *mod2);

void motor_hw_set_pwm(uint32_t ccr_u, uint32_t ccr_v, uint32_t ccr_w);

void motor_hw_set_pwm_brake(void);

void motor_hw_get_current_adc(uint32_t *adc_u, uint32_t *adc_v, uint32_t *adc_w);

float motor_hw_adc_to_current(uint32_t adc_value, float offset);

void motor_hw_get_currents(phase_currents_t *currents, const current_offset_t *offset);

void motor_hw_start_encoder_read(void);

/* Convert the raw encoder word (from the DMA read kicked at the end of the
 * previous tick) into mech/elec angle. raw_corr is the linearization (LTU)
 * correction in counts, subtracted from the raw count before scaling; pass 0
 * when no correction applies. */
void motor_hw_process_encoder_data(encoder_data_t *encoder, int32_t raw_corr);

void motor_hw_debug_pin(uint8_t state);

#endif /* MOTOR_HW_H */
