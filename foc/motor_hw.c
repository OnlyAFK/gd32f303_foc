#include "motor_hw.h"
#include "drv8323.h"
#include "tle5012b.h"
#include "pwm_curr.h"
#include "fault.h"
#include <math.h>

#define CALIBRATION_SAMPLES 1000u
#define RAW_TO_RAD          (FOC_2PI / (float)HW_ENCODER_CPR)

// Hardware emergency stop, invoked from the DRV8323 nFAULT EXTI ISR. Must be
// short and must not touch SPI: just kill the gate-drive PWM outputs.
static void motor_hw_emergency_stop(void) {
  pwm_disable();
}

void motor_hw_init(void) {
  // Bring up the whole power/feedback stage. drv8323_init enables the gate
  // driver and sets 3x PWM mode; pwm_curr_init starts TIMER0 + the ADC trigger
  // (PWM channel outputs stay disabled until motor_hw_enable_driver).
  // SPI read-back verifies the link to the DRV8323; a mismatch latches a comm
  // fault (fault_init() has already run by this point).
  if (!drv8323_init()) {
    fault_raise(FAULT_DRV_COMM, 0.0f, 0.0f);
  }
  tle5012b_init();
  pwm_curr_init();

  // Now that the PWM timer exists, arm the nFAULT hardware emergency stop.
  drv8323_attach_emergency_cb(motor_hw_emergency_stop);
  drv8323_fault_exti_init();
}

void motor_hw_enable_driver(void) {
  pwm_enable();
}

void motor_hw_disable_driver(void) {
  pwm_disable();
}

float motor_hw_get_vbus(void) {
  return read_vbus();
}

bool motor_hw_driver_fault_pin(void) {
  return drv8323_fault_active();
}

bool motor_hw_driver_fault_edge(void) {
  return drv8323_nfault_edge_pending();
}

void motor_hw_driver_get_faults(motor_hw_driver_fault_t *f) {
  drv8323_faults_t d;
  drv8323_parse_faults(&d);
  f->overcurrent       = d.vds_ocp || d.sen_ocp;
  f->gate_fault        = d.gdf;
  f->undervoltage      = d.uvlo;
  f->charge_pump_uv    = d.cpuv;
  f->overtemp_shutdown = d.otsd;
  f->overtemp_warning  = d.otw;
  f->raw1 = d.fs1_raw;
  f->raw2 = d.fs2_raw;
}

void motor_hw_driver_reset(void) {
  drv8323_apply_config();
  drv8323_clear_fault();
}

bool motor_hw_encoder_ok(void) {
  // A healthy SSC bus echoes ~0x8020 on the command word; all-ones / all-zeros
  // means SCK/MOSI/MISO are not reaching the sensor.
  uint16_t echo = tle5012b_debug_cmd_echo();
  return (echo != 0x0000u) && (echo != 0xFFFFu);
}

// Phase mapping for this board (verified on the dgm reference):
//   phase A = u -> CH2 (PA10/INHA), current IA (PA3)
//   phase B = v -> CH1 (PA9/INHB),  current IB (PA2)
//   phase C = w -> CH0 (PA8/INHC),  current IC (PA1)
void motor_hw_set_pwm(uint32_t ccr_u, uint32_t ccr_v, uint32_t ccr_w) {
  pwm_set_duty((uint16_t)ccr_w, (uint16_t)ccr_v, (uint16_t)ccr_u);
}

void motor_hw_set_pwm_brake(void) {
  uint16_t mid = (uint16_t)(HW_PWM_PERIOD / 2u);
  pwm_set_duty(mid, mid, mid);
}

void motor_hw_get_current_adc(uint32_t *adc_u, uint32_t *adc_v, uint32_t *adc_w) {
  *adc_u = READ_IA_ADC();  // phase A
  *adc_v = READ_IB_ADC();  // phase B
  *adc_w = READ_IC_ADC();  // phase C
}

float motor_hw_adc_to_current(uint32_t adc_value, float offset) {
  return ((float)adc_value - offset) * HW_CURRENT_SCALE;
}

void motor_hw_get_currents(phase_currents_t *currents, const current_offset_t *offset) {
  uint32_t adc_u, adc_v, adc_w;
  motor_hw_get_current_adc(&adc_u, &adc_v, &adc_w);

  currents->iu = motor_hw_adc_to_current(adc_u, offset->offset_u);
  currents->iv = motor_hw_adc_to_current(adc_v, offset->offset_v);
  /* Iw from Kirchhoff's law: Iu + Iv + Iw = 0 */
  currents->iw = -currents->iu - currents->iv;
}

void motor_hw_start_encoder_read(void) {
  tle5012b_start_read();
}

static inline float normalize_angle(float angle) {
  while (angle >= FOC_2PI) angle -= FOC_2PI;
  while (angle < 0.0f)    angle += FOC_2PI;
  return angle;
}

void motor_hw_process_encoder_data(encoder_data_t *encoder) {
  encoder->raw_angle = tle5012b_get_raw() & (uint16_t)(HW_ENCODER_CPR - 1u);
  encoder->mech_angle = encoder->direction * encoder->raw_angle * RAW_TO_RAD;
  encoder->mech_angle = normalize_angle(encoder->mech_angle);

  /* Electrical angle: θe = (θm - offset) * Pp  (offset from alignment) */
  encoder->elec_angle = (encoder->mech_angle - encoder->zero_offset) * HW_MOTOR_POLE_PAIRS;
  encoder->elec_angle = normalize_angle(encoder->elec_angle);
}

uint8_t motor_hw_calibrate_current_offset(current_offset_t *offset,
                                          uint32_t adc_u, uint32_t adc_v, uint32_t adc_w) {
  static uint32_t sum_u = 0, sum_v = 0, sum_w = 0;
  static uint32_t sample_count = 0;

  if (offset->is_calibrated) {
    return 1;
  }

  if (sample_count < CALIBRATION_SAMPLES) {
    sum_u += adc_u;
    sum_v += adc_v;
    sum_w += adc_w;
    sample_count++;
    return 0;
  }

  offset->offset_u = (float)sum_u / (float)CALIBRATION_SAMPLES;
  offset->offset_v = (float)sum_v / (float)CALIBRATION_SAMPLES;
  offset->offset_w = (float)sum_w / (float)CALIBRATION_SAMPLES;
  offset->is_calibrated = 1;

  sum_u = sum_v = sum_w = 0;
  sample_count = 0;

  return 1;
}

void motor_hw_debug_pin(uint8_t state) {
  // No dedicated debug pin on this board; left as a no-op. Wire a spare GPIO
  // here (e.g. via gpio_bit_set/reset) to scope the control-loop timing.
  (void)state;
}
