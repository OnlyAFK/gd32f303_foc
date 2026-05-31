#include "interrupt.h"
#include "pwm_curr.h"
#include "tle5012b.h"

static motor_t *g_motor = 0;

// ADC inserted end-of-conversion (PWM rate). The control loop is a state
// machine; the power-up current-offset calibration is its initial state, so
// this just dispatches every cycle.
static void foc_adc_isr(void) {
  if (g_motor != 0) {
    foc_control_loop(g_motor);
  }
}

// Encoder SPI/DMA transfer complete: convert the raw angle to mech/elec angle.
static void foc_encoder_isr(void) {
  if (g_motor != 0) {
    foc_encoder_callback(g_motor);
  }
}

void interrupt_set_active_motor(motor_t *motor) {
  g_motor = motor;
  pwm_curr_attach_callback(foc_adc_isr);
  tle5012b_attach_callback(foc_encoder_isr);
}
