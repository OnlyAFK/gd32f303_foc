#include "interrupt.h"
#include "pwm_curr.h"

static motor_t *g_motor = 0;

// ADC inserted end-of-conversion (PWM rate). The control loop is a state
// machine; the power-up current-offset calibration is its initial state, so
// this just dispatches every cycle. The encoder angle is processed inside the
// loop (top of each tick) and the next DMA read is kicked at the end of it, so
// no encoder state is touched from a second ISR context.
static void foc_adc_isr(void) {
  if (g_motor != 0) {
    foc_control_loop(g_motor);
  }
}

void interrupt_set_active_motor(motor_t *motor) {
  g_motor = motor;
  pwm_curr_attach_callback(foc_adc_isr);
}
