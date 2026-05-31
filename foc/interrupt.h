#ifndef INTERRUPT_H
#define INTERRUPT_H

#include "foc_core.h"

// Bind the motor instance to the interrupt handlers and attach the ADC
// inserted-conversion callback (FOC tick) and the encoder DMA-complete callback
// (angle processing). Call once after foc_init().
void interrupt_set_active_motor(motor_t *motor);

#endif /* INTERRUPT_H */
