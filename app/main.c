#include "main.h"
#include "uart.h"
#include "foc_core.h"
#include "foc_math.h"
#include "interrupt.h"
#include "vofa.h"

static motor_t g_motor;

int main(void) {
    uart_init();

    foc_init(&g_motor);
    interrupt_set_active_motor(&g_motor);

    // Step 1: power-up current-offset calibration (~100 ms).
    while (foc_get_state(&g_motor) != FOC_STATE_IDLE) {}

    // Step 2: encoder alignment + direction auto-detect (~1200 ms).
    // The rotor will twitch to phase A, then step to +90 deg.
    foc_start_calibration(&g_motor, 1);
    while (foc_get_state(&g_motor) != FOC_STATE_IDLE) {}

    // Step 3: position loop.
    foc_set_state(&g_motor, FOC_STATE_POSITION);


    while (1) {
        vofa_send_justfloat(vofa_data, VOFA_CH_SPEED_TARGET + 1);
        delay_ms(1);
        foc_set_target_position(&g_motor, g_motor.target.pos);
    }
}

void Error_Handler(void) {
    __disable_irq();
    timer_primary_output_config(TIMER0, DISABLE);
    while (1) {}
}

void delay_ms(const uint16_t ms) {
    volatile uint32_t i = ms * 17200U;
    while (i-- > 0U) { __NOP(); }
}
