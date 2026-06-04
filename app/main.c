#include "main.h"
#include "uart.h"
#include "foc_core.h"
#include "foc_math.h"
#include "fault.h"
#include "shell_port.h"
#include "interrupt.h"

static motor_t g_motor;

int main(void) {
    uart_init();

    // Record the reset cause (latches a watchdog fault if we hung last time)
    // before anything else touches the fault state.
    fault_init();

    foc_init(&g_motor);
    interrupt_set_active_motor(&g_motor);

    // Step 1: power-up current-offset calibration (~100 ms). Exit on ERROR too
    // so a fault during calibration cannot hang the boot before the console
    // comes up (otherwise it is impossible to diagnose).
    while (foc_get_state(&g_motor) != FOC_STATE_IDLE &&
           foc_get_state(&g_motor) != FOC_STATE_ERROR) {}

    // Step 2: encoder alignment + direction auto-detect (~1200 ms), only if the
    // offset calibration succeeded. The rotor twitches to phase A, then +90 deg.
    if (foc_get_state(&g_motor) == FOC_STATE_IDLE) {
        foc_start_calibration(&g_motor, 1);
        while (foc_get_state(&g_motor) != FOC_STATE_IDLE &&
               foc_get_state(&g_motor) != FOC_STATE_ERROR) {}
    }

    // Calibration done — start the watchdog now (it would have tripped during
    // the >1 s blocking calibration above).
    fault_watchdog_start();

    // Calibration leaves the motor in IDLE (or ERROR on failure). From here the
    // shell drives everything: use `mode`, `speed`, `pos`, etc. to run the motor.

    // Bring up the letter-shell console on the UART transport. VOFA waveform
    // streaming is no longer free-running; use the shell `scope` command to
    // stream it on demand (shell and VOFA share the UART, so they are mutually
    // exclusive — see CHANGELOG/ARCHITECTURE).
    shell_port_init(uart_stream(), &g_motor);

    uint32_t tick_ms = 0u;
    while (1) {
        delay_ms(1);
        tick_ms++;

        // L3 background protection monitor (~1 ms cadence). Current-based checks
        // need valid offsets, same as the fast path.
        uint8_t driving = (foc_get_state(&g_motor) != FOC_STATE_IDLE &&
                           foc_get_state(&g_motor) != FOC_STATE_ERROR &&
                           g_motor.sensor.cur_offset.is_calibrated);
        fault_monitor_slow(motor_hw_get_vbus(),
                           g_motor.actual.iq, g_motor.target.iq, g_motor.actual.rpm,
                           g_motor.sensor.currents.iu, g_motor.sensor.currents.iv,
                           driving != 0u, tick_ms);

        // If a protection fired, latch the control state machine into ERROR.
        if (fault_any_active() && foc_get_state(&g_motor) != FOC_STATE_ERROR) {
            foc_fault(&g_motor);
        }

        // Serve shell commands (or stream VOFA frames in scope mode).
        shell_port_poll();

        fault_watchdog_feed();

        if (foc_get_state(&g_motor) != FOC_STATE_ERROR) {
            foc_set_target_position(&g_motor, g_motor.target.pos);
        }
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
