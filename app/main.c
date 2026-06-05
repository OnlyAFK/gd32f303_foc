#include "main.h"
#include "uart.h"
#include "foc_core.h"
#include "foc_math.h"
#include "fault.h"
#include "params.h"
#include "shell_port.h"
#include "interrupt.h"

static motor_t g_motor;

int main(void) {
    uart_init();

    // Record the reset cause (latches a watchdog fault if we hung last time)
    // before anything else touches the fault state.
    fault_init();

    // foc_init() seeds the motor with compile-time defaults and kicks the
    // power-up current-offset calibration. params_init() then mounts the flash
    // KVDB and overrides those defaults (PID gains, limits, stored encoder
    // alignment) before the control ISR goes live.
    foc_init(&g_motor);
    params_init(&g_motor);
    interrupt_set_active_motor(&g_motor);

    // Step 1: power-up current-offset calibration (~100 ms). Exit on ERROR too
    // so a fault during calibration cannot hang the boot before the console
    // comes up (otherwise it is impossible to diagnose). Offsets are re-measured
    // every boot (never persisted).
    while (foc_get_state(&g_motor) != FOC_STATE_IDLE &&
           foc_get_state(&g_motor) != FOC_STATE_ERROR) {}

    // Step 2: encoder alignment + direction auto-detect (~1200 ms), only if the
    // offset calibration succeeded AND there is no stored alignment yet. With a
    // persisted calibration the rotor no longer twitches on every boot.
    if (foc_get_state(&g_motor) == FOC_STATE_IDLE &&
        params_get()->cali_valid == 0u) {
        foc_start_calibration(&g_motor, 1);
        while (foc_get_state(&g_motor) != FOC_STATE_IDLE &&
               foc_get_state(&g_motor) != FOC_STATE_ERROR) {}
        // Persist the fresh alignment (deferred flush happens in the loop below
        // once we are settled in IDLE).
        if (foc_get_state(&g_motor) == FOC_STATE_IDLE) {
            params_capture_calibration(&g_motor);
        }
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
    foc_state_t prev_state = foc_get_state(&g_motor);
    while (1) {
        delay_ms(1);
        tick_ms++;

        // Auto-persist a calibration the moment an alignment run finishes (covers
        // a shell-triggered `align`/`cali`; the boot alignment is captured above).
        foc_state_t cur_state = foc_get_state(&g_motor);
        if (prev_state == FOC_STATE_CALI && cur_state == FOC_STATE_IDLE &&
            g_motor.cali.with_align) {
            params_capture_calibration(&g_motor);
        }
        prev_state = cur_state;

        // Flush any pending parameter change to flash — only fires in IDLE so a
        // flash erase never stalls an active control loop.
        params_poll_save(&g_motor);

        // L3 background protection monitor (~1 ms cadence). Current-based checks
        // need valid offsets, same as the fast path.
        uint8_t driving = (foc_get_state(&g_motor) != FOC_STATE_IDLE &&
                           foc_get_state(&g_motor) != FOC_STATE_ERROR &&
                           g_motor.sensor.cur_offset.is_calibrated);
        fault_monitor_slow(motor_hw_get_vbus(),
                           g_motor.actual.iq, g_motor.target.iq, g_motor.actual.rpm,
                           g_motor.sensor.currents.iu, g_motor.sensor.currents.iv,
                           driving != 0u, tick_ms);

        // If a protection fired, latch the control state machine into ERROR and
        // persist the fault snapshot (outputs are now off, so the flash write is
        // safe). Fires once per fault event: ERROR blocks re-entry until cleared.
        if (fault_any_active() && foc_get_state(&g_motor) != FOC_STATE_ERROR) {
            foc_fault(&g_motor);
            params_fault_log_push(fault_get());
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
