#include "main.h"
#include "uart.h"
#include "can.h"
#include "foc_core.h"
#include "foc_math.h"
#include "fault.h"
#include "params.h"
#include "shell_port.h"
#include "can_proto.h"
#include "interrupt.h"

static motor_t g_motor;

/* ===================================================================
 * Auto-calibration orchestrator (the `autocal` shell command).
 *
 * Runs the full commissioning sequence back-to-back, each stage being a foc
 * state machine that returns to IDLE when done. A settle delay between stages
 * lets residual current/motion decay so one measurement does not corrupt the
 * next (e.g. the flux spin must fully stop before the J/B accel test). Results
 * are captured by the edge logic in the main loop; gains are deployed here.
 * =================================================================== */
#define AUTOCAL_SETTLE_MS   1000u    /* inter-stage settle (cross-talk guard) */
#define AUTOCAL_IMECH_IQ    1.0f     /* J/B test current */
#define AUTOCAL_CUR_BW_HZ   1000.0f  /* current-loop bandwidth */
#define AUTOCAL_SPD_BW_HZ   10.0f    /* speed-loop bandwidth */
#define AUTOCAL_LTU_CURRENT 3.0f     /* LTU open-loop drag current (sizes the
                                        drag voltage from the identified Rs) */
#define AUTOCAL_LTU_VOLTS   2.0f     /* drag voltage fallback if Rs unknown */

typedef enum {
  AC_OFF = 0,
  AC_BEGIN,
  AC_ALIGN_WAIT,
  AC_ALIGN_SETTLE,
  AC_IDENT_WAIT,
  AC_IDENT_SETTLE,
  AC_LTU_WAIT,
  AC_LTU_SETTLE,
  AC_IMECH_WAIT,
  AC_DONE,
} ac_phase_t;

static ac_phase_t s_ac_phase = AC_OFF;
static uint32_t   s_ac_delay;

void autocal_start(void) { s_ac_phase = AC_BEGIN; s_ac_delay = 0u; }
void autocal_abort(void) { s_ac_phase = AC_OFF; }
bool autocal_active(void) { return s_ac_phase != AC_OFF; }

/* Advance the sequence; call once per ~1 ms main-loop tick. */
static void autocal_poll(motor_t *m) {
  if (s_ac_phase == AC_OFF) {
    return;
  }
  foc_state_t st = foc_get_state(m);
  if (st == FOC_STATE_ERROR) {     /* a fault aborts the whole sequence */
    s_ac_phase = AC_OFF;
    return;
  }

  switch (s_ac_phase) {
  case AC_BEGIN:
    foc_start_calibration(m, 1);   /* current-offset cal + encoder alignment */
    s_ac_phase = AC_ALIGN_WAIT;
    break;

  case AC_ALIGN_WAIT:
    if (st == FOC_STATE_IDLE) {
      if (!m->sensor.cur_offset.is_calibrated) { s_ac_phase = AC_OFF; break; }
      s_ac_delay = AUTOCAL_SETTLE_MS;
      s_ac_phase = AC_ALIGN_SETTLE;
    }
    break;

  case AC_ALIGN_SETTLE:
    if (s_ac_delay == 0u) {
      foc_start_ident(m);          /* electrical: Rs/Ld/Lq/flux */
      s_ac_phase = AC_IDENT_WAIT;
    } else {
      s_ac_delay--;
    }
    break;

  case AC_IDENT_WAIT:
    if (st == FOC_STATE_IDLE) {
      if (params_get()->flux <= 0.0f) { s_ac_phase = AC_OFF; break; }
      params_tune_current_loop(m, AUTOCAL_CUR_BW_HZ);  /* deploy current gains */
      s_ac_delay = AUTOCAL_SETTLE_MS;
      s_ac_phase = AC_IDENT_SETTLE;
    }
    break;

  case AC_IDENT_SETTLE:
    if (s_ac_delay == 0u) {
      /* Encoder linearization: one open-loop rev each way (~56 s). Size the
       * drag voltage from the just-identified Rs (V = I*R*3/2, same sizing as
       * the reference firmware) so the drag current sits near 3 A. */
      float rs = params_get()->rs;
      float v = (rs > 0.0f) ? AUTOCAL_LTU_CURRENT * rs * 1.5f : AUTOCAL_LTU_VOLTS;
      foc_start_ltu(m, v);
      s_ac_phase = AC_LTU_WAIT;
    } else {
      s_ac_delay--;
    }
    break;

  case AC_LTU_WAIT:
    if (st == FOC_STATE_IDLE) {
      if (!foc_ltu_is_valid(m)) { s_ac_phase = AC_OFF; break; }  /* sweep failed */
      s_ac_delay = AUTOCAL_SETTLE_MS;
      s_ac_phase = AC_LTU_SETTLE;
    }
    break;

  case AC_LTU_SETTLE:
    if (s_ac_delay == 0u) {
      float kt = 1.5f * (float)HW_MOTOR_POLE_PAIRS * (params_get()->flux * 1.0e-3f);
      foc_start_identmech(m, kt, AUTOCAL_IMECH_IQ);    /* mechanical: J/B/Tc */
      s_ac_phase = AC_IMECH_WAIT;
    } else {
      s_ac_delay--;
    }
    break;

  case AC_IMECH_WAIT:
    if (st == FOC_STATE_IDLE) {
      params_tune_speed_loop(m, AUTOCAL_SPD_BW_HZ);    /* deploy speed gains */
      s_ac_phase = AC_DONE;
    }
    break;

  case AC_DONE:
  default:
    s_ac_phase = AC_OFF;           /* params auto-save on the next IDLE poll */
    break;
  }
}

int main(void) {
    uart_init();
    can_bus_init();

    // Record the reset cause (latches a watchdog fault if we hung last time)
    // before anything else touches the fault state.
    fault_init();

    // foc_init() seeds the motor with compile-time defaults and kicks the
    // power-up current-offset calibration. params_init() then mounts the flash
    // KVDB and overrides those defaults (PID gains, limits, stored encoder
    // alignment) before the control ISR goes live.
    foc_init(&g_motor);
    params_init(&g_motor);
    // CAN protocol layer: binds the motor + reads the node address from params
    // (must follow params_init). Translates CAN frames <-> the foc setters.
    can_proto_init(&g_motor);
    interrupt_set_active_motor(&g_motor);

    // Power-up current-offset calibration (~100 ms). foc_init() kicked it with
    // NO encoder align, so the rotor stays still. Exit on ERROR too so a fault
    // during calibration cannot hang the boot before the console comes up
    // (otherwise it is impossible to diagnose). Offsets are re-measured every
    // boot (never persisted).
    while (foc_get_state(&g_motor) != FOC_STATE_IDLE &&
           foc_get_state(&g_motor) != FOC_STATE_ERROR) {}

    // Encoder zero-alignment is intentionally NOT run automatically on boot:
    // it twitches the rotor. After a flash erase (cali_valid == 0) the stored
    // alignment is gone and closed-loop modes will not behave until the user
    // manually triggers calibration with the `autocal` shell command. This
    // avoids an unexpected jolt on every fresh power-up (the align stage of
    // autocal persists the result via the CALI->IDLE edge in the loop below).

    // Offset calibration done — start the watchdog now (it would have tripped
    // during the blocking current-offset calibration above).
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
        // boot alignment and the align stage of `autocal`).
        foc_state_t cur_state = foc_get_state(&g_motor);
        if (prev_state == FOC_STATE_CALI && cur_state == FOC_STATE_IDLE &&
            g_motor.cali.with_align) {
            params_capture_calibration(&g_motor);
        }
        // Persist the identified motor parameters the moment an ident run finishes.
        if (prev_state == FOC_STATE_IDENT && cur_state == FOC_STATE_IDLE) {
            params_capture_ident(&g_motor);
        }
        // Persist the mechanical parameters when a J/B/Tc run finishes.
        if (prev_state == FOC_STATE_IMECH && cur_state == FOC_STATE_IDLE) {
            params_capture_identmech(&g_motor);
        }
        // Finalise + persist the encoder linearization table when a sweep
        // finishes (FIR/decimate math, kept out of the ISR). Only on a NATURAL
        // completion (phase == LTU_DONE): an aborted sweep is discarded.
        if (prev_state == FOC_STATE_LTU && cur_state == FOC_STATE_IDLE &&
            g_motor.ltu.phase == LTU_DONE) {
            if (foc_ltu_finish(&g_motor)) {
                params_capture_ltu(&g_motor);
            }
        }
        prev_state = cur_state;

        // Advance the auto-calibration sequence (no-op unless `autocal` running).
        // Runs after the capture edges above so deployed gains see fresh results.
        autocal_poll(&g_motor);

        // Flush any pending parameter change to flash — only fires in IDLE so a
        // flash erase never stalls an active control loop.
        params_poll_save(&g_motor);

        // L3 background protection monitor (~1 ms cadence). Current-based checks
        // need valid offsets, same as the fast path.
        uint8_t driving = (foc_get_state(&g_motor) != FOC_STATE_IDLE &&
                           foc_get_state(&g_motor) != FOC_STATE_ERROR &&
                           foc_get_state(&g_motor) != FOC_STATE_CALI &&
                           foc_get_state(&g_motor) != FOC_STATE_IDENT &&
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

        // Service the CAN protocol layer: dispatch received commands, run the
        // command watchdog, emit periodic telemetry. Runs at the 1 ms tick.
        can_proto_poll();

        fault_watchdog_feed();

        // Re-assert the held position target each ms so POSITION mode holds.
        foc_state_t pos_st = foc_get_state(&g_motor);
        if (pos_st != FOC_STATE_ERROR) {
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
