#include "fault.h"

#include <math.h>
#include "gd32f30x.h"
#include "motor_hw.h"

// See fault.h for the layered protection architecture. This module consumes the
// chip-neutral HAL (motor_hw) only; it never touches the gate-driver or encoder
// drivers directly, so changing those chips does not ripple into the protection
// logic.

static fault_status_t s_status;

// Live, runtime-tunable thresholds (defaults set in fault_init, overridden from
// flash via app/params -> fault_set_cfg). Read from the control ISR, so updates
// are made under a brief IRQ mask.
static fault_cfg_t s_cfg;

// Free-running millisecond timestamp, updated by fault_monitor_slow(). Also
// used by fault_raise() (which may run in ISR context) for the trip timestamp.
static volatile uint32_t s_tick_ms;

static inline float clampf(float v, float lo, float hi) {
  return (v < lo) ? lo : ((v > hi) ? hi : v);
}

// Detector state (owned by the slow monitor / main context).
static uint32_t s_uv_since;        // tick when bus first went under-voltage (0 = ok)
static uint32_t s_ov_since;        // tick when bus first went over-voltage  (0 = ok)
static uint32_t s_stall_since;     // tick when the stall condition began    (0 = ok)
static uint32_t s_phaseloss_since; // tick when the phase-loss condition began(0 = ok)
static uint32_t s_i2t_last_tick;   // previous tick for the I^2 t dt
static float    s_i2t_accum;       // A^2 * s accumulated above continuous

static inline float fmax3(float a, float b, float c) {
  float m = (a > b) ? a : b;
  return (m > c) ? m : c;
}

// Decode the DRV8323 status registers and raise the matching fault bits. Always
// refreshes the cached register snapshot so diagnostics show the latest state.
static void fault_decode_drv(float vbus, float iq) {
  motor_hw_driver_fault_t f;
  motor_hw_driver_get_faults(&f);

  s_status.drv_fs1 = f.raw1;
  s_status.drv_fs2 = f.raw2;

  uint32_t code = 0u;
  // The VDS-vs-sense overcurrent distinction is preserved in the raw status
  // registers (drv_fs1/drv_fs2) for diagnostics.
  if (f.overcurrent)       code |= FAULT_DRV_VDS_OCP;
  if (f.gate_fault)        code |= FAULT_DRV_GDF;
  if (f.undervoltage)      code |= FAULT_DRV_UVLO;
  if (f.charge_pump_uv)    code |= FAULT_DRV_CPUV;
  if (f.overtemp_shutdown) code |= FAULT_DRV_OTSD;
  if (f.overtemp_warning)  code |= FAULT_DRV_OTW;

  if (code != 0u) {
    fault_raise(code, vbus, iq);
  }
}

void fault_init(void) {
  s_status.active = 0u;
  s_status.latched = 0u;
  s_status.first = 0u;
  s_status.drv_fs1 = 0u;
  s_status.drv_fs2 = 0u;
  s_status.vbus_at_fault = 0.0f;
  s_status.iq_at_fault = 0.0f;
  s_status.tick_at_fault = 0u;

  s_tick_ms = 0u;
  s_uv_since = 0u;
  s_ov_since = 0u;
  s_stall_since = 0u;
  s_phaseloss_since = 0u;
  s_i2t_last_tick = 0u;
  s_i2t_accum = 0.0f;

  // Seed the runtime thresholds with the compile-time defaults so protection is
  // valid before app/params loads any stored overrides.
  s_cfg.thr_protect_en = 1u;
  s_cfg.vbus_ov_trip   = FAULT_VBUS_OV_TRIP;
  s_cfg.vbus_uv_trip   = FAULT_VBUS_UV_TRIP;
  s_cfg.oc_sw_peak     = FAULT_OC_SW_PEAK;
  s_cfg.i_cont         = FAULT_I_CONT;
  s_cfg.i2t_budget     = FAULT_I2T_BUDGET;
  s_cfg.stall_iq       = FAULT_STALL_IQ;
  s_cfg.stall_rpm      = FAULT_STALL_RPM;
  s_cfg.overspeed_rpm  = FAULT_OVERSPEED_RPM;

  // If the previous reset came from the independent watchdog, record it so the
  // operator knows the control loop hung.
  if (rcu_flag_get(RCU_FLAG_FWDGTRST) != RESET) {
    s_status.active = FAULT_WATCHDOG;
    s_status.latched = FAULT_WATCHDOG;
  }
  rcu_all_reset_flag_clear();
}

void fault_watchdog_start(void) {
  // FWDGT runs from the 40 kHz IRC40K. Prescaler /32 -> 1250 Hz tick.
  // reload = timeout_ms * 1250 / 1000.
  fwdgt_config((uint16_t)((FAULT_WDG_TIMEOUT_MS * 1250u) / 1000u), FWDGT_PSC_DIV32);
  fwdgt_enable();
}

void fault_set_cfg(const fault_cfg_t *cfg) {
  fault_cfg_t c = *cfg;

  // VESC-style guard: clamp to the silicon-limit bounds unless protection is
  // explicitly disabled.
  if (c.thr_protect_en) {
    c.oc_sw_peak    = clampf(c.oc_sw_peak,    FAULT_OC_SW_HARD_MIN,     FAULT_OC_SW_HARD_MAX);
    c.vbus_ov_trip  = clampf(c.vbus_ov_trip,  FAULT_VBUS_OV_HARD_MIN,   FAULT_VBUS_OV_HARD_MAX);
    c.vbus_uv_trip  = clampf(c.vbus_uv_trip,  FAULT_VBUS_UV_HARD_MIN,   FAULT_VBUS_UV_HARD_MAX);
    c.i_cont        = clampf(c.i_cont,        FAULT_I_CONT_HARD_MIN,    FAULT_I_CONT_HARD_MAX);
    c.i2t_budget    = clampf(c.i2t_budget,    FAULT_I2T_HARD_MIN,       FAULT_I2T_HARD_MAX);
    c.stall_iq      = clampf(c.stall_iq,      FAULT_STALL_IQ_HARD_MIN,  FAULT_STALL_IQ_HARD_MAX);
    c.stall_rpm     = clampf(c.stall_rpm,     FAULT_STALL_RPM_HARD_MIN, FAULT_STALL_RPM_HARD_MAX);
    c.overspeed_rpm = clampf(c.overspeed_rpm, FAULT_OVERSPEED_HARD_MIN, FAULT_OVERSPEED_HARD_MAX);
  }

  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  s_cfg = c;
  __set_PRIMASK(primask);
}

const fault_cfg_t *fault_get_cfg(void) {
  return &s_cfg;
}

void fault_raise(uint32_t code, float vbus, float iq) {
  if (code == 0u) {
    return;
  }
  // IRQ-safe read-modify-write (callable from the ADC/control ISR).
  uint32_t primask = __get_PRIMASK();
  __disable_irq();

  bool was_clear = (s_status.active == 0u);
  s_status.active |= code;
  s_status.latched |= code;
  if (was_clear) {
    s_status.first = code;   // root cause: the fault(s) of this no-fault -> fault edge
    s_status.vbus_at_fault = vbus;
    s_status.iq_at_fault = iq;
    s_status.tick_at_fault = s_tick_ms;
  }

  __set_PRIMASK(primask);
}

bool fault_any_active(void) {
  return s_status.active != 0u;
}

const fault_status_t *fault_get(void) {
  return &s_status;
}

bool fault_check_fast(float iu, float iv, float iw, float vbus, float iq,
                      bool driving) {
  bool trip = false;

  // Software instantaneous overcurrent (per phase). Only meaningful while the
  // outputs are live.
  if (driving) {
    if (fabsf(iu) > s_cfg.oc_sw_peak ||
        fabsf(iv) > s_cfg.oc_sw_peak ||
        fabsf(iw) > s_cfg.oc_sw_peak) {
      fault_raise(FAULT_OVERCURRENT_SW, vbus, iq);
      trip = true;
    }
  }

  // Bus overvoltage must react immediately (regenerative kickback can ramp the
  // rail in well under a millisecond). Checked regardless of drive state.
  if (vbus > s_cfg.vbus_ov_trip) {
    fault_raise(FAULT_BUS_OV, vbus, iq);
    trip = true;
  }

  // Gate-driver hardware fault pin. The specific cause is decoded over SPI by
  // the slow monitor; here we only need to stop driving right away.
  if (driving && motor_hw_driver_fault_pin()) {
    trip = true;
  }

  return trip;
}

void fault_monitor_slow(float vbus, float iq, float iq_cmd, float rpm,
                        float iu, float iv, bool driving, uint32_t tick_ms) {
  s_tick_ms = tick_ms;

  // --- Gate-driver status decode (only when the pin is asserted or an edge is
  //     pending; reading SPI every ms is unnecessary otherwise). ---
  if (motor_hw_driver_fault_edge() || motor_hw_driver_fault_pin()) {
    fault_decode_drv(vbus, iq);
  }

  // --- Bus overvoltage (debounced; the fast path already trips hard) ---
  if (vbus > s_cfg.vbus_ov_trip) {
    if (s_ov_since == 0u) s_ov_since = tick_ms ? tick_ms : 1u;
    if ((tick_ms - s_ov_since) >= FAULT_OV_DEBOUNCE_MS) {
      fault_raise(FAULT_BUS_OV, vbus, iq);
    }
  } else {
    s_ov_since = 0u;
  }

  // --- Bus undervoltage (debounced so stall/inrush sag is ignored) ---
  if (vbus < s_cfg.vbus_uv_trip) {
    if (s_uv_since == 0u) s_uv_since = tick_ms ? tick_ms : 1u;
    if ((tick_ms - s_uv_since) >= FAULT_UV_DEBOUNCE_MS) {
      fault_raise(FAULT_BUS_UV, vbus, iq);
    }
  } else {
    s_uv_since = 0u;
  }

  // --- I^2 t winding overload --------------------------------------------
  float dt = (float)(tick_ms - s_i2t_last_tick) * 0.001f;
  s_i2t_last_tick = tick_ms;
  if (dt > 0.0f && dt < 0.1f) {  // ignore the first call / long gaps
    float iw = -iu - iv;
    float i2 = fmax3(iu * iu, iv * iv, iw * iw);  // worst-phase current^2
    float excess = i2 - (s_cfg.i_cont * s_cfg.i_cont);
    s_i2t_accum += excess * dt;                   // integrate over/under continuous
    if (s_i2t_accum < 0.0f) s_i2t_accum = 0.0f;   // never bank credit
    if (s_i2t_accum > s_cfg.i2t_budget) {
      fault_raise(FAULT_OVERLOAD_I2T, vbus, iq);
    }
  }

  // --- Overspeed (immediate) ---------------------------------------------
  if (fabsf(rpm) > s_cfg.overspeed_rpm) {
    fault_raise(FAULT_OVERSPEED, vbus, iq);
  }

  // --- Stall: torque current high but rotor not turning, sustained -------
  if (driving && fabsf(iq) > s_cfg.stall_iq && fabsf(rpm) < s_cfg.stall_rpm) {
    if (s_stall_since == 0u) s_stall_since = tick_ms ? tick_ms : 1u;
    if ((tick_ms - s_stall_since) >= FAULT_STALL_MS) {
      fault_raise(FAULT_STALL, vbus, iq);
    }
  } else {
    s_stall_since = 0u;
  }

  // --- Phase loss: commanding torque but no current flows, sustained -----
  if (driving && fabsf(iq_cmd) > FAULT_PHASELOSS_IQCMD &&
      fabsf(iu) < FAULT_PHASELOSS_I && fabsf(iv) < FAULT_PHASELOSS_I) {
    if (s_phaseloss_since == 0u) s_phaseloss_since = tick_ms ? tick_ms : 1u;
    if ((tick_ms - s_phaseloss_since) >= FAULT_PHASELOSS_MS) {
      fault_raise(FAULT_PHASE_LOSS, vbus, iq);
    }
  } else {
    s_phaseloss_since = 0u;
  }

  // --- Encoder bus health ------------------------------------------------
  // Reliably catches an encoder bus that is dead from bring-up. NOTE: a
  // *mid-run* disconnect is not caught here yet — add a "frozen angle while
  // commanding motion" check in the HAL if needed.
  if (!motor_hw_encoder_ok()) {
    fault_raise(FAULT_ENCODER, vbus, iq);
  }
}

bool fault_try_clear(float vbus) {
  // 1. DRV8323: re-apply the configuration (registers reset on a UVLO/sleep) and
  //    pulse CLR_FLT. If nFAULT is still low afterwards the condition persists.
  if (motor_hw_driver_fault_pin() || motor_hw_driver_fault_edge()) {
    motor_hw_driver_reset();
    for (volatile uint32_t i = 0u; i < 2000u; i++) {  // brief settle (~tens of us)
      __NOP();
    }
    if (motor_hw_driver_fault_pin()) {
      fault_decode_drv(vbus, 0.0f);  // refresh which driver fault is stuck
      return false;
    }
  }

  // 2. Bus voltage must be back inside the safe window.
  if (vbus > s_cfg.vbus_ov_trip || vbus < s_cfg.vbus_uv_trip) {
    return false;
  }

  // 3. Conditions look clear. Reset detector state so latching software faults
  //    (phase loss / encoder) re-arm and will trip again immediately if the
  //    underlying problem is still present.
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  s_status.active = 0u;
  s_status.first = 0u;
  __set_PRIMASK(primask);

  s_uv_since = 0u;
  s_ov_since = 0u;
  s_stall_since = 0u;
  s_phaseloss_since = 0u;
  s_i2t_accum = 0.0f;

  // Make sure the driver is configured and its latched bits are cleared even if
  // no hardware fault was pending above (e.g. a pure software trip).
  motor_hw_driver_reset();
  return true;
}

void fault_clear_latched(void) {
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  s_status.latched = s_status.active;  // keep what is still active
  __set_PRIMASK(primask);
}

void fault_watchdog_feed(void) {
  fwdgt_counter_reload();
}
