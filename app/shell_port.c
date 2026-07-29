#include "shell_port.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "shell.h"
#include "fault.h"
#include "params.h"
#include "version.h"
#include "vofa.h"
#include "motor_hw.h"
#include "foc_math.h"
#include "main.h"
#include "can.h"
#include "can_proto.h"

// Number of VOFA channels streamed in scope mode (channels 0..SPEED_TARGET).
#define SCOPE_CHANNELS  (VOFA_CH_SPEED_TARGET + 1)

// letter-shell splits this buffer across the edit line + SHELL_HISTORY_MAX_NUMBER
// history slots (shell.c: bufferSize = size / (HISTORY+1)). With 5 history slots
// the usable command length is only SIZE/6 - 1, so keep this generous: 512 -> ~84
// chars per line, enough for the longest command + args.
#define SHELL_BUFFER_SIZE  512u
#define DEG_TO_RAD         (FOC_PI / 180.0f)
#define RAD_TO_DEG         (180.0f / FOC_PI)

static Shell s_shell;
static char s_shell_buffer[SHELL_BUFFER_SIZE];
static const io_stream_t *s_io;
static motor_t *s_motor;
static volatile uint8_t s_scope;

// --- Transport adapters: bridge letter-shell's read/write to the io_stream. ---
static signed short shell_write_adapter(char *data, unsigned short len) {
  return (signed short)s_io->write(data, (uint32_t)len);
}

static signed short shell_read_adapter(char *data, unsigned short len) {
  return (signed short)s_io->read(data, (uint32_t)len);
}

// --- Small helpers ---
static float arg_f(const char *s) {
  return (float)atof(s);
}

// Print "  name = [-]X.XXX" without pulling in %f formatting.
static void print_f(const char *name, float v) {
  int neg = (v < 0.0f);
  float a = neg ? -v : v;
  int whole = (int)a;
  int frac = (int)((a - (float)whole) * 1000.0f + 0.5f);
  if (frac >= 1000) {
    whole += 1;
    frac -= 1000;
  }
  shellPrint(&s_shell, "  %s = %s%d.%03d\r\n", name, neg ? "-" : "", whole, frac);
}

static const struct {
  uint32_t bit;
  const char *name;
} k_fault_names[] = {
  { FAULT_DRV_VDS_OCP,    "DRV_VDS_OCP" },
  { FAULT_DRV_SEN_OCP,    "DRV_SEN_OCP" },
  { FAULT_DRV_GDF,        "DRV_GDF" },
  { FAULT_DRV_UVLO,       "DRV_UVLO" },
  { FAULT_DRV_CPUV,       "DRV_CPUV" },
  { FAULT_DRV_OTSD,       "DRV_OTSD" },
  { FAULT_DRV_OTW,        "DRV_OTW" },
  { FAULT_DRV_COMM,       "DRV_COMM" },
  { FAULT_BUS_OV,         "BUS_OV" },
  { FAULT_BUS_UV,         "BUS_UV" },
  { FAULT_OVERCURRENT_SW, "OC_SW" },
  { FAULT_OVERLOAD_I2T,   "OVERLOAD_I2T" },
  { FAULT_MOTOR_OVERTEMP, "MOTOR_OTEMP" },
  { FAULT_BOARD_OVERTEMP, "BOARD_OTEMP" },
  { FAULT_PHASE_LOSS,     "PHASE_LOSS" },
  { FAULT_STALL,          "STALL" },
  { FAULT_OVERSPEED,      "OVERSPEED" },
  { FAULT_ENCODER,        "ENCODER" },
  { FAULT_WATCHDOG,       "WATCHDOG" },
};

static void print_fault_names(uint32_t mask) {
  if (mask == 0u) {
    shellWriteString(&s_shell, " none");
    return;
  }
  for (uint32_t i = 0u; i < (sizeof(k_fault_names) / sizeof(k_fault_names[0])); i++) {
    if (mask & k_fault_names[i].bit) {
      shellWriteString(&s_shell, " ");
      shellWriteString(&s_shell, k_fault_names[i].name);
    }
  }
}

static const char *state_name(foc_state_t s) {
  switch (s) {
  case FOC_STATE_IDLE:            return "IDLE";
  case FOC_STATE_CALI:            return "CALI";
  case FOC_STATE_OPENLOOP_VOLT:   return "OPENLOOP";
  case FOC_STATE_CLOSEDLOOP_VOLT: return "VOLT";
  case FOC_STATE_CURRENT:         return "CURRENT";
  case FOC_STATE_SPEED:           return "SPEED";
  case FOC_STATE_POSITION:        return "POSITION";
  case FOC_STATE_IDENT:           return "IDENT";
  case FOC_STATE_IMECH:           return "IMECH";
  case FOC_STATE_ACOG:            return "ACOG";
  case FOC_STATE_ERROR:           return "ERROR";
  default:                        return "?";
  }
}

// =================== Commands (all SHELL_TYPE_CMD_MAIN) ===================

static int cmd_mode(int argc, char *argv[]) {
  if (argc < 2) {
    shellPrint(&s_shell, "mode = %s\r\n", state_name(foc_get_state(s_motor)));
    shellWriteString(&s_shell, "usage: mode <idle|current|speed|position|volt|openloop>\r\n");
    return 0;
  }
  const char *m = argv[1];
  // Entering a mode clears that mode's target so the motor does not lurch on
  // a stale setpoint. POSITION holds the present angle instead of jumping to 0.
  if (strcmp(m, "idle") == 0) {
    foc_set_state(s_motor, FOC_STATE_IDLE);
  } else if (strcmp(m, "current") == 0) {
    foc_set_target_current(s_motor, 0.0f, 0.0f);
    foc_set_state(s_motor, FOC_STATE_CURRENT);
  } else if (strcmp(m, "speed") == 0) {
    foc_set_target_speed(s_motor, 0.0f);
    foc_set_state(s_motor, FOC_STATE_SPEED);
  } else if (strcmp(m, "position") == 0) {
    foc_set_state(s_motor, FOC_STATE_POSITION);  // self-holds current position
  } else if (strcmp(m, "volt") == 0) {
    foc_set_voltage(s_motor, 0.0f, 0.0f);
    foc_set_state(s_motor, FOC_STATE_CLOSEDLOOP_VOLT);
  } else if (strcmp(m, "openloop") == 0) {
    foc_set_voltage(s_motor, 0.0f, 0.0f);
    foc_set_openloop_speed(s_motor, 0.0f);
    foc_set_state(s_motor, FOC_STATE_OPENLOOP_VOLT);
  } else {
    shellWriteString(&s_shell, "unknown mode\r\n");
    return 0;
  }
  shellPrint(&s_shell, "mode -> %s\r\n", state_name(foc_get_state(s_motor)));
  return 0;
}

static int cmd_speed(int argc, char *argv[]) {
  if (argc >= 2) {
    foc_set_target_speed(s_motor, arg_f(argv[1]));
  }
  print_f("speed_ref_rpm", s_motor->target.rpm);
  print_f("speed_rpm", s_motor->actual.rpm);
  return 0;
}

static int cmd_id(int argc, char *argv[]) {
  if (argc >= 2) {
    foc_set_target_current(s_motor, arg_f(argv[1]), s_motor->target.iq);
  }
  print_f("id_ref_A", s_motor->target.id);
  print_f("id_A", s_motor->actual.id);
  return 0;
}

static int cmd_iq(int argc, char *argv[]) {
  if (argc >= 2) {
    foc_set_target_current(s_motor, s_motor->target.id, arg_f(argv[1]));
  }
  print_f("iq_ref_A", s_motor->target.iq);
  print_f("iq_A", s_motor->actual.iq);
  return 0;
}

static int cmd_pos(int argc, char *argv[]) {
  if (argc >= 2) {
    foc_set_target_position(s_motor, arg_f(argv[1]) * DEG_TO_RAD);
  }
  print_f("pos_ref_deg", s_motor->target.pos * RAD_TO_DEG);
  print_f("pos_deg", s_motor->actual.pos * RAD_TO_DEG);
  return 0;
}

static int cmd_volt(int argc, char *argv[]) {
  if (argc >= 3) {
    foc_set_voltage(s_motor, arg_f(argv[1]), arg_f(argv[2]));
  }
  print_f("vd", s_motor->target.vd);
  print_f("vq", s_motor->target.vq);
  return 0;
}

static int cmd_autocal(int argc, char *argv[]) {
  (void)argc; (void)argv;
  if (foc_get_state(s_motor) != FOC_STATE_IDLE) {
    shellWriteString(&s_shell, "need IDLE first (use 'stop')\r\n");
    return 0;
  }
  autocal_start();
  shellWriteString(&s_shell, "auto-calibration started (~45 s):\r\n");
  shellWriteString(&s_shell, "  cal+align -> Rs/Ld/Lq/flux -> J/B/Tc, gains auto-deployed\r\n");
  shellWriteString(&s_shell, "  rotor MOVES then SPINS - keep clear! watch 'status'\r\n");
  return 0;
}

// Raw identification measurements (for cross-checking the result). Reads the
// last ident run straight from the motor struct.
static int cmd_identinfo(int argc, char *argv[]) {
  (void)argc; (void)argv;
  const ident_ctx_t *c = &s_motor->ident;
  shellWriteString(&s_shell, "Rs (two-point DC):\r\n");
  print_f("  vd_low_V", c->vd_low);
  print_f("  vd_high_V", c->vd_high);
  print_f("  id_low_A", c->id_low);
  print_f("  id_high_A", c->id_high);
  print_f("  rs_ohm", c->rs);
  shellWriteString(&s_shell, "Ld/Lq (square-wave injection):\r\n");
  print_f("  vd_bias_V", c->vd_bias);
  print_f("  ld_pp_lo_A", c->ld_pp_low);
  print_f("  ld_pp_hi_A", c->ld_pp_high);
  print_f("  ld_uH", c->ld);
  print_f("  lq_pp_lo_A", c->lq_pp_low);
  print_f("  lq_pp_hi_A", c->lq_pp_high);
  print_f("  lq_uH", c->lq);
  shellWriteString(&s_shell, "flux (Iq=0 coast):\r\n");
  print_f("  flux_mWb", c->flux);
  shellWriteString(&s_shell, "mechanical (J/B/Tc):\r\n");
  print_f("  J_g.cm2", s_motor->imech.j);
  print_f("  B_uNm.s", s_motor->imech.b);
  print_f("  Tc_mNm", s_motor->imech.tc);
  return 0;
}

// Re-deploy the current-loop PI gains from the stored Rs/Ld/Lq at a chosen
// bandwidth (default 1 kHz) without re-identifying: Kp = L*w_bw, Ki = R*w_bw
// (continuous; the integrator scales Ki by Ts). Pole-zero cancellation ->
// first-order closed loop. autocal does this automatically.
static int cmd_tune(int argc, char *argv[]) {
  if (foc_get_state(s_motor) != FOC_STATE_IDLE) {
    shellWriteString(&s_shell, "need IDLE first (use 'stop')\r\n");
    return 0;
  }
  const params_t *p = params_get();
  if (p->rs <= 0.0f || p->ld <= 0.0f || p->lq <= 0.0f) {
    shellWriteString(&s_shell, "no ident data; run 'autocal' first\r\n");
    return 0;
  }
  float bw = (argc >= 2) ? arg_f(argv[1]) : 1000.0f;
  params_tune_current_loop(s_motor, bw);
  shellPrint(&s_shell, "tuned current loop @ %d Hz from Rs/Ld/Lq:\r\n", (int)bw);
  print_f("  id_kp", p->id_kp);
  print_f("  id_ki", p->id_ki);
  print_f("  iq_kp", p->iq_kp);
  print_f("  iq_ki", p->iq_ki);
  shellWriteString(&s_shell, "applied; auto-saves in IDLE\r\n");
  return 0;
}

// Re-deploy the speed-loop gains from the stored J/B at a chosen bandwidth
// (default 10 Hz) without re-identifying.
static int cmd_tunespd(int argc, char *argv[]) {
  if (foc_get_state(s_motor) != FOC_STATE_IDLE) {
    shellWriteString(&s_shell, "need IDLE first (use 'stop')\r\n");
    return 0;
  }
  const params_t *p = params_get();
  if (!p->mech_valid || p->inertia_j <= 0.0f || p->flux <= 0.0f) {
    shellWriteString(&s_shell, "no J/B data; run 'autocal' first\r\n");
    return 0;
  }
  float bw = (argc >= 2) ? arg_f(argv[1]) : 10.0f;
  params_tune_speed_loop(s_motor, bw);
  shellPrint(&s_shell, "tuned speed loop @ %d Hz from J/B:\r\n", (int)bw);
  print_f("  spd_kp", p->spd_kp);
  print_f("  spd_ki", p->spd_ki);
  shellWriteString(&s_shell, "applied; auto-saves in IDLE\r\n");
  return 0;
}

static int cmd_stop(int argc, char *argv[]) {
  (void)argc; (void)argv;
  autocal_abort();                 /* cancel any running auto-calibration */
  foc_set_state(s_motor, FOC_STATE_IDLE);
  shellWriteString(&s_shell, "stop -> IDLE\r\n");
  return 0;
}

// Anti-cogging map: calibrate / enable / disable / clear / info.
//   acog cal [rpm]  start a calibration sweep (steps 1 rev each way; ~3.5 min)
//   acog on|off     enable/disable the run-time feed-forward (persisted in IDLE)
//   acog clear      discard the map (deletes the stored copy)
//   acog            show map status
static void acog_print_info(void) {
  shellPrint(&s_shell, "acog: valid=%d enabled=%d\r\n",
             (int)foc_acog_is_valid(s_motor), (int)foc_acog_is_enabled(s_motor));
  print_f("  peak_A", foc_acog_peak(s_motor));
}

static int cmd_acog(int argc, char *argv[]) {
  if (argc < 2) {
    acog_print_info();
    shellWriteString(&s_shell, "usage: acog <cal [rpm]|on|off|clear|info>\r\n");
    return 0;
  }
  const char *sub = argv[1];

  if (strcmp(sub, "cal") == 0) {
    if (foc_get_state(s_motor) != FOC_STATE_IDLE) {
      shellWriteString(&s_shell, "need IDLE first (use 'stop')\r\n");
      return 0;
    }
    if (!params_get()->cali_valid) {
      shellWriteString(&s_shell, "encoder not aligned; run 'autocal' first\r\n");
      return 0;
    }
    if (!s_motor->sensor.cur_offset.is_calibrated) {
      shellWriteString(&s_shell, "current offsets not calibrated\r\n");
      return 0;
    }
    float rpm = (argc >= 3) ? arg_f(argv[2]) : 0.0f;   /* 0 -> default creep */
    foc_start_acog(s_motor, rpm, 0.0f);
    shellWriteString(&s_shell, "anti-cogging cal started (~3.5 min):\r\n");
    shellWriteString(&s_shell, "  rotor STEPS one rev each way - keep clear! watch 'status'\r\n");
  } else if (strcmp(sub, "on") == 0 || strcmp(sub, "off") == 0) {
    uint8_t on = (sub[1] == 'n');                      /* "on" vs "off" */
    foc_acog_set_enable(s_motor, on);
    if (foc_get_state(s_motor) == FOC_STATE_IDLE) {
      params_capture_acog(s_motor);                    /* persist the enable flag */
      shellPrint(&s_shell, "acog %s (saved)\r\n", on ? "on" : "off");
    } else {
      shellPrint(&s_shell, "acog %s (live; stop to persist)\r\n", on ? "on" : "off");
    }
  } else if (strcmp(sub, "clear") == 0) {
    foc_acog_clear(s_motor);
    if (foc_get_state(s_motor) == FOC_STATE_IDLE) {
      params_capture_acog(s_motor);                    /* invalid map -> deletes KV */
    }
    shellWriteString(&s_shell, "acog map cleared\r\n");
  } else if (strcmp(sub, "info") == 0) {
    acog_print_info();
  } else {
    shellWriteString(&s_shell, "usage: acog <cal [rpm]|on|off|clear|info>\r\n");
  }
  return 0;
}

// CAN low-level debug hooks (kept alongside the can_proto layer for raw bus
// debugging; they carry no protocol semantics).
//   cansend <id_hex> [b0 ... b7]  send one standard data frame (bytes in hex)
//   canstat                       show bus stats and drain received frames
static int cmd_cansend(int argc, char *argv[]) {
  if (argc < 2) {
    shellWriteString(&s_shell, "usage: cansend <id_hex> [b0..b7] (hex)\r\n");
    return 0;
  }
  uint32_t id = strtoul(argv[1], NULL, 16);
  uint8_t data[8] = {0};
  uint8_t dlc = (uint8_t)(argc - 2);
  if (dlc > 8u) {
    dlc = 8u;
  }
  for (uint8_t i = 0u; i < dlc; i++) {
    data[i] = (uint8_t)strtoul(argv[2 + i], NULL, 16);
  }
  if (can_bus_send(id, dlc, data)) {
    shellPrint(&s_shell, "can tx id=0x%03X dlc=%d\r\n", (unsigned int)id, (int)dlc);
  } else {
    shellWriteString(&s_shell, "can tx failed: no free mailbox (no bus / no ack?)\r\n");
  }
  return 0;
}

static int cmd_canstat(int argc, char *argv[]) {
  (void)argc; (void)argv;
  uint16_t dropped;
  uint8_t tec, rec;
  can_bus_get_stats(&dropped, &tec, &rec);
  shellPrint(&s_shell, "can: TEC=%d REC=%d rx_dropped=%d\r\n",
             (int)tec, (int)rec, (int)dropped);
  can_frame_t f;
  uint8_t n = 0u;
  while (n < 16u && can_bus_recv(&f)) {
    shellPrint(&s_shell, "  rx id=0x%03X dlc=%d data=", (unsigned int)f.id, (int)f.dlc);
    for (uint8_t i = 0u; i < f.dlc; i++) {
      shellPrint(&s_shell, "%02X ", (unsigned int)f.data[i]);
    }
    shellWriteString(&s_shell, "\r\n");
    n++;
  }
  if (n == 0u) {
    shellWriteString(&s_shell, "  (no rx frames)\r\n");
  }
  return 0;
}

// CAN protocol-layer status: node id, telemetry/watchdog state, rx/tx counts.
// (Low-level bus stats + raw frame drain are still `canstat`.)
static int cmd_canproto(int argc, char *argv[]) {
  (void)argc; (void)argv;
  can_proto_stats_t s;
  can_proto_get_stats(&s);
  shellPrint(&s_shell, "canproto: node=%d telem=%s wdog=%s",
             (int)s.node_id, s.telem_enabled ? "on" : "off",
             s.wdog_enabled ? "on" : "off");
  if (s.wdog_enabled) {
    shellPrint(&s_shell, " (%dms, remain %dms)", (int)s.wdog_ms, (int)s.wdog_remain_ms);
  }
  shellPrint(&s_shell, "\r\n  rx_cmds=%d rx_unknown=%d tx_telem=%d last_cmd=0x%02X\r\n",
             (int)s.rx_cmds, (int)s.rx_unknown, (int)s.tx_telem, (unsigned)s.last_cmd);
  shellWriteString(&s_shell, "  (set node with 'set can_id N', watchdog with 'set can_wdog ms')\r\n");
  return 0;
}

static int cmd_status(int argc, char *argv[]) {
  (void)argc; (void)argv;
  const fault_status_t *f = fault_get();

  shellPrint(&s_shell, "fw = %s\r\n", FW_VER_STRING);
  shellPrint(&s_shell, "mode = %s\r\n", state_name(foc_get_state(s_motor)));
  if (autocal_active()) {
    shellWriteString(&s_shell, "autocal: RUNNING (do not interrupt)\r\n");
  }
  print_f("vbus_V", motor_hw_get_vbus());

  shellWriteString(&s_shell, "target:\r\n");
  print_f("id_ref_A", s_motor->target.id);
  print_f("iq_ref_A", s_motor->target.iq);
  print_f("speed_ref_rpm", s_motor->target.rpm);
  print_f("pos_ref_deg", s_motor->target.pos * RAD_TO_DEG);

  shellWriteString(&s_shell, "actual:\r\n");
  print_f("id_A", s_motor->actual.id);
  print_f("iq_A", s_motor->actual.iq);
  print_f("speed_rpm", s_motor->actual.rpm);
  print_f("pos_deg", s_motor->actual.pos * RAD_TO_DEG);

  shellPrint(&s_shell, "fault active=0x%05x latched=0x%05x first=0x%05x\r\n",
             (unsigned)f->active, (unsigned)f->latched, (unsigned)f->first);
  shellWriteString(&s_shell, "active:");
  print_fault_names(f->active);
  shellWriteString(&s_shell, "\r\nlatched:");
  print_fault_names(f->latched);
  shellPrint(&s_shell, "\r\nDRV FS1=0x%03x FS2=0x%03x\r\n",
             (unsigned)f->drv_fs1, (unsigned)f->drv_fs2);
  return 0;
}

// One VOFA frame as text. Names MUST stay in sync with the enum in vofa.h.
static int cmd_dump(int argc, char *argv[]) {
  (void)argc; (void)argv;
  static const char *names[] = {
    "vd", "vq", "mech_theta", "svpwm_alpha", "svpwm_beta",
    "ccr1", "ccr2", "ccr3", "sector", "iu", "iv", "iw",
    "i_alpha", "i_beta", "id", "iq", "id_ref", "iq_ref",
    "pos_target", "pos_actual", "debug", "speed_actual", "speed_target",
  };
  for (int i = 0; i <= VOFA_CH_SPEED_TARGET; i++) {
    print_f(names[i], vofa_data[i]);
  }
  return 0;
}

// pid/lim route through params_set so a tuned gain applies live AND persists
// (auto-saved on the next return to IDLE), same path as the generic `set`.
static int cmd_pid(int argc, char *argv[]) {
  if (argc < 4) {
    shellWriteString(&s_shell, "usage: pid <id|iq|spd> <kp> <ki>\r\n");
    return 0;
  }
  const char *kp_key, *ki_key;
  if (strcmp(argv[1], "id") == 0) {
    kp_key = "id_kp"; ki_key = "id_ki";
  } else if (strcmp(argv[1], "iq") == 0) {
    kp_key = "iq_kp"; ki_key = "iq_ki";
  } else if (strcmp(argv[1], "spd") == 0) {
    kp_key = "spd_kp"; ki_key = "spd_ki";
  } else {
    shellWriteString(&s_shell, "loop must be id|iq|spd\r\n");
    return 0;
  }
  params_set(s_motor, kp_key, arg_f(argv[2]));
  params_set(s_motor, ki_key, arg_f(argv[3]));
  shellWriteString(&s_shell, "pid updated (auto-saves in IDLE)\r\n");
  return 0;
}

static int cmd_lim(int argc, char *argv[]) {
  if (argc < 3) {
    shellWriteString(&s_shell, "usage: lim <cur|spd> <value>\r\n");
    return 0;
  }
  const char *key;
  if (strcmp(argv[1], "cur") == 0) {
    key = "cur_lim";
  } else if (strcmp(argv[1], "spd") == 0) {
    key = "spd_lim";
  } else {
    shellWriteString(&s_shell, "target must be cur|spd\r\n");
    return 0;
  }
  params_set(s_motor, key, arg_f(argv[2]));
  shellWriteString(&s_shell, "limit updated (auto-saves in IDLE)\r\n");
  return 0;
}

static int cmd_clrfault(int argc, char *argv[]) {
  (void)argc; (void)argv;
  foc_clear_fault(s_motor);
  shellWriteString(&s_shell, (foc_get_state(s_motor) == FOC_STATE_ERROR)
                   ? "clear FAILED: a fault condition is still present\r\n"
                   : "clear OK -> IDLE\r\n");
  return 0;
}

static int cmd_clrlatch(int argc, char *argv[]) {
  (void)argc; (void)argv;
  fault_clear_latched();
  shellWriteString(&s_shell, "latched cleared\r\n");
  return 0;
}

static int cmd_scope(int argc, char *argv[]) {
  (void)argc; (void)argv;
  s_scope = 1u;
  shellWriteString(&s_shell, "[scope] streaming VOFA frames; press any key to stop\r\n");
  return 0;
}

static int cmd_reboot(int argc, char *argv[]) {
  (void)argc; (void)argv;
  shellWriteString(&s_shell, "rebooting...\r\n");
  NVIC_SystemReset();
  return 0;  // not reached
}

// --- Parameter (flash) commands ---

static int cmd_get(int argc, char *argv[]) {
  if (argc >= 2) {
    float v;
    if (params_get_value(argv[1], &v)) {
      print_f(argv[1], v);
    } else {
      shellWriteString(&s_shell, "unknown param\r\n");
    }
    return 0;
  }
  // no arg: list every parameter + value
  for (uint32_t i = 0u; i < params_field_count(); i++) {
    print_f(params_field_name(i), params_field_value(i));
  }
  return 0;
}

static int cmd_set(int argc, char *argv[]) {
  if (argc < 3) {
    shellWriteString(&s_shell, "usage: set <name> <value>  (run 'get' for names)\r\n");
    return 0;
  }
  if (!params_set(s_motor, argv[1], arg_f(argv[2]))) {
    shellWriteString(&s_shell, "unknown param\r\n");
    return 0;
  }
  // echo the effective (possibly clamped) value
  float v;
  params_get_value(argv[1], &v);
  print_f(argv[1], v);
  shellWriteString(&s_shell, "applied; auto-saves in IDLE (or use 'save')\r\n");
  return 0;
}

static int cmd_save(int argc, char *argv[]) {
  (void)argc; (void)argv;
  if (foc_get_state(s_motor) == FOC_STATE_IDLE) {
    params_save();
    shellWriteString(&s_shell, "saved to flash\r\n");
  } else {
    params_mark_dirty();
    shellWriteString(&s_shell, "not IDLE: queued, will save on return to IDLE\r\n");
  }
  return 0;
}

static int cmd_load(int argc, char *argv[]) {
  (void)argc; (void)argv;
  params_reload(s_motor);
  shellWriteString(&s_shell, "reloaded from flash (unsaved changes discarded)\r\n");
  return 0;
}

static int cmd_factory(int argc, char *argv[]) {
  (void)argc; (void)argv;
  if (foc_get_state(s_motor) != FOC_STATE_IDLE) {
    shellWriteString(&s_shell, "need IDLE first (use 'stop')\r\n");
    return 0;
  }
  params_factory_reset(s_motor);
  shellWriteString(&s_shell, "factory defaults restored + saved (re-align needed)\r\n");
  return 0;
}

static int cmd_thrprot(int argc, char *argv[]) {
  if (argc >= 2) {
    params_set(s_motor, "thr_en", arg_f(argv[1]));
  }
  float v = 0.0f;
  params_get_value("thr_en", &v);
  shellPrint(&s_shell, "thr_protect = %d (1=clamp to safe limits, 0=raw expert)\r\n",
             (int)v);
  return 0;
}

static int cmd_faultlog(int argc, char *argv[]) {
  (void)argc; (void)argv;
  uint32_t depth = params_fault_log_depth();
  if (depth == 0u) {
    shellWriteString(&s_shell, "fault log empty\r\n");
    return 0;
  }
  // newest first; survives power-off (stored in flash)
  for (uint32_t i = 0u; i < depth; i++) {
    const fault_status_t *e = params_fault_log_entry(i);
    shellPrint(&s_shell, "[%u] first=0x%05x latched=0x%05x tick=%ums\r\n",
               (unsigned)i, (unsigned)e->first, (unsigned)e->latched,
               (unsigned)e->tick_at_fault);
    shellWriteString(&s_shell, "    cause:");
    print_fault_names(e->first);
    shellWriteString(&s_shell, "\r\n");
    print_f("    vbus_V", e->vbus_at_fault);
    print_f("    iq_A", e->iq_at_fault);
    shellPrint(&s_shell, "    DRV FS1=0x%03x FS2=0x%03x\r\n",
               (unsigned)e->drv_fs1, (unsigned)e->drv_fs2);
  }
  return 0;
}

// --- Command table (SHELL_USING_CMD_EXPORT == 0: shell.c reads this directly).
//     The default user + line-editing keys mirror the library's shell_cmd_list.c
//     (which we do not compile); our commands are appended. ---
extern void shellUp(Shell *);
extern void shellDown(Shell *);
extern void shellLeft(Shell *);
extern void shellRight(Shell *);
extern void shellTab(Shell *);
extern void shellBackspace(Shell *);
extern void shellDelete(Shell *);
extern void shellEnter(Shell *);
extern void shellHelp(int, char *[]);
extern void shellClear(void);

#define CMD_ATTR \
  (SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN) | SHELL_CMD_DISABLE_RETURN)

const ShellCommand shellCommandList[] = {
  { .attr.value = SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_USER),
    .data.user.name = SHELL_DEFAULT_USER,
    .data.user.password = SHELL_DEFAULT_USER_PASSWORD,
    .data.user.desc = "default user" },

  SHELL_KEY_ITEM(SHELL_CMD_PERMISSION(0), 0x1B5B4100, shellUp, up),
  SHELL_KEY_ITEM(SHELL_CMD_PERMISSION(0), 0x1B5B4200, shellDown, down),
  SHELL_KEY_ITEM(SHELL_CMD_PERMISSION(0) | SHELL_CMD_ENABLE_UNCHECKED, 0x1B5B4300, shellRight, right),
  SHELL_KEY_ITEM(SHELL_CMD_PERMISSION(0) | SHELL_CMD_ENABLE_UNCHECKED, 0x1B5B4400, shellLeft, left),
  SHELL_KEY_ITEM(SHELL_CMD_PERMISSION(0), 0x09000000, shellTab, tab),
  SHELL_KEY_ITEM(SHELL_CMD_PERMISSION(0) | SHELL_CMD_ENABLE_UNCHECKED, 0x08000000, shellBackspace, backspace),
  SHELL_KEY_ITEM(SHELL_CMD_PERMISSION(0) | SHELL_CMD_ENABLE_UNCHECKED, 0x7F000000, shellDelete, delete),
  SHELL_KEY_ITEM(SHELL_CMD_PERMISSION(0) | SHELL_CMD_ENABLE_UNCHECKED, 0x1B5B337E, shellDelete, delete),
  SHELL_KEY_ITEM(SHELL_CMD_PERMISSION(0) | SHELL_CMD_ENABLE_UNCHECKED, 0x0A000000, shellEnter, enter),
  SHELL_KEY_ITEM(SHELL_CMD_PERMISSION(0) | SHELL_CMD_ENABLE_UNCHECKED, 0x0D000000, shellEnter, enter),

  SHELL_CMD_ITEM(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN) | SHELL_CMD_DISABLE_RETURN,
                 help, shellHelp, show command info),
  SHELL_CMD_ITEM(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC) | SHELL_CMD_DISABLE_RETURN,
                 cls, shellClear, clear screen),

  SHELL_CMD_ITEM(CMD_ATTR, mode, cmd_mode, switch/show mode <idle|current|speed|position|volt|openloop>),
  SHELL_CMD_ITEM(CMD_ATTR, speed, cmd_speed, set/show speed target (rpm)),
  SHELL_CMD_ITEM(CMD_ATTR, id, cmd_id, set/show d-axis current (A)),
  SHELL_CMD_ITEM(CMD_ATTR, iq, cmd_iq, set/show q-axis current (A)),
  SHELL_CMD_ITEM(CMD_ATTR, pos, cmd_pos, set/show position target (DEGREES)),
  SHELL_CMD_ITEM(CMD_ATTR, volt, cmd_volt, set voltage: volt <vd> <vq>),
  SHELL_CMD_ITEM(CMD_ATTR, autocal, cmd_autocal, full auto-calibration: align+ident+deploy (rotor moves/spins)),
  SHELL_CMD_ITEM(CMD_ATTR, identinfo, cmd_identinfo, show raw measurements of the last autocal run),
  SHELL_CMD_ITEM(CMD_ATTR, tune, cmd_tune, re-deploy current-loop gains: tune [bw_hz]),
  SHELL_CMD_ITEM(CMD_ATTR, tunespd, cmd_tunespd, re-deploy speed-loop gains: tunespd [bw_hz]),
  SHELL_CMD_ITEM(CMD_ATTR, acog, cmd_acog, anti-cogging map: acog <cal [rpm]|on|off|clear|info>),
  SHELL_CMD_ITEM(CMD_ATTR, cansend, cmd_cansend, CAN debug: cansend <id_hex> [b0..b7] (hex)),
  SHELL_CMD_ITEM(CMD_ATTR, canstat, cmd_canstat, CAN debug: bus stats + drain rx frames),
  SHELL_CMD_ITEM(CMD_ATTR, canproto, cmd_canproto, CAN protocol status: node/telem/wdog/counters),
  SHELL_CMD_ITEM(CMD_ATTR, stop, cmd_stop, stop -> IDLE),
  SHELL_CMD_ITEM(CMD_ATTR, status, cmd_status, show mode + targets + actuals + faults),
  SHELL_CMD_ITEM(CMD_ATTR, dump, cmd_dump, print all telemetry channels (text VOFA frame)),
  SHELL_CMD_ITEM(CMD_ATTR, pid, cmd_pid, set PID gains: pid <id|iq|spd> <kp> <ki>),
  SHELL_CMD_ITEM(CMD_ATTR, lim, cmd_lim, set limit: lim <cur|spd> <value>),
  SHELL_CMD_ITEM(CMD_ATTR, clrfault, cmd_clrfault, clear active fault (ERROR -> IDLE)),
  SHELL_CMD_ITEM(CMD_ATTR, clrlatch, cmd_clrlatch, clear latched fault history),
  SHELL_CMD_ITEM(CMD_ATTR, get, cmd_get, list/show stored params: get [name]),
  SHELL_CMD_ITEM(CMD_ATTR, set, cmd_set, set stored param: set <name> <value>),
  SHELL_CMD_ITEM(CMD_ATTR, save, cmd_save, write params to flash (IDLE)),
  SHELL_CMD_ITEM(CMD_ATTR, load, cmd_load, reload params from flash (discard unsaved)),
  SHELL_CMD_ITEM(CMD_ATTR, factory, cmd_factory, restore factory defaults (IDLE)),
  SHELL_CMD_ITEM(CMD_ATTR, thrprot, cmd_thrprot, threshold guard: thrprot <0|1>),
  SHELL_CMD_ITEM(CMD_ATTR, faultlog, cmd_faultlog, show stored fault history (survives power-off)),
  SHELL_CMD_ITEM(CMD_ATTR, scope, cmd_scope, stream VOFA waveform (press key to stop)),
  SHELL_CMD_ITEM(CMD_ATTR, reboot, cmd_reboot, software reset),
};

const unsigned short shellCommandCount = sizeof(shellCommandList) / sizeof(ShellCommand);

void shell_port_init(const io_stream_t *io, motor_t *motor) {
  s_io = io;
  s_motor = motor;
  s_scope = 0u;

  s_shell.write = shell_write_adapter;
  s_shell.read = shell_read_adapter;
  shellInit(&s_shell, s_shell_buffer, sizeof(s_shell_buffer));
}

void shell_port_poll(void) {
  if (s_scope) {
    uint8_t frame[SCOPE_CHANNELS * 4u + 4u];
    uint32_t n = vofa_build_justfloat(vofa_data, (uint8_t)SCOPE_CHANNELS, frame, sizeof(frame));
    if (n != 0u) {
      s_io->write(frame, n);
    }
    char c;
    if (s_io->read(&c, 1u) == 1u) {  // any key exits scope mode
      s_scope = 0u;
      shellWriteString(&s_shell, "\r\n[scope] stopped\r\n");
    }
    return;
  }

  char c;
  while (s_io->read(&c, 1u) == 1u) {
    shellHandler(&s_shell, c);
  }
}
