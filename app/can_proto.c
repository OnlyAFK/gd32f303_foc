#include "can_proto.h"

#include "can.h"
#include "params.h"
#include "fault.h"
#include "motor_hw.h"

/* --- ID field layout: ID = (node_id << 5) | cmd_id --- */
#define CAN_CMD_BITS   5u
#define CAN_CMD_MASK   0x1Fu
#define CAN_NODE_BCAST 0u                 /* node 0 = broadcast (commands only) */
#define CAN_ID_OF(node, cmd)  (((uint32_t)(node) << CAN_CMD_BITS) | (uint32_t)(cmd))
#define CAN_NODE_OF(id)       (uint8_t)(((id) >> CAN_CMD_BITS) & 0x3Fu)
#define CAN_CMD_OF(id)        (uint8_t)((id) & CAN_CMD_MASK)

/* Telemetry cadence: poll runs every ~1 ms. Each frame type is emitted every
 * 20 ms (50 Hz); the two are offset by 10 ms so a frame leaves every 10 ms. */
#define TELEM_PERIOD_MS   20u
#define TELEM_FEEDBACK_PHASE_MS  10u

/* Heartbeat flags byte. */
#define HB_FLAG_TELEM     0x01u
#define HB_FLAG_FAULT     0x02u
#define HB_FLAG_DRIVING   0x04u

static motor_t *s_motor;
static params_t *s_p;                     /* live parameter set (can_id, can_wdog) */

static uint32_t s_tick_ms;                /* +1 per poll, our millisecond base */
static uint32_t s_last_cmd_ms;            /* tick of the last accepted command */
static uint8_t  s_can_drive;              /* 1 while CAN is responsible for the current
                                             motion (arms the watchdog; a shell-driven
                                             motor is left alone) */
static uint8_t  s_telem_on;               /* periodic telemetry enabled (default off) */
static uint8_t  s_filter_node;            /* node currently in the HW RX filter */

/* stats (mirrored into can_proto_stats_t on request) */
static uint32_t s_rx_cmds;
static uint32_t s_rx_unknown;
static uint32_t s_tx_telem;
static uint8_t  s_last_cmd;

/* --- little-endian payload packing helpers --- */
static int16_t rd_i16(const uint8_t *p) {
  return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}
static int32_t rd_i32(const uint8_t *p) {
  return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                   ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
}
static void wr_i16(uint8_t *p, int16_t v) {
  p[0] = (uint8_t)(v & 0xFFu);
  p[1] = (uint8_t)((v >> 8) & 0xFFu);
}
static void wr_u32(uint8_t *p, uint32_t v) {
  p[0] = (uint8_t)(v & 0xFFu);
  p[1] = (uint8_t)((v >> 8) & 0xFFu);
  p[2] = (uint8_t)((v >> 16) & 0xFFu);
  p[3] = (uint8_t)((v >> 24) & 0xFFu);
}

/* Map the protocol mode byte onto a foc_set_state call, clearing that mode's
 * setpoint first so the motor does not lurch on a stale target (mirrors the
 * shell `mode` command). Unknown mode -> ignored. */
static void apply_mode(uint8_t mode) {
  switch (mode) {
  case CAN_MODE_IDLE:
    foc_set_state(s_motor, FOC_STATE_IDLE);
    break;
  case CAN_MODE_CURRENT:
    foc_set_target_current(s_motor, 0.0f, 0.0f);
    foc_set_state(s_motor, FOC_STATE_CURRENT);
    break;
  case CAN_MODE_SPEED:
    foc_set_target_speed(s_motor, 0.0f);
    foc_set_state(s_motor, FOC_STATE_SPEED);
    break;
  case CAN_MODE_POSITION:
    foc_set_state(s_motor, FOC_STATE_POSITION);  /* self-holds present angle */
    break;
  case CAN_MODE_VOLT:
    foc_set_voltage(s_motor, 0.0f, 0.0f);
    foc_set_state(s_motor, FOC_STATE_CLOSEDLOOP_VOLT);
    break;
  case CAN_MODE_OPENLOOP:
    foc_set_voltage(s_motor, 0.0f, 0.0f);
    foc_set_openloop_speed(s_motor, 0.0f);
    foc_set_state(s_motor, FOC_STATE_OPENLOOP_VOLT);
    break;
  default:
    return;   /* unknown mode: ignore entirely (watchdog arming untouched) */
  }

  /* A KNOWN mode other than IDLE means CAN is now driving the motor -> arm
   * the watchdog against this motion. IDLE hands responsibility back. */
  s_can_drive = (mode != CAN_MODE_IDLE) ? 1u : 0u;
}

/* Dispatch one received command frame. Returns 1 if it carried a recognised
 * command (feeds the watchdog), 0 otherwise. */
static uint8_t dispatch(const can_frame_t *f) {
  uint8_t cmd = CAN_CMD_OF(f->id);

  switch (cmd) {
  case CAN_CMD_SET_MODE:
    if (f->dlc >= 1u) {
      apply_mode(f->data[0]);
    }
    break;

  case CAN_CMD_SET_CURRENT:
    if (f->dlc >= 4u) {
      float iq = (float)rd_i16(&f->data[0]) * 0.001f;   /* mA -> A */
      float id = (float)rd_i16(&f->data[2]) * 0.001f;
      foc_set_target_current(s_motor, id, iq);
    }
    break;

  case CAN_CMD_SET_SPEED:
    if (f->dlc >= 4u) {
      float rpm = (float)rd_i32(&f->data[0]) * 0.01f;   /* rpm*100 -> rpm */
      foc_set_target_speed(s_motor, rpm);
    }
    break;

  case CAN_CMD_SET_POS:
    if (f->dlc >= 4u) {
      float pos = (float)rd_i32(&f->data[0]) * 0.0001f; /* rad*10000 -> rad */
      foc_set_target_position(s_motor, pos);
    }
    break;

  case CAN_CMD_ESTOP:
    foc_set_state(s_motor, FOC_STATE_IDLE);
    s_can_drive = 0u;
    break;

  case CAN_CMD_CLEAR_FAULT:
    foc_clear_fault(s_motor);
    break;

  case CAN_CMD_SET_TELEM:
    if (f->dlc >= 1u) {
      s_telem_on = (f->data[0] != 0u) ? 1u : 0u;
    }
    break;

  case CAN_CMD_MIT_CONTROL:
    /* Reserved for impedance control; not implemented yet. Count it as unknown
     * so the host can tell it was not acted on. */
    return 0u;

  default:
    return 0u;   /* unrecognised cmd_id */
  }

  s_last_cmd = cmd;
  return 1u;
}

/* Emit the heartbeat frame: control state + active fault mask + flags. */
static void send_heartbeat(void) {
  const fault_status_t *fs = fault_get();
  foc_state_t st = foc_get_state(s_motor);
  uint8_t d[8] = {0};

  d[0] = (uint8_t)st;
  wr_u32(&d[1], fs->active);
  d[5] = (uint8_t)((s_telem_on ? HB_FLAG_TELEM : 0u) |
                   (fs->active ? HB_FLAG_FAULT : 0u) |
                   ((st != FOC_STATE_IDLE && st != FOC_STATE_ERROR) ? HB_FLAG_DRIVING : 0u));
  if (can_bus_send(CAN_ID_OF(s_p->can_id, CAN_CMD_HEARTBEAT), 6u, d)) {
    s_tx_telem++;
  }
}

/* Emit the feedback frame: rpm / iq / pos / vbus as scaled int16s. */
static void send_feedback(void) {
  uint8_t d[8] = {0};

  wr_i16(&d[0], (int16_t)(s_motor->actual.rpm));
  wr_i16(&d[2], (int16_t)(s_motor->actual.iq * 1000.0f));           /* A -> mA */
  wr_i16(&d[4], (int16_t)(s_motor->actual.pos * 100.0f));           /* rad*100 */
  wr_i16(&d[6], (int16_t)(motor_hw_get_vbus() * 10.0f));            /* V*10 */
  if (can_bus_send(CAN_ID_OF(s_p->can_id, CAN_CMD_FEEDBACK), 8u, d)) {
    s_tx_telem++;
  }
}

void can_proto_init(motor_t *motor) {
  s_motor = motor;
  s_p = params_get();
  s_tick_ms = 0u;
  s_last_cmd_ms = 0u;
  s_can_drive = 0u;
  s_telem_on = 0u;              /* telemetry off by default (quiet bus for debug) */
  s_rx_cmds = 0u;
  s_rx_unknown = 0u;
  s_tx_telem = 0u;
  s_last_cmd = 0xFFu;

  /* Tighten the RX filter to this node + broadcast (was accept-all). */
  s_filter_node = (uint8_t)(s_p->can_id & 0x3Fu);
  can_bus_set_node_filter(s_filter_node);
}

void can_proto_poll(void) {
  can_frame_t f;

  s_tick_ms++;

  /* --- drain and dispatch received command frames --- */
  while (can_bus_recv(&f)) {
    if (f.ide || f.rtr) {
      continue;                 /* protocol uses standard data frames only */
    }
    uint8_t node = CAN_NODE_OF(f.id);
    if (node != s_p->can_id && node != CAN_NODE_BCAST) {
      continue;                 /* addressed to another node */
    }
    if (dispatch(&f)) {
      s_rx_cmds++;
      s_last_cmd_ms = s_tick_ms;
    } else {
      s_rx_unknown++;
    }
  }

  /* --- live re-addressing: `set can_id N` only touches params, so keep the
   * hardware filter (and thus what reaches dispatch) in sync with it --- */
  if ((uint8_t)(s_p->can_id & 0x3Fu) != s_filter_node) {
    s_filter_node = (uint8_t)(s_p->can_id & 0x3Fu);
    can_bus_set_node_filter(s_filter_node);
  }

  /* --- command watchdog: stop if the host goes silent while CAN is driving.
   * Gated on s_can_drive so a shell-driven motor is never touched. If the motor
   * left the driving states by another path (fault, shell `stop`), drop the flag. */
  if (s_can_drive) {
    foc_state_t st = foc_get_state(s_motor);
    if (st == FOC_STATE_IDLE || st == FOC_STATE_ERROR) {
      s_can_drive = 0u;
    } else if (s_p->can_wdog_ms > 0u &&
               (s_tick_ms - s_last_cmd_ms) >= s_p->can_wdog_ms) {
      foc_set_state(s_motor, FOC_STATE_IDLE);
      s_can_drive = 0u;
    }
  }

  /* --- periodic telemetry: heartbeat and feedback interleaved --- */
  if (s_telem_on) {
    uint32_t phase = s_tick_ms % TELEM_PERIOD_MS;
    if (phase == 0u) {
      send_heartbeat();
    } else if (phase == TELEM_FEEDBACK_PHASE_MS) {
      send_feedback();
    }
  }
}

void can_proto_get_stats(can_proto_stats_t *out) {
  out->node_id       = s_p->can_id;
  out->telem_enabled = s_telem_on;
  out->wdog_enabled  = (s_p->can_wdog_ms > 0u) ? 1u : 0u;
  out->wdog_ms       = s_p->can_wdog_ms;
  out->rx_cmds       = s_rx_cmds;
  out->rx_unknown    = s_rx_unknown;
  out->tx_telem      = s_tx_telem;
  out->last_cmd      = s_last_cmd;

  /* remaining watchdog time, only meaningful while enabled + CAN is driving */
  if (out->wdog_enabled && s_can_drive) {
    uint32_t elapsed = s_tick_ms - s_last_cmd_ms;
    out->wdog_remain_ms = (elapsed >= s_p->can_wdog_ms)
                          ? 0u : (uint16_t)(s_p->can_wdog_ms - elapsed);
  } else {
    out->wdog_remain_ms = 0u;
  }
}
