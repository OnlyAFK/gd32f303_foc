#ifndef CAN_PROTO_H
#define CAN_PROTO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "foc_core.h"

/*
 * CAN application protocol layer.
 *
 * Sits on top of the frame transport in bsp/can.{c,h} and translates CAN
 * frames <-> the FOC control API (foc_set_state / foc_set_target_*). It owns
 * NO control logic: command frames call the same setters the shell uses, so
 * CAN and the letter-shell console are peer front-ends onto one motor.
 *
 * --- Addressing (11-bit standard IDs) ---------------------------------------
 * The ID is split, ODrive-style, into a per-node address and a command code:
 *
 *     ID = (node_id << 5) | cmd_id      node_id 1..63, cmd_id 0..31
 *
 * node_id 0 is reserved for broadcast (commands only). The node's own address
 * comes from the `can_id` parameter (params.c), so a board can be re-addressed
 * without recompiling. The RX hardware filter accepts only this node's IDs plus
 * broadcast; it is applied in can_proto_init and re-applied live when `can_id`
 * changes (see can_proto_poll).
 *
 * --- Command frames (host -> board), cmd_id ---------------------------------
 *   0x01 SET_MODE     b0 = mode enum (see can_proto_mode_t)
 *   0x02 SET_CURRENT  int16 iq_mA, int16 id_mA          (little-endian)
 *   0x03 SET_SPEED    int32 rpm_x100
 *   0x04 SET_POSITION int32 pos_rad_x10000
 *   0x05 ESTOP        (no payload) -> immediate IDLE
 *   0x06 CLEAR_FAULT  (no payload) -> ERROR -> IDLE
 *   0x08 SET_TELEM    b0 = 0 stop / 1 start periodic telemetry
 *   0x10 MIT_CONTROL  Mini Cheetah impedance control frame (8 bytes):
 *                     p[15:0]  pos_ref  [0, 2pi) rad mech      (2pi/65536 /bit)
 *                     v[11:0]  vel_ref  +/-61.44 rad/s mech    (0.03 /bit, signed)
 *                     kp[11:0] stiffness 0..499.7 Nm/rad       (0.122 /bit)
 *                     kd[11:0] damping 0..98.3 Nm*s/rad        (0.024 /bit)
 *                     t[11:0]  tau_ff   +/-9.03 Nm             (0.00441 /bit, signed)
 *                     Entering MIT mode when not already there.
 *
 * --- Telemetry frames (board -> host), cmd_id -------------------------------
 * Sent periodically when telemetry is enabled (off by default), two frames
 * interleaved at 50 Hz each (~10 ms total period):
 *   0x00 HEARTBEAT    b0 state, b1..4 fault_active(u32 LE), b5 flags
 *   0x07 FEEDBACK     int16 rpm, int16 iq_mA, int16 pos_rad_x100, int16 vbus_x10
 *
 * All calls are non-blocking and run from the ~1 ms main loop.
 */

/* cmd_id field (low 5 bits of the standard ID). */
typedef enum {
  CAN_CMD_HEARTBEAT   = 0x00,   /* board -> host (telemetry) */
  CAN_CMD_SET_MODE    = 0x01,
  CAN_CMD_SET_CURRENT = 0x02,
  CAN_CMD_SET_SPEED   = 0x03,
  CAN_CMD_SET_POS     = 0x04,
  CAN_CMD_ESTOP       = 0x05,
  CAN_CMD_CLEAR_FAULT = 0x06,
  CAN_CMD_FEEDBACK    = 0x07,   /* board -> host (telemetry) */
  CAN_CMD_SET_TELEM   = 0x08,
  CAN_CMD_MIT_CONTROL = 0x10,   /* Mini Cheetah impedance control (see header doc) */
} can_cmd_t;

/* SET_MODE payload byte 0. Values are protocol-stable; they map onto the
 * subset of foc_state_t that a host is allowed to request directly. */
typedef enum {
  CAN_MODE_IDLE     = 0,
  CAN_MODE_CURRENT  = 1,
  CAN_MODE_SPEED    = 2,
  CAN_MODE_POSITION = 3,
  CAN_MODE_VOLT     = 4,
  CAN_MODE_OPENLOOP = 5,
  CAN_MODE_MIT      = 6,   /* impedance control (PD + torque FF via CAN) */
} can_proto_mode_t;

/* Bring up the protocol layer: bind the motor, read the node address from
 * params, and tighten the CAN RX filter to this node + broadcast. Call once,
 * after can_bus_init() and params_init(). */
void can_proto_init(motor_t *motor);

/* Service the protocol layer. Call every ~1 ms from the main loop: drains and
 * dispatches received command frames, runs the command watchdog (if enabled),
 * and emits periodic telemetry (if enabled). */
void can_proto_poll(void);

/* Diagnostics for the `canproto` shell command. */
typedef struct {
  uint8_t  node_id;
  uint8_t  telem_enabled;
  uint8_t  wdog_enabled;
  uint16_t wdog_ms;             /* configured timeout */
  uint16_t wdog_remain_ms;      /* ms left before stop (only if driving + enabled) */
  uint32_t rx_cmds;             /* command frames accepted */
  uint32_t rx_unknown;          /* frames with an unrecognised cmd_id */
  uint32_t tx_telem;            /* telemetry frames sent */
  uint32_t last_cmd;            /* most recent accepted cmd_id */
} can_proto_stats_t;

void can_proto_get_stats(can_proto_stats_t *out);

#ifdef __cplusplus
}
#endif

#endif /* CAN_PROTO_H */
