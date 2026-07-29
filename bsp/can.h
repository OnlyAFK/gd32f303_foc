#ifndef CAN_H
#define CAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// CAN0 frame transport on PA11 (RX) / PA12 (TX), classic CAN 2.0B.
//
// Unlike the UART this is a FRAME-oriented device, so it is deliberately NOT
// an io_stream backend: callers exchange whole CAN frames. The API is named
// can_bus_* because the GD32 SPL already owns the can_* prefix.
//
// All calls are non-blocking. RX frames are buffered in an internal ring
// (filled by the FIFO0 interrupt); can_bus_recv() pops one frame at a time.
// can_bus_send() queues a frame into a hardware TX mailbox and returns 0 when
// all mailboxes are busy (caller retries later).

typedef struct {
  uint32_t id;     // frame identifier (11-bit standard, 29-bit if ide)
  uint8_t  ide;    // 0 = standard frame, 1 = extended frame
  uint8_t  rtr;    // 0 = data frame, 1 = remote frame
  uint8_t  dlc;    // data length 0..8
  uint8_t  data[8];
} can_frame_t;

// Bring up CAN0 (GPIO, bit timing, accept-all filter, FIFO0 RX interrupt).
// The protocol layer may later tighten reception with can_bus_set_node_filter.
void can_bus_init(void);

// Queue a standard data frame for transmission. id is the 11-bit identifier,
// dlc is clamped to 8. Returns 1 if the frame was loaded into a TX mailbox,
// 0 if all mailboxes were busy.
uint8_t can_bus_send(uint32_t id, uint8_t dlc, const uint8_t *data);

// Pop the oldest received frame. Returns 1 on success, 0 if the ring is empty.
uint8_t can_bus_recv(can_frame_t *frame);

// Tighten the RX acceptance to one protocol node: standard frames whose ID is
// (node_id << 5) | cmd_id with cmd_id don't-care, plus the broadcast node 0.
// Replaces the accept-all filter set by can_bus_init(); call again to
// re-address (node_id 1..63). To sniff the whole bus again, re-init.
void can_bus_set_node_filter(uint8_t node_id);

// Bus health diagnostics: frames dropped because the RX ring was full, plus
// the hardware transmit/receive error counters (CAN_ERR register).
void can_bus_get_stats(uint16_t *rx_dropped, uint8_t *tec, uint8_t *rec);

#ifdef __cplusplus
}
#endif

#endif /* CAN_H */
