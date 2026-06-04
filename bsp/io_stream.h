#ifndef IO_STREAM_H
#define IO_STREAM_H

#include <stdint.h>

// Generic byte-stream transport interface. The CLI and telemetry sit on top of
// this so the underlying link can be swapped without touching them: today the
// backend is the UART (bsp/uart.c, uart_stream()); a USB CDC backend can be
// added later as another io_stream_t with no change to the upper layers.
//
// Both calls are non-blocking and return the number of bytes actually moved.
typedef struct {
  uint32_t (*write)(const void *data, uint32_t len);
  uint32_t (*read)(void *data, uint32_t len);
} io_stream_t;

#endif /* IO_STREAM_H */
