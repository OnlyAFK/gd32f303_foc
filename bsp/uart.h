#ifndef UART_H
#define UART_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "io_stream.h"

// USART2 on PB10 (TX) / PB11 (RX), DMA-driven with lwrb ring buffers.
// TX: application data is queued in a ring buffer and drained by DMA0_CH1.
// RX: DMA0_CH2 fills a circular buffer; IDLE + DMA half/full events push the
//     received bytes into an RX ring buffer that the application reads from.

void uart_init(void);

// Queue bytes for transmission. Returns the number of bytes actually queued
// (may be less than len if the TX ring buffer is full). Non-blocking.
uint32_t uart_write(const void *data, uint32_t len);

// Pop up to len bytes from the RX ring buffer. Returns bytes copied out.
uint32_t uart_read(void *data, uint32_t len);

// Number of bytes currently available to read.
uint32_t uart_rx_available(void);

// io_stream backend bound to this UART (write/read). Used by the CLI/telemetry
// so they can be retargeted to another transport (e.g. USB CDC) later.
const io_stream_t *uart_stream(void);

#ifdef __cplusplus
}
#endif

#endif /* UART_H */
