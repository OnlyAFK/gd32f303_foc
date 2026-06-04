#ifndef SHELL_PORT_H
#define SHELL_PORT_H

#include "foc_core.h"
#include "io_stream.h"

// letter-shell based command console over a swappable transport. Bind it to an
// io_stream (UART today, USB CDC later) and the active motor, then service it
// from the main loop with shell_port_poll().
void shell_port_init(const io_stream_t *io, motor_t *motor);

// Service the shell. In normal mode it feeds received bytes to the shell; in
// scope mode it streams VOFA+ waveform frames and exits on any received key.
void shell_port_poll(void);

#endif /* SHELL_PORT_H */
