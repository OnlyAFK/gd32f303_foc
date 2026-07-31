#ifndef TLE5012B_H
#define TLE5012B_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

// TLE5012B GMR angle sensor over the 3-wire SSC interface (SPI-compatible).
// Wiring (see doc and pinmap):
//   SPI2: SCK=PB3, MISO=PB4, MOSI=PB5 (-> 100R -> DATA), CSQ=PA15 (software GPIO)
//   MISO and MOSI share the single bidirectional DATA line through a 100R
//   series resistor, so the bus runs as full-duplex SPI: the master sends the
//   command on MOSI, then clocks dummy words while the sensor drives DATA and
//   the master samples it on MISO.
//
// NOTE: PB3/PB4/PA15 are JTAG pins. tle5012b_init() disables JTAG (keeping
//       SWD) so these pins are usable. SWD debugging still works; JTAG does not.
//
// The angle is fetched by DMA (SPI2 RX = DMA1_CH0, TX = DMA1_CH1). Call
// tle5012b_start_read() periodically (e.g. from the FOC loop); the completion
// interrupt stores the latest angle, which tle5012b_get_* return without
// blocking.

bool tle5012b_init(void);

// Register a callback invoked from the DMA-complete ISR once a read started by
// tle5012b_start_read() has finished and the angle has been stored. Pass NULL
// to detach. NOTE: the FOC layer polls at the top of its control tick instead
// of using this hook, so angle processing stays off the DMA ISR (no shared-
// state race with the control loop). Kept for non-FOC users.
void tle5012b_attach_callback(void (*cb)(void));

// Kick off a non-blocking DMA read of the angle register. The result lands in
// the internal angle store when the transfer completes (see tle5012b_busy).
void tle5012b_start_read(void);

// True while a DMA read started by tle5012b_start_read() is still in flight.
bool tle5012b_busy(void);

// Latest 15-bit raw angle (0..32767 over one mechanical revolution).
uint16_t tle5012b_get_raw(void);

// Latest angle in radians, [0, 2*pi).
float tle5012b_get_angle_rad(void);

// Blocking single read (start + wait), returns the 15-bit raw angle. Intended
// for bring-up and calibration, not the real-time control loop.
uint16_t tle5012b_read_raw_blocking(void);

// Diagnostic: the value read back on MISO during the command word. On a
// healthy bus it echoes the command driven on MOSI (~0x8020). A reading of
// 0xFFFF or 0x0000 means SCK/MOSI/MISO are not reaching the sensor.
uint16_t tle5012b_debug_cmd_echo(void);

// True when the init-time config read-back confirmed (or successfully
// corrected) the assumed sensor configuration: fast 42.7 us filter update,
// prediction on, autocalibration mode 1 (the TLE5012B-E1000 fuse defaults).
// False means a different derivate is fitted and could not be corrected —
// the loop still runs but with degraded angle quality.
bool tle5012b_config_ok(void);

// Raw MOD_1 / MOD_2 register values as read back during init (after any
// correction), for diagnostics.
void tle5012b_get_config(uint16_t *mod1, uint16_t *mod2);

#ifdef __cplusplus
}
#endif

#endif /* TLE5012B_H */
