#ifndef VOFA_H
#define VOFA_H

#include <stdint.h>
#include "foc_core.h"

#define VOFA_CHANNEL_CNT  30

enum {
  VOFA_CH_VD = 0,
  VOFA_CH_VQ,
  VOFA_CH_MECH_THETA,
  VOFA_CH_SVPWM_ALPHA,
  VOFA_CH_SVPWM_BETA,
  VOFA_CH_CCR1, // 5
  VOFA_CH_CCR2,
  VOFA_CH_CCR3,
  VOFA_CH_SECTOR,
  VOFA_CH_IU,
  VOFA_CH_IV, // 10
  VOFA_CH_IW,
  VOFA_CH_I_ALPHA,
  VOFA_CH_I_BETA,
  VOFA_CH_ID,
  VOFA_CH_IQ, // 15
  VOFA_CH_ID_REF,
  VOFA_CH_IQ_REF,
  VOFA_CH_POS_TARGET,
  VOFA_CH_POS_ACTUAL,
  VOFA_CH_DEBUG, // 20
  VOFA_CH_SPEED_ACTUAL,
  VOFA_CH_SPEED_TARGET,
};

extern float vofa_data[VOFA_CHANNEL_CNT];

// Send `count` floats as a VOFA+ "justfloat" frame over the debug UART.
void vofa_send_justfloat(float *data_ptr, uint8_t count);

// Build a VOFA+ "justfloat" frame (count floats + tail) into `out`. Returns the
// frame length in bytes, or 0 if `out_size` is too small. Transport-agnostic:
// the caller writes the bytes to whatever io_stream it likes.
uint32_t vofa_build_justfloat(const float *data, uint8_t count,
                              uint8_t *out, uint32_t out_size);

void vofa_update_from_motor(motor_t *motor);

static inline void vofa_set_channel(uint8_t channel, float value) {
  if (channel < VOFA_CHANNEL_CNT) {
    vofa_data[channel] = value;
  }
}

#endif /* VOFA_H */
