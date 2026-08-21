#include "vofa.h"
#include <string.h>
#include "uart.h"

// VOFA+ "justfloat" frame: N little-endian floats followed by the tail bytes
// 00 00 80 7F. The MCU is little-endian, so the floats are copied verbatim.
static const uint8_t vofa_tail[4] = {0x00, 0x00, 0x80, 0x7F};

float vofa_data[VOFA_CHANNEL_CNT] = {0};

void vofa_send_justfloat(float *data_ptr, uint8_t count) {
  uart_write((uint8_t *)data_ptr, (uint32_t)count * sizeof(float));
  uart_write(vofa_tail, sizeof(vofa_tail));
}

uint32_t vofa_build_justfloat(const float *data, uint8_t count,
                              uint8_t *out, uint32_t out_size) {
  uint32_t payload = (uint32_t)count * sizeof(float);
  uint32_t total = payload + sizeof(vofa_tail);
  if (out == 0 || out_size < total) {
    return 0;
  }
  memcpy(out, data, payload);
  memcpy(out + payload, vofa_tail, sizeof(vofa_tail));
  return total;
}

void vofa_update_from_motor(motor_t *motor) {
  vofa_data[VOFA_CH_VD] = motor->xform.inv_park.d;
  vofa_data[VOFA_CH_VQ] = motor->xform.inv_park.q;

  vofa_data[VOFA_CH_MECH_THETA] = motor->sensor.encoder.mech_angle;

  vofa_data[VOFA_CH_SVPWM_ALPHA] = motor->xform.svpwm.alpha;//motor->sensor.encoder.elec_angle;
  vofa_data[VOFA_CH_SVPWM_BETA] = motor->xform.svpwm.beta;
  vofa_data[VOFA_CH_CCR1] = (float)motor->xform.svpwm.ccr1;
  vofa_data[VOFA_CH_CCR2] = (float)motor->xform.svpwm.ccr2;
  vofa_data[VOFA_CH_CCR3] = (float)motor->xform.svpwm.ccr3;
  vofa_data[VOFA_CH_SECTOR] = (float)motor->xform.svpwm.sector;

  vofa_data[VOFA_CH_IU] = motor->sensor.currents.iu;
  vofa_data[VOFA_CH_IV] = motor->sensor.currents.iv;
  vofa_data[VOFA_CH_IW] = motor->sensor.currents.iw;

  vofa_data[VOFA_CH_I_ALPHA] = motor->xform.clarke.alpha;
  vofa_data[VOFA_CH_I_BETA] = motor->xform.clarke.beta;

  vofa_data[VOFA_CH_ID] = motor->actual.id;
  vofa_data[VOFA_CH_IQ] = motor->actual.iq;

  vofa_data[VOFA_CH_ID_REF] = motor->target.id;
  vofa_data[VOFA_CH_IQ_REF] = motor->target.iq;

  vofa_data[VOFA_CH_POS_TARGET] = motor->ctrl.pos.target_pos;
  vofa_data[VOFA_CH_POS_ACTUAL] = motor->ctrl.pos.current_pos;

  // Diagnostic: commanded open-loop electrical angle, to compare against the
  // encoder-derived elec_angle (currently on VOFA_CH_SVPWM_ALPHA).
  vofa_data[VOFA_CH_DEBUG] = motor->openloop_angle;

  vofa_data[VOFA_CH_SPEED_ACTUAL] = motor->actual.rpm;
  vofa_data[VOFA_CH_SPEED_TARGET] = motor->target.rpm;

  vofa_data[VOFA_CH_IQ_FF] = motor->iq_ff;
}
