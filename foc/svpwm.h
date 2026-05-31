#ifndef SVPWM_H
#define SVPWM_H

#include <stdint.h>

typedef struct {
  float alpha;
  float beta;

  float udc;
  uint32_t ts;

  uint8_t sector;
  uint32_t ccr1;
  uint32_t ccr2;
  uint32_t ccr3;

  float t1;
  float t2;
} svpwm_t;

void svpwm_init(svpwm_t *svpwm, float udc, uint32_t ts);

void svpwm_calc(svpwm_t *svpwm);

#endif /* SVPWM_H */
