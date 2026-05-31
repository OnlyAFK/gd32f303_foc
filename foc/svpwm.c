#include "svpwm.h"
#include "foc_math.h"

void svpwm_init(svpwm_t *svpwm, float udc, uint32_t ts) {
  svpwm->udc = udc;
  svpwm->ts = ts;
  svpwm->alpha = 0.0f;
  svpwm->beta = 0.0f;
  svpwm->sector = 0;
  svpwm->ccr1 = ts / 2;
  svpwm->ccr2 = ts / 2;
  svpwm->ccr3 = ts / 2;
}

void svpwm_calc(svpwm_t *svpwm) {
  float alpha = svpwm->alpha;
  float beta = svpwm->beta;
  float ts = (float)svpwm->ts;
  float udc = svpwm->udc;

  /* Determine sector (1..6) via sign of three test vectors */
  float a = beta;
  float b = FOC_SQRT3_2 * alpha - 0.5f * beta;
  float c = -FOC_SQRT3_2 * alpha - 0.5f * beta;

  uint8_t sector = 0;
  if (a > 0.0f) sector += 1;
  if (b > 0.0f) sector += 2;
  if (c > 0.0f) sector += 4;
  svpwm->sector = sector;

  /* Compute vector time bases */
  float temp = (FOC_SQRT3 * ts) / udc;
  float x = temp * beta;
  float y = temp * (FOC_SQRT3_2 * alpha + 0.5f * beta);
  float z = temp * (-FOC_SQRT3_2 * alpha + 0.5f * beta);

  /* Map to t1, t2 per sector */
  float t1, t2;
  switch (sector) {
  case 3: t1 = -z; t2 =  x; break;
  case 1: t1 =  z; t2 =  y; break;
  case 5: t1 =  x; t2 = -y; break;
  case 4: t1 = -x; t2 =  z; break;
  case 6: t1 = -y; t2 = -z; break;
  case 2: t1 =  y; t2 = -x; break;
  default: t1 = 0.0f; t2 = 0.0f; break;
  }

  svpwm->t1 = t1;
  svpwm->t2 = t2;

  /* Saturate if overmodulation */
  if ((t1 + t2) > ts) {
    float ratio = ts / (t1 + t2);
    t1 *= ratio;
    t2 *= ratio;
  }

  /* Convert to phase duty cycles */
  float ta = (ts + t1 + t2) * 0.5f;
  float tb = (ts - t1 + t2) * 0.5f;
  float tc = (ts - t1 - t2) * 0.5f;

  /* Assign CCR per sector */
  switch (sector) {
  case 3: svpwm->ccr1 = (uint32_t)ta; svpwm->ccr2 = (uint32_t)tb; svpwm->ccr3 = (uint32_t)tc; break;
  case 1: svpwm->ccr1 = (uint32_t)tb; svpwm->ccr2 = (uint32_t)ta; svpwm->ccr3 = (uint32_t)tc; break;
  case 5: svpwm->ccr1 = (uint32_t)tc; svpwm->ccr2 = (uint32_t)ta; svpwm->ccr3 = (uint32_t)tb; break;
  case 4: svpwm->ccr1 = (uint32_t)tc; svpwm->ccr2 = (uint32_t)tb; svpwm->ccr3 = (uint32_t)ta; break;
  case 6: svpwm->ccr1 = (uint32_t)tb; svpwm->ccr2 = (uint32_t)tc; svpwm->ccr3 = (uint32_t)ta; break;
  case 2: svpwm->ccr1 = (uint32_t)ta; svpwm->ccr2 = (uint32_t)tc; svpwm->ccr3 = (uint32_t)tb; break;
  default:
    svpwm->ccr1 = (uint32_t)(ts * 0.5f);
    svpwm->ccr2 = (uint32_t)(ts * 0.5f);
    svpwm->ccr3 = (uint32_t)(ts * 0.5f);
    break;
  }
}
