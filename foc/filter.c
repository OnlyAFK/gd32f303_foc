#include "filter.h"

void lpf_init(lpf_t *lpf, float alpha) {
  if (alpha > 1.0f) alpha = 1.0f;
  if (alpha < 0.0f) alpha = 0.0f;

  lpf->alpha = alpha;
  lpf->out_prev = 0.0f;
}

float lpf_apply(lpf_t *lpf, float input) {
  float output = (lpf->alpha * input) + ((1.0f - lpf->alpha) * lpf->out_prev);
  lpf->out_prev = output;
  return output;
}
