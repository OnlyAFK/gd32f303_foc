#include "foc_math.h"
#include "arm_math.h"

void clarke_calc(clarke_t *c) {
  c->alpha = c->ia;
  c->beta  = FOC_1_SQRT3 * (c->ia + 2.0f * c->ib);
}

void park_calc(park_t *p) {
  float sin_val = arm_sin_f32(p->theta);
  float cos_val = arm_cos_f32(p->theta);

  p->d =  p->alpha * cos_val + p->beta * sin_val;
  p->q = -p->alpha * sin_val + p->beta * cos_val;
}

void inv_park_calc(inv_park_t *ip) {
  float sin_val = arm_sin_f32(ip->theta);
  float cos_val = arm_cos_f32(ip->theta);

  ip->alpha = ip->d * cos_val - ip->q * sin_val;
  ip->beta  = ip->d * sin_val + ip->q * cos_val;
}

void inv_clarke_calc(inv_clarke_t *ic) {
  float term_a_half = ic->alpha * 0.5f;
  float term_beta = ic->beta * FOC_SQRT3_2;

  ic->va = ic->alpha;
  ic->vb = -term_a_half + term_beta;
  ic->vc = -term_a_half - term_beta;
}
