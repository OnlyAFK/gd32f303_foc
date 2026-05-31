#ifndef FILTER_H
#define FILTER_H

typedef struct {
  float alpha;
  float out_prev;
} lpf_t;

void lpf_init(lpf_t *lpf, float alpha);

float lpf_apply(lpf_t *lpf, float input);

#endif /* FILTER_H */
