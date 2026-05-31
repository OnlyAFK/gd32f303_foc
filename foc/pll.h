#ifndef PLL_H
#define PLL_H

#include <stdint.h>

typedef struct {
  float kp;
  float ki;

  float angle_est;
  float speed_est;

  float speed_rpm;
  float speed_rad_s;
} pll_t;

void pll_init(pll_t *pll, float kp, float ki);

void pll_reset(pll_t *pll);

float pll_update(pll_t *pll, float theta_meas, float dt);

static inline float pll_get_speed_rpm(pll_t *pll) {
  return pll->speed_rpm;
}

static inline float pll_get_speed_rad_s(pll_t *pll) {
  return pll->speed_rad_s;
}

static inline float pll_get_angle(pll_t *pll) {
  return pll->angle_est;
}

#endif /* PLL_H */
