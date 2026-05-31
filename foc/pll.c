#include "pll.h"
#include "foc_math.h"

static inline float normalize_angle(float angle) {
  while (angle >= FOC_2PI) angle -= FOC_2PI;
  while (angle < 0.0f)   angle += FOC_2PI;
  return angle;
}

static inline float normalize_angle_error(float error) {
  if (error > FOC_PI)  error -= FOC_2PI;
  if (error < -FOC_PI) error += FOC_2PI;
  return error;
}

void pll_init(pll_t *pll, float kp, float ki) {
  pll->kp = kp;
  pll->ki = ki;

  pll->angle_est = 0.0f;
  pll->speed_est = 0.0f;
  pll->speed_rpm = 0.0f;
  pll->speed_rad_s = 0.0f;
}

void pll_reset(pll_t *pll) {
  pll->angle_est = 0.0f;
  pll->speed_est = 0.0f;
  pll->speed_rpm = 0.0f;
  pll->speed_rad_s = 0.0f;
}

float pll_update(pll_t *pll, float theta_meas, float dt) {
  float angle_err = theta_meas - pll->angle_est;
  angle_err = normalize_angle_error(angle_err);

  pll->speed_est += pll->ki * angle_err * dt;
  pll->angle_est += (pll->speed_est + pll->kp * angle_err) * dt;
  pll->angle_est = normalize_angle(pll->angle_est);

  pll->speed_rad_s = pll->speed_est;
  pll->speed_rpm = pll->speed_est * FOC_RAD_TO_RPM;

  return pll->speed_rad_s;
}
