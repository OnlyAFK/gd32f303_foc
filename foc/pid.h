#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef struct {
  float kp;
  float ki;          /* continuous-domain integral gain (the integrator scales by ts) */
  float kd;
  float ts;          /* loop sample period [s] — multiplies the integral each call */

  float ref;
  float fdb;
  float out;

  float integral;
  float last_error;

  float out_max;
  float out_min;
  float integral_max;
  float integral_min;

  float last_error_1;
  float last_error_2;
} pid_t;

void pid_init(pid_t *pid, float kp, float ki, float kd, float ts, float out_max, float out_min);

void pid_reset(pid_t *pid);

float pi_calc(pid_t *pid, float ref, float fdb);

#endif /* PID_H */
