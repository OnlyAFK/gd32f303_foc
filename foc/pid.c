#include "pid.h"

void pid_init(pid_t *pid, float kp, float ki, float kd, float ts, float out_max, float out_min) {
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->ts = ts;

  pid->ref = 0.0f;
  pid->fdb = 0.0f;
  pid->out = 0.0f;

  pid->integral = 0.0f;
  pid->last_error = 0.0f;

  pid->out_max = out_max;
  pid->out_min = out_min;
  pid->integral_max = out_max;
  pid->integral_min = out_min;

  pid->last_error_1 = 0.0f;
  pid->last_error_2 = 0.0f;
}

void pid_reset(pid_t *pid) {
  pid->integral = 0.0f;
  pid->last_error = 0.0f;
  pid->out = 0.0f;
}

static inline float clamp(float value, float min, float max) {
  if (value > max) return max;
  if (value < min) return min;
  return value;
}

float pi_calc(pid_t *pid, float ref, float fdb) {
  pid->ref = ref;
  pid->fdb = fdb;

  float error = ref - fdb;

  float p_out = pid->kp * error;

  pid->integral += pid->ki * error * pid->ts;
  pid->integral = clamp(pid->integral, pid->integral_min, pid->integral_max);

  pid->out = p_out + pid->integral;
  pid->out = clamp(pid->out, pid->out_min, pid->out_max);

  return pid->out;
}
