#ifndef FOC_MATH_H
#define FOC_MATH_H

#include <stdint.h>

/* Math constants — single source of truth for the project */
#define FOC_PI          3.14159265358979323846f
#define FOC_2PI         6.28318530717958647692f
#define FOC_PI_2        1.57079632679489661923f
#define FOC_SQRT3       1.73205080756887729353f
#define FOC_SQRT3_2     0.86602540378443864676f
#define FOC_1_SQRT3     0.57735026918962576451f
#define FOC_RAD_TO_RPM  9.549296585513720f
#define FOC_RPM_TO_RAD  0.104719755119660f

/* Clarke transform: abc → αβ */
typedef struct {
  float ia, ib, ic;
  float alpha, beta;
} clarke_t;

/* Park transform: αβ → dq */
typedef struct {
  float alpha, beta;
  float theta;
  float d, q;
} park_t;

/* Inverse Park transform: dq → αβ */
typedef struct {
  float d, q;
  float theta;
  float alpha, beta;
} inv_park_t;

/* Inverse Clarke transform: αβ → abc */
typedef struct {
  float alpha, beta;
  float va, vb, vc;
} inv_clarke_t;

void clarke_calc(clarke_t *c);

void park_calc(park_t *p);

void inv_park_calc(inv_park_t *ip);

void inv_clarke_calc(inv_clarke_t *ic);

#endif /* FOC_MATH_H */
