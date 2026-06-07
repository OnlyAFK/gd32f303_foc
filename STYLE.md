# VESC FOC Firmware — Coding Standard

Version: 1.0 | 2026-05-23

---

## 1. Naming Conventions

### Types (struct, union, enum)

Lowercase, snake_case, `_t` suffix.

```c
// correct
typedef struct { ... } motor_t;
typedef struct { ... } pid_t;
typedef enum { ... } foc_mode_t;

// wrong
typedef struct { ... } Motor_t;       // no PascalCase
typedef struct { ... } PIDController; // no PascalCase, no _t
```

### Functions

Lowercase, snake_case, `<module>_<action>` prefix.

```c
// correct
void foc_init(motor_t *motor);
void foc_control_loop(motor_t *motor);
float pi_calc(pid_t *pid, float ref, float fdb);
void svpwm_calc(svpwm_t *svpwm);

// wrong
void FOC_Init(motor_t *motor);        // no PascalCase
void focControlLoop(motor_t *motor);   // no camelCase
```

### Variables (local, static, struct members)

Lowercase, snake_case.

```c
// correct
float target_rpm;
uint32_t pwm_period;
uint8_t speed_loop_cnt;
int32_t total_turns;

// wrong
float TargetRPM;       // no PascalCase
uint32_t PwmPeriod;    // no PascalCase
```

### Macros and Constants

UPPERCASE, SNAKE_CASE.

```c
// correct
#define FOC_2PI            6.28318530718f
#define HW_PWM_PERIOD       4200u
#define CALIBRATION_SAMPLES 1000u

// wrong
#define Foc_2pi   6.28318530718f      // wrong case
#define hwPwmPeriod 4200              // wrong case
```

### Enum Values

UPPERCASE, SNAKE_CASE.

```c
// correct
typedef enum {
  FOC_MODE_IDLE,
  FOC_MODE_CURRENT,
  FOC_MODE_SPEED,
  FOC_MODE_POSITION,
} foc_mode_t;

// wrong
typedef enum {
  focModeIdle,         // no camelCase
  Foc_Mode_Current,    // no mixed case
} foc_mode_t;
```

---

## 2. Formatting

### Braces — K&R style (opening brace on same line)

```c
// correct
void foc_control_loop(motor_t *motor) {
  if (motor->mode == FOC_MODE_IDLE) {
    vd_out = 0.0f;
    vq_out = 0.0f;
  } else {
    vd_out = pi_calc(&motor->ctrl.id, motor->target.id, motor->feedback.id);
  }
  for (int i = 0; i < 3; i++) {
    do_thing(i);
  }
}

// wrong
void foc_control_loop(motor_t *motor)
{                              // brace on its own line
  if (motor->mode == FOC_MODE_IDLE)
  {                            // brace on its own line
    vd_out = 0.0f;
  }
}
```

Rule: **always** use braces, even for single-line bodies. No exceptions.

```c
// correct
if (error) {
  return -1;
}

// wrong
if (error) return -1;       // no brace
if (error)
  return -1;                // no brace
```

### Indentation — 2 spaces, no tabs

```c
void example(void) {
  if (cond) {
    do_thing();              // 4 spaces = 2 levels
  }
}
```

### Line length

100 characters soft limit. 120 hard limit.

### Spacing

- One space after `if`, `for`, `while`, `switch`
- No space between function name and `(`
- Spaces around binary operators
- One space after comma

```c
// correct
if (error > 0) {
  float x = a + b;
  pid_init(&pid, kp, ki, kd, ts, out_max, out_min);
}

// wrong
if(error>0){                        // missing spaces
  float x=a+b;                      // missing spaces
  pid_init(&pid,kp,ki,kd,ts,out_max,out_min);  // missing spaces after commas
}
```

### Switch statements

```c
switch (sector) {
case 1:
  t1 = z;
  t2 = y;
  break;
case 2:
  t1 = y;
  t2 = -x;
  break;
default:
  t1 = 0.0f;
  t2 = 0.0f;
  break;
}
```

---

## 3. Comments

### Language

**English only.** Chinese comments cause encoding conflicts between Keil (GB2312) and other editors (UTF-8).

### What to comment

- **Why**, not what. The code already says what.
- Non-obvious algorithms (e.g., "sector = (A>0) + 2*(B>0) + 4*(C>0)" warrants a brief explanation)
- Hardware workarounds and silicon errata
- Assumptions that could break (e.g., "Udc must be > 0 before calling")

### What NOT to comment

- Obvious mechanics ("increment counter", "set pin high")
- Function name re-statements (if your function is `pid_reset`, don't comment `// reset PID`)
- Changelogs, authorship, dates (use git for that)

### Format

```c
// Single-line comment for brief notes.

/*
 * Multi-line comment for longer explanations.
 * Use this sparingly.
 */
```

No `@brief` / `@param` / `@note` doxygen blocks. Keep comments lightweight.

---

## 4. File Organization

### Header files

Every `.c` has a corresponding `.h`. The header exposes only the **public API** — types and functions that other modules need. Everything else stays in `.c` as `static`.

```c
// example_module.h
#ifndef EXAMPLE_MODULE_H     // include guard: FILENAME_H
#define EXAMPLE_MODULE_H

#include <stdint.h>

// public types
typedef struct { ... } example_t;

// public functions
void example_init(example_t *obj);
float example_calc(example_t *obj, float input);

#endif
```

### Include guards

Format: `FILENAME_H` (uppercase filename, underscore, H).

```c
#ifndef MOTOR_HW_H
#define MOTOR_HW_H
...
#endif /* MOTOR_HW_H */   // trailing comment optional
```

### Include order

```c
// 1. Corresponding header (in .c file)
#include "example_module.h"

// 2. Standard library
#include <stdint.h>
#include <math.h>

// 3. Project headers — dependency order (low-level first)
#include "pid.h"
#include "foc_math.h"
#include "svpwm.h"
#include "motor_hw.h"
#include "foc_core.h"
```

### Avoid exposing internal structs in headers

When a module's internals don't need to be visible, use an opaque pointer:

```c
// foo.h — header: forward declaration only
typedef struct foo_t foo_t;

foo_t *foo_alloc(void);
void foo_do_thing(foo_t *f);

// foo.c — source: full definition
struct foo_t {
  int private_data;
};
```

For embedded code where dynamic allocation is banned, it's acceptable to define the struct in the header *but document that its members are private* and must not be accessed directly by other modules.

---

## 5. Module Design

### Single responsibility

Each module does one thing:

| Module | Responsibility |
|--------|---------------|
| `pid` | PID/PI calculation, nothing else |
| `filter` | Digital low-pass filter |
| `foc_math` | Clarke/Park transforms, math constants |
| `svpwm` | Space-vector PWM modulation |
| `pll` | Phase-locked loop speed estimation |
| `motor_hw` | STM32 hardware abstraction (GPIO, ADC, TIM, SPI, encoder) |
| `foc_core` | FOC state machine, cascaded loop orchestration |
| `interrupt` | ISR entry point, control loop scheduling |
| `vofa` | Telemetry data formatting and transmission |
| `usb_vcp` | USB virtual COM port transport |

### Data ownership

- Each module **owns** its data. Other modules read/write through public API functions, never through direct struct member access.
- A module may read from another module's public struct fields **only if** those fields are documented as read-only public.
- No module may write to another module's data directly.

### No global variables in headers

```c
// correct — in .h: extern declaration with explicit "this is the global instance"
extern const motor_config_t g_motor_config;  // read-only config, OK

// wrong — in .h: mutable global
extern motor_t g_motor;   // don't do this
extern float vofa_data[]; // don't do this
```

Instead: declare the instance in `main.c` and pass it by pointer.

---

## 6. Constants and Configuration

### No magic numbers

All numeric literals beyond `0`, `1`, `-1` must be named.

```c
// correct
#define HW_PWM_PERIOD   4200u
#define HW_DEADTIME_NS  1200u
pwm_set_period(HW_PWM_PERIOD);

// wrong
pwm_set_period(4200);    // what is 4200?
```

### Where constants live

- **Math constants** → `foc_math.h` (PI, 2PI, SQRT3, etc.)
- **Hardware parameters** → `motor_hw.h` (PWM period, ADC scale, pole pairs, encoder CPR)
- **Control defaults** → each module's `.c` file as `static const` or `#define` near the top
- **Tunable parameters** → eventually a `config.h` with runtime-accessible struct (not #define)

### Physical units

All FOC calculations use SI units internally:

| Quantity | Unit | Notes |
|----------|------|-------|
| Voltage | V | |
| Current | A | |
| Angle | rad | [0, 2π) |
| Angular velocity | rad/s | internal; convert to RPM only at API boundary |
| Speed (reported) | RPM | human-facing only |
| Time | s | |
| Frequency | Hz | |

Conversions must use named macros, never raw numbers:

```c
#define RAD_S_TO_RPM    9.54929658551f   // 60/(2π)
#define RPM_TO_RAD_S    0.10471975512f   // 2π/60
```

---

## 7. Embedded Constraints

### No dynamic allocation

`malloc` / `free` / `calloc` are forbidden. All memory is static or stack.

### No VLAs (Variable-Length Arrays)

```c
// wrong
void foo(int n) {
  float buffer[n];         // VLA — stack overflow risk
}

// correct
#define BUFFER_SIZE 32
void foo(void) {
  float buffer[BUFFER_SIZE];
}
```

### ISR rules

- ISRs do **scheduling only**: set flags, enqueue data, trigger the main control loop
- No floating-point math in ISRs (unless FPU context save is confirmed enabled)
- No blocking calls (`HAL_Delay`, `printf`, semaphore wait)
- Keep ISRs under ~20 lines

### Stack safety

- No large local arrays (>256 bytes) in functions that may be called from ISR context
- No recursion

---

## 8. Error Handling

This firmware uses a simple status pattern:

```c
typedef enum {
  MOTOR_STATE_IDLE,
  MOTOR_STATE_CALIBRATING,
  MOTOR_STATE_RUNNING,
  MOTOR_STATE_ERROR,
} motor_state_t;
```

- Fatal errors: transition to `MOTOR_STATE_ERROR`, disable driver, set PWM to brake
- Non-fatal errors: ignore or clamp (e.g., current limit via PID output clamp)
- No exception-style error propagation — embedded targets don't need it
- `assert()` is acceptable for "this should never happen" conditions during development

---

## 9. Git and Project Hygiene

- No generated files in version control (`*.o`, `*.hex`, `*.bin`, `*.elf`, build output)
- No IDE-specific user preference files (already in `.gitignore`)
- Commit messages in English, imperative mood ("add current limit", not "added current limit")
- One logical change per commit

---

## 10. Quick Reference

| Element | Rule | Example |
|---------|------|---------|
| struct/enum type | `_t` suffix, lowercase | `pid_t`, `motor_t` |
| function | `module_action`, lowercase | `foc_init()`, `svpwm_calc()` |
| variable | lowercase, snake_case | `target_rpm`, `pwm_period` |
| macro/constant | UPPERCASE | `FOC_2PI`, `HW_PWM_PERIOD` |
| enum value | UPPERCASE | `FOC_MODE_IDLE` |
| braces | K&R (same line) | `if (x) {` |
| indent | 2 spaces, no tabs | |
| comments | English only, why not what | |
| include guard | `FILENAME_H` | `#ifndef PID_H` |
| malloc | forbidden | |
| globals in .h | forbidden | |
| magic numbers | forbidden | |
| units internally | SI (V, A, rad, s) | |
