#ifndef __MAIN_H__
#define __MAIN_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "gd32f30x.h"
#include "gd32f30x_it.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

//#define __DEBUG__

#ifdef __DEBUG__
// clang-format off
    #include "SEGGER_RTT.h"
    #define DEBUG(format, ...) SEGGER_RTT_printf(0, format, ##__VA_ARGS__);
    inline void DEBUG_PLOT(float f1, float f2) { float value[2] = {f1, f2}; SEGGER_RTT_Write(1, &value, sizeof(value)); }
// clang-format on
#else
#define DEBUG(format, ...)
#define DEBUG_PLOT(value)
#endif


static inline uint32_t get_ms_since(uint32_t tick)
{
    return (uint32_t) ((SystickCount - tick) / 2U);
}

void Error_Handler(void);
void delay_ms(const uint16_t ms);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
