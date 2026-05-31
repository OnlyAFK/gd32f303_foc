#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "gd32f30x.h"
#include "gd32f30x_it.h"
#include <stdbool.h>
#include <stdio.h>

void Error_Handler(void);
void delay_ms(const uint16_t ms);

#ifdef __cplusplus
}
#endif

#endif /* MAIN_H */
