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

/* One-shot full commissioning sequence (the `autocal` shell command): current-
 * offset cal + encoder align -> Rs/Ld/Lq/flux ident -> deploy current gains ->
 * J/B/Tc ident -> deploy speed gains, with settle delays between stages. Driven
 * from the main loop; watch the control state to follow progress. */
void autocal_start(void);
void autocal_abort(void);
bool autocal_active(void);

#ifdef __cplusplus
}
#endif

#endif /* MAIN_H */
