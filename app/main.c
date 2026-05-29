#include "main.h"

int main(void)
{
    while (1) {
        /* TODO */
    }
}

void Error_Handler(void)
{
    __disable_irq();
    timer_primary_output_config(TIMER0, DISABLE);
    while (1) {}
}

void delay_ms(const uint16_t ms)
{
    volatile uint32_t i = ms * 17200U;
    while (i-- > 0U) { __NOP(); }
}
