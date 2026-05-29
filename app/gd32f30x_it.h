#ifndef GD32F30X_IT_H
#define GD32F30X_IT_H

#include "gd32f30x.h"
#include <stdint.h>

extern volatile uint32_t SystickCount;

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);

#endif /* GD32F30X_IT_H */
