#include "gd32f30x_it.h"
#include "main.h"

void NMI_Handler(void)          { Error_Handler(); }
void HardFault_Handler(void)    { Error_Handler(); }
void MemManage_Handler(void)    { Error_Handler(); }
void BusFault_Handler(void)     { Error_Handler(); }
void UsageFault_Handler(void)   { Error_Handler(); }
void SVC_Handler(void)          { Error_Handler(); }
void DebugMon_Handler(void)     { Error_Handler(); }
void PendSV_Handler(void)       { Error_Handler(); }
void SysTick_Handler(void)      { }


