#ifndef _SysTick_H
#define _SysTick_H

#include "system.h"





void SysTick_Init(void);
void delay_ms(__IO u32 nTime);
void delay_us(u32 nus);
void TimingDelay_Crement(void);
uint64_t millis(void);

#endif
