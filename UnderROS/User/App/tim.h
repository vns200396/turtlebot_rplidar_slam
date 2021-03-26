#ifndef __TIM_H
#define	__TIM_H

#include "stm32f10x.h"

#define MAX_PWM 239
#define MIN_PWM 127

void TIM_ResetCounter(TIM_TypeDef* TIMx);
void tim_init(void);
#endif /*__TIM_H*/ 
