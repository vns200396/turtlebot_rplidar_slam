#ifndef __SRF04_H
#define __SRF04_H

#include "mylib.h"


#define RCC_SONAR_CLK				RCC_APB2Periph_GPIOA
#define TRIGGER_Pin					GPIO_Pin_11
#define ECHO_Pin						GPIO_Pin_8
#define SONAR_PORT    			GPIOA

#define TRIGGER_SONAR				PAout(11)
#define ECHO_SONAR					PAin(8)

#define START_TIME RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 , ENABLE);TIM_Cmd(TIM1, ENABLE)
#define STOP_TIME  TIM_Cmd(TIM1, DISABLE);RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 , DISABLE)

void sonar_srf04_init(void);
void ping_obstacle(void);
uint16_t getDistance(void);
uint16_t getTimeSonar(void);
#endif
