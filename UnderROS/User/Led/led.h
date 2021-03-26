#ifndef __LED_H
#define __LED_H

#include "stm32f10x.h"
#include "system.h"

#define RCC_CLK_LED			RCC_APB2Periph_GPIOC

#define LED_PORT				GPIOC
#define LED_RED_Pin			GPIO_Pin_15
#define LED_GREEN_Pin		GPIO_Pin_14
#define LED_BLUE_Pin		GPIO_Pin_13

#define LED_RED					PCout(15)
#define LED_GREEN       PCout(14)
#define LED_BLUE        PCout(13)

#define RED							0x01
#define GREEN						0x02
#define BLUE						0x03

#define ON	   0x00
#define OFF	   0x01

void cmdLed(uint8_t _led);
void set_fmc_led(void);
void clear_fmc_led(void);
void turnLed(uint8_t _led);
void led_init(void);
#endif
