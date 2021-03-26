#include "led.h"

uint8_t flag_master_control=0;

void cmdLed(uint8_t _led)
{
	switch (_led)
	{
		case RED:
				LED_RED   = ON;
				LED_GREEN = OFF;
				LED_BLUE  = OFF;
				break;
		case GREEN:
				LED_RED   = OFF;
				LED_GREEN = ON;
				LED_BLUE  = OFF;
				break;
		case BLUE:
				LED_RED   = OFF;
				LED_GREEN = OFF;
				LED_BLUE  = ON;
				break;
		default:
				LED_RED   = OFF;
				LED_GREEN = OFF;
				LED_BLUE  = OFF;
				break;
	}
}

void set_fmc_led(void)
{
	flag_master_control =1;
}

void clear_fmc_led(void)
{
	flag_master_control =0;
}

void turnLed(uint8_t _led)
{
	if(!flag_master_control)
		cmdLed(_led);
}

void led_init(void)
{
	GPIO_InitTypeDef ledTypDef;
	
	RCC_APB2PeriphClockCmd( RCC_CLK_LED, ENABLE);
	
	ledTypDef.GPIO_Pin = LED_RED_Pin | LED_GREEN_Pin | LED_BLUE_Pin;
	ledTypDef.GPIO_Mode = GPIO_Mode_Out_PP;
	ledTypDef.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(LED_PORT, &ledTypDef);
	
	cmdLed(RED);
}

