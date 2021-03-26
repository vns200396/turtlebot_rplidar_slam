/*********************************************************************************************************
*
* File                : main.c
*
*********************************************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "comm.h"
#include "led.h"
#include "adxl345.h"
#include "srf04.h"
#include "motor.h"


int main(void)
{
	SystemInit();
	SysTick_Init();
	led_init();
	sonar_srf04_init();
	comm_init();
	motor_init();
  while (1)
	{	
		comm_parse();
		adxl345_parse();
		runtimeDriver();
		ping_obstacle();
  }
}

