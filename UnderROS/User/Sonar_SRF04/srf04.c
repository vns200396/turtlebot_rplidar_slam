#include "srf04.h"



/* 1ms */
static void timersonar_init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_ICInitTypeDef  TIM_ICInitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	
  /* TIM3 clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	
	TIM_TimeBaseInitStructure.TIM_Period=0xffff;   
	TIM_TimeBaseInitStructure.TIM_Prescaler=71; 
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; 
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseInitStructure);
	
	TIM_OCInitTypeDef TIM_OCInitStruct;
	TIM_OCStructInit(&TIM_OCInitStruct);
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStruct.TIM_Pulse = 15; //us
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC4Init(TIM1, &TIM_OCInitStruct);
	
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x5;

  TIM_PWMIConfig(TIM1, &TIM_ICInitStructure);
	TIM_SelectInputTrigger(TIM1, TIM_TS_TI1FP1);
	TIM_SelectMasterSlaveMode(TIM1, TIM_MasterSlaveMode_Enable);


  TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);

  TIM_ARRPreloadConfig(TIM1, ENABLE);

  /* TIM3 enable counter */
  TIM_Cmd(TIM1, ENABLE);
  TIM_CtrlPWMOutputs(TIM1, ENABLE);	
	TIM_ITConfig(TIM1,TIM_IT_Update, ENABLE); 
	TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);  	
  NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void sonar_srf04_init(void)
{
	GPIO_InitTypeDef sonar_typdef;

	RCC_APB2PeriphClockCmd(RCC_SONAR_CLK, ENABLE);
	
	/* Config trigger pin */
	sonar_typdef.GPIO_Pin = TRIGGER_Pin;
	sonar_typdef.GPIO_Mode = GPIO_Mode_AF_PP;
	sonar_typdef.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SONAR_PORT, &sonar_typdef);

	/* Config echo pin */
	sonar_typdef.GPIO_Pin = ECHO_Pin;
	sonar_typdef.GPIO_Mode = GPIO_Mode_IPD;
	sonar_typdef.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SONAR_PORT, &sonar_typdef);
	
	/* Init interrup */
	timersonar_init();
}


uint16_t timer1_uS=0;
uint8_t ping_event=0;
volatile uint16_t distance;
void TIM1_UP_IRQHandler(void)
{
	
	if(TIM_GetITStatus(TIM1,TIM_IT_Update)) 
	{
		TIM_Cmd(TIM1, DISABLE);
		ping_event =2;
		timer1_uS = TIM_GetCapture2(TIM1)-TIM_GetCapture1(TIM1);
		distance = (uint16_t )(timer1_uS/2/29.412);
	}
	TIM_ClearITPendingBit(TIM1,TIM_IT_Update);
}


void ping_obstacle(void)
{
	static uint32_t timedly=0;
	switch (ping_event)
	{
		case 0:
			ping_event++;
			timedly= millis();
			TIM_Cmd(TIM1, ENABLE);
			break;
		case 1:
			// wait rely
			if(millis() - timedly > 40)
			{
				timedly=0;
				ping_event =0;
				timedly =0;
			}
			break;
		case 2:
			if(!timedly)
				timedly = millis();
			
			if(millis() - timedly > 10)
			{
				timedly=0;
				ping_event =0;
				timedly =0;
			}
			break;
		default:
			ping_event=0;
		  break;
		
	}
}


uint16_t getDistance(void)
{
		return distance;
}

uint16_t getTimeSonar(void)
{
	return timer1_uS;
}
