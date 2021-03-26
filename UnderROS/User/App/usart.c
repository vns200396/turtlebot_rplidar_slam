#include "usart.h"



void NVIC_USART_Init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	#ifdef  VECT_TAB_RAM  
	  /* Set the Vector Table base location at 0x20000000 */ 
	  NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0); 
	#else  /* VECT_TAB_FLASH  */
	  /* Set the Vector Table base location at 0x08000000 */ 
	  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);   
	#endif

	/* Configure one bit for preemption priority */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

#ifdef USART1_Interrup
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	NVIC_Init(&NVIC_InitStructure);
#endif /* USART1_Interrup */
	
#ifdef USART2_Interrup
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2; 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	NVIC_Init(&NVIC_InitStructure);
#endif /* USART2_Interrup */

#ifdef USART3_Interrup
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3; 
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
#endif /* USART3_Interrup */
}


void Serial_Config(USART_TypeDef * USARTx, uint32_t baudrate)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	
	NVIC_USART_Init();
	
	if(USARTx == USART1)
	{
		RCC_APB2PeriphClockCmd(RCC_USART1_Tx_CLK | RCC_USART1_Rx_CLK | RCC_USART1_CLK,ENABLE);
		
		GPIO_InitStructure.GPIO_Pin = USART1_Tx_Pin;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(USART1_Tx_Port, &GPIO_InitStructure);    
		
		GPIO_InitStructure.GPIO_Pin = USART1_Rx_Pin;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_Init(USART1_Rx_Port, &GPIO_InitStructure);   
			
		
		USART_InitStructure.USART_BaudRate = baudrate;
		USART_InitStructure.USART_WordLength=USART_WordLength_8b;
		USART_InitStructure.USART_StopBits=USART_StopBits_1;
		USART_InitStructure.USART_Parity=USART_Parity_No;
		USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
		USART_InitStructure.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;

		USART_Init(USART1,&USART_InitStructure);
		USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
		USART_Cmd(USART1,ENABLE);
	}
	else if (USARTx == USART2)
	{
		RCC_APB1PeriphClockCmd(RCC_USART2_CLK, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_USART2_Tx_CLK | RCC_USART2_Rx_CLK, ENABLE); //


		GPIO_InitStructure.GPIO_Pin=USART2_Tx_Pin;//TX
		GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
		GPIO_Init(USART2_Tx_Port,&GPIO_InitStructure);

		GPIO_InitStructure.GPIO_Pin=USART2_Rx_Pin;//RX
		GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;
		GPIO_Init(USART2_Rx_Port,&GPIO_InitStructure);
			
		/* USART2 */
		USART_InitStructure.USART_BaudRate=baudrate;
		USART_InitStructure.USART_WordLength=USART_WordLength_8b;
		USART_InitStructure.USART_StopBits=USART_StopBits_1;
		USART_InitStructure.USART_Parity=USART_Parity_No;
		USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
		USART_InitStructure.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;

		USART_Init(USART2,&USART_InitStructure);
		USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
		USART_Cmd(USART2,ENABLE);
	}
	else
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

		/* Congfig RX TX*/  
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOB, &GPIO_InitStructure);    
		
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_Init(GPIOB, &GPIO_InitStructure);  
			
		/* USART3 */
		USART_InitStructure.USART_BaudRate = baudrate;	
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;	
		USART_InitStructure.USART_StopBits = USART_StopBits_1; 	
		USART_InitStructure.USART_Parity = USART_Parity_No ; 
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
		
		USART_Init(USART3, &USART_InitStructure);  
		USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	  USART_ClearITPendingBit(USART3, USART_IT_RXNE);
		USART_Cmd(USART3, ENABLE);// USART3
	}
}


void USART_SendByte(USART_TypeDef * USARTx, uint8_t SendData)
{	   
	while(USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET);	
	USART_SendData(USARTx,SendData);    
}  

void USART_PutString(USART_TypeDef * USARTx, uint8_t* s)
{
	// Send a string
	while (*s)
	{
		USART_SendByte(USARTx, *s++);
	}
}

void USART_SendArray(USART_TypeDef* USARTx,uint8_t* str,uint32_t length)
{
	uint32_t i;
	
	for(i=0; i < length; i++)
	{
		USART_SendByte(USARTx,str[i]);
	}
}



