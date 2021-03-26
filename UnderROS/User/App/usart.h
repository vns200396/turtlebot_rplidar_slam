#ifndef __USART_H
#define	__USART_H

#include "stm32f10x.h"




#define RCC_USART1_CLK					RCC_APB2Periph_USART1
#define RCC_USART1_Tx_CLK 			RCC_APB2Periph_GPIOA
#define RCC_USART1_Rx_CLK 			RCC_APB2Periph_GPIOA

#define USART1_Tx_Pin						GPIO_Pin_9
#define USART1_Tx_Port					GPIOA

#define USART1_Rx_Pin						GPIO_Pin_10
#define USART1_Rx_Port					GPIOA


#define RCC_USART2_CLK					RCC_APB1Periph_USART2
#define RCC_USART2_Tx_CLK 			RCC_APB2Periph_GPIOA
#define RCC_USART2_Rx_CLK 			RCC_APB2Periph_GPIOA

#define USART2_Tx_Pin						GPIO_Pin_2
#define USART2_Tx_Port					GPIOA

#define USART2_Rx_Pin						GPIO_Pin_3
#define USART2_Rx_Port					GPIOA


#define RCC_USART3_CLK					RCC_APB1Periph_USART3
#define RCC_USART3_Tx_CLK 			RCC_APB2Periph_GPIOB
#define RCC_USART3_Rx_CLK 			RCC_APB2Periph_GPIOB

#define USART3_Tx_Pin						GPIO_Pin_9
#define USART3_Tx_Port					GPIOB

#define USART3_Rx_Pin						GPIO_Pin_10
#define USART3_Rx_Port					GPIOB


#define USART1_Interrup 
//#define USART2_Interrup 
//#define USART3_Interrup 


#define USART1_SubPriority 			1
#define USART2_SubPriority 			2
#define USART3_SubPriority 			3

void Serial_Config(USART_TypeDef * USARTx, uint32_t baudrate);
void NVIC_USART_Init(void);
void USART_SendByte(USART_TypeDef* USARTx, uint8_t SendData);
void USART_PutString(USART_TypeDef* USARTx, uint8_t* s);
void USART_SendArray(USART_TypeDef* USARTx,uint8_t* str,uint32_t length);

#define USART1_SendByte(ch)					USART_SendByte(USART1,ch)
#define USART1_SendStr(str)					USART_PutString(USART1,str)
#define USART1_SendArr(str, len)		USART_SendArray(USART1,str, len)

#define USART2_SendByte(ch)					USART_SendByte(USART2,ch)
#define USART2_SendStr(str)					USART_PutString(USART2,str)
#define USART2_SendArr(str, len)		USART_SendArray(USART2,str, len)

#define USART3_SendByte(ch)					USART_SendByte(USART3,ch)
#define USART3_SendStr(str)					USART_PutString(USART3,str)
#define USART3_SendArr(str, len)		USART_SendArray(USART3,str, len)

#endif
