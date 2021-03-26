#ifndef __COMM_H
#define __COMM_H

#include "mylib.h"



#define Comm											USART1
#define Comm_IRQHandler 					USART1_IRQHandler
#define Comm_SendByte(ch)					USART1_SendByte(ch)
#define Comm_SendStr(str)					USART1_SendStr(str)
#define Comm_SendArr(arr, len)		USART1_SendArr(arr, len)

#define RingBufferSize						128

#define Rasp_Start_SMS		0xAA
#define Driver_Start_SMS 	0x55

#define Rasp_End_SMS			Driver_Start_SMS
#define Driver_End_SMS		Rasp_Start_SMS

#define LengSMS						11
#define LengPayLoad							(uint8_t )8

#define CMD_FAULT_CLEAR							0x00
#define CMD_SET_SST_DRIVER1					0x01
#define CMD_SET_SST_DRIVER2					0x02
#define CMD_REQUEST_ACC_IMU							0x03
#define CMD_REQUEST_SONAR_ENC				0x04
#define CMD_RP_PIDLEFT							0x05
#define CMD_RP_PIDRIGHT									0x06
#define CMD_REQUSET_INFORDRIVER1				0x07
#define CMD_REQUSET_INFORDRIVER2				0x08
#define CMD_RESET_ENCODER						0x09
#define CMD_SET_PID									0x0A
#define CMD_RESET_IMU					  		0x0B
#define CMD_SET_LED									0x0C




void comm_init(void);
void comm_parse(void);
void driverResponseSttDriver1(void);
#endif /* __COMM_H */
