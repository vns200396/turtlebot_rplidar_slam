#include	"comm.h"
#include "ringbuf.h"
#include "motor.h"
#include "adxl345.h"
#include "srf04.h"


struct ringbuf commbuff;
uint8_t* commbuffer;
uint8_t* datSMS;

void comm_init(void)
{
	Serial_Config(Comm, 115200);
	commbuffer = (uint8_t *)malloc(RingBufferSize*sizeof(uint8_t));
	datSMS = (uint8_t *)malloc(RingBufferSize*sizeof(uint8_t));
	ringbuf_init(&commbuff, commbuffer, RingBufferSize);
}

void	Comm_IRQHandler(void)
{
	uint8_t c;
	
	if(USART_GetITStatus(Comm, USART_IT_RXNE) != RESET)
	{	
		USART_ClearITPendingBit(Comm, USART_IT_RXNE);
		
		c= USART_ReceiveData(Comm);
		
		ringbuf_put(&commbuff, c);
	}
}

/**
  * @brief  Calculate CRC checksum
  * @param  Pointer to payload
  * @param  Length payload
  * @retval Result CRC
  */
unsigned int CRC_Verify(unsigned char* cBuffer, unsigned int iBufLen)
{
	unsigned int i, j; //#define wPolynom 0xA001
	unsigned int wCrc = 0xffff;
	unsigned int wPolynom = 0xA001;
	/*---------------------------------------------------------------------------------*/
	for (i = 0; i < iBufLen; i++)
	{
		wCrc ^= cBuffer[i];
		for (j = 0; j < 8; j++)
		{
			if (wCrc &0x0001)
			{ wCrc = (wCrc >> 1) ^ wPolynom; }
			else
			{ wCrc = wCrc >> 1; }
		}
	}
	return wCrc;
}


void driverResponseFaulClear(void)
{
	uint8_t* payload;
	uint16_t crc;

	payload = (uint8_t* )malloc(LengSMS*sizeof(uint8_t));
	
	// response infor driver
	payload[0] = CMD_FAULT_CLEAR;
	payload[1] = 0;
	payload[2] = 0;
	payload[3] = 0;
	payload[4] = 0;
	payload[5] = 0;
	payload[6] = 0;
	payload[7] = 0;
	payload[8] = 0;
	
	crc = CRC_Verify(payload , LengPayLoad+1);
	payload[9] = (uint8_t)(crc&0xFF);
	payload[10] = (uint8_t)((crc&0xFF00)>>8);
	
	Comm_SendByte(Driver_Start_SMS);
	Comm_SendArr(payload, LengSMS);
	Comm_SendByte(Driver_End_SMS);

	free(payload);
}

void driverResponseSttDriver1(void)
{
	uint8_t* payload;
	uint16_t crc;
	uint16_t encleft;
	uint16_t encright;
	
	payload = (uint8_t* )malloc(LengSMS*sizeof(uint8_t));
	
	// response infor driver
	payload[0] = CMD_SET_SST_DRIVER1;
	payload[1] = getStatusIN(A);
	payload[2] = getStatusIN(B);
	payload[3] = getStatusIN(C);
	payload[4] = getStatusIN(D);
	
	encleft = getEncoder(ENCODERL);
	encright = getEncoder(ENCODERR);
	
	payload[5] = (uint8_t)(encleft&0xFF);
	payload[6] = (uint8_t)((encleft&0xFF00)>>8);
	payload[7] = (uint8_t)(encright&0xFF);
	payload[8] = (uint8_t)((encright&0xFF00)>>8);
	
	crc = CRC_Verify(payload , LengPayLoad+1);
	payload[9] = (uint8_t)(crc&0xFF);
	payload[10] = (uint8_t)((crc&0xFF00)>>8);
	
	Comm_SendByte(Driver_Start_SMS);
	Comm_SendArr(payload, LengSMS);
	Comm_SendByte(Driver_End_SMS);

	free(payload);
}

void driverResponseSttDriver2(void)
{
	uint8_t* payload;
	uint16_t crc;
	uint16_t speedleft;
	uint16_t speedright;
	
	payload = (uint8_t* )malloc(LengSMS*sizeof(uint8_t));
	
	// response infor driver
	payload[0] = CMD_SET_SST_DRIVER2;
	payload[1] = getStatus(MOTOR_LEFT);
	payload[2] = getStatus(MOTOR_RIGHT);
	payload[3] = getDir(MOTOR_LEFT);
	payload[4] = getDir(MOTOR_LEFT);
	
	speedleft = getSpeedRPM(MOTOR_LEFT);
	speedright = getSpeedRPM(MOTOR_RIGHT);
	
	payload[5] = (uint8_t)(speedleft&0xFF);
	payload[6] = (uint8_t)((speedleft&0xFF00)>>8);
	payload[7] = (uint8_t)(speedright&0xFF);
	payload[8] = (uint8_t)((speedright&0xFF00)>>8);
	
	crc = CRC_Verify(payload , LengPayLoad+1);
	payload[9] = (uint8_t)(crc&0xFF);
	payload[10] = (uint8_t)((crc&0xFF00)>>8);
	
	Comm_SendByte(Driver_Start_SMS);
	Comm_SendArr(payload, LengSMS);
	Comm_SendByte(Driver_End_SMS);

	free(payload);
}

void driverResponsePID(uint8_t motor)
{
	uint8_t* payload;
	uint16_t crc;
	PID_Setup_TypDef pid_param;
	
	payload = (uint8_t* )malloc(LengSMS*sizeof(uint8_t));
	
	if(motor == MOTOR_LEFT)
	{
		getParamPID(MOTOR_LEFT, &pid_param);
		payload[0] = CMD_RP_PIDLEFT;
	}
	else
	{
		getParamPID(MOTOR_RIGHT, &pid_param);
		payload[0] = CMD_RP_PIDRIGHT;
	}
	// response infor pid
	payload[1] = (uint8_t)(pid_param.KP&0xFF);
	payload[2] = (uint8_t)((pid_param.KP&0xFF00)>>8);
	payload[3] = (uint8_t)(pid_param.KI&0xFF);
	payload[4] = (uint8_t)((pid_param.KI&0xFF00)>>8);
	payload[5] = (uint8_t)(pid_param.KD&0xFF);
	payload[6] = (uint8_t)((pid_param.KD&0xFF00)>>8);
	payload[7] = (uint8_t)(pid_param.SampleTime&0xFF);
	payload[8] = (uint8_t)((pid_param.SampleTime&0xFF00)>>8);
	
	crc = CRC_Verify(payload , LengPayLoad+1);
	payload[9] = (uint8_t)(crc&0xFF);
	payload[10] = (uint8_t)((crc&0xFF00)>>8);
	
	crc = CRC_Verify(payload , LengPayLoad+1);
	payload[9] = (uint8_t)(crc&0xFF);
	payload[10] = (uint8_t)((crc&0xFF00)>>8);
	
	Comm_SendByte(Driver_Start_SMS);
	Comm_SendArr(payload, LengSMS);
	Comm_SendByte(Driver_End_SMS);

	free(payload);
}

void driverResponseIMU(void)
{
	uint8_t* payload;
	uint16_t crc;

	payload = (uint8_t* )malloc(LengSMS*sizeof(uint8_t));
	
	payload[0] = CMD_REQUEST_ACC_IMU;
	
	payload[1] =  get_sst_adxl345_init();
	payload[2] = (uint8_t)((get_acceleration_rawX())&0xFF);
	payload[3] = (uint8_t)(((get_acceleration_rawX())&0xFF00)>>8);
	payload[4] = (uint8_t)((get_acceleration_rawY())&0xFF);
	payload[5] = (uint8_t)(((get_acceleration_rawY())&0xFF00)>>8);
	payload[6] = (uint8_t)((get_acceleration_rawZ())&0xFF);
	payload[7] = (uint8_t)(((get_acceleration_rawZ())&0xFF00)>>8);
	payload[8] = 0;
	
	crc = CRC_Verify(payload , LengPayLoad+1);
	payload[9] = (uint8_t)(crc&0xFF);
	payload[10] = (uint8_t)((crc&0xFF00)>>8);
	
	/* send package */
	Comm_SendByte(Driver_Start_SMS);
	Comm_SendArr(payload, LengSMS);
	Comm_SendByte(Driver_End_SMS);

	/* free memory */
	free(payload);
}

void driverResponseRestIMU(void)
{
	uint8_t* payload;
	uint16_t crc;

	payload = (uint8_t* )malloc(LengSMS*sizeof(uint8_t));
	
	payload[0] = CMD_RESET_IMU;
	
	payload[1] =  get_sst_adxl345_init();
	payload[2] = (uint8_t)((get_acceleration_rawX())&0xFF);
	payload[3] = (uint8_t)(((get_acceleration_rawX())&0xFF00)>>8);
	payload[4] = (uint8_t)((get_acceleration_rawY())&0xFF);
	payload[5] = (uint8_t)(((get_acceleration_rawY())&0xFF00)>>8);
	payload[6] = (uint8_t)((get_acceleration_rawZ())&0xFF);
	payload[7] = (uint8_t)(((get_acceleration_rawZ())&0xFF00)>>8);
	payload[8] = 0;
	
	crc = CRC_Verify(payload , LengPayLoad+1);
	payload[9] = (uint8_t)(crc&0xFF);
	payload[10] = (uint8_t)((crc&0xFF00)>>8);
	
	/* send package */
	Comm_SendByte(Driver_Start_SMS);
	Comm_SendArr(payload, LengSMS);
	Comm_SendByte(Driver_End_SMS);

	/* free memory */
	free(payload);
}

void driverResponseSonarEnc(void)
{
	uint8_t* payload;
	uint16_t crc;
	uint16_t encleft;
	uint16_t encright;
	uint16_t distance;
	uint16_t time;
	
	payload = (uint8_t* )malloc(LengSMS*sizeof(uint8_t));
	
	payload[0] = CMD_REQUEST_SONAR_ENC;
	
	encleft = getEncoder(ENCODERL);
	encright = getEncoder(ENCODERR);
	
	payload[1] = (uint8_t)(encleft&0xFF);
	payload[2] = (uint8_t)((encleft&0xFF00)>>8);
	payload[3] = (uint8_t)(encright&0xFF);
	payload[4] = (uint8_t)((encright&0xFF00)>>8);
	
	distance = getDistance();
		
	payload[5] = (uint8_t)(distance&0xFF);		
	payload[6] = (uint8_t)((distance&0xFF00)>>8);
	
	time = getTimeSonar();
	payload[7] = (uint8_t)(time&0xFF);
	payload[8] = (uint8_t)((time&0xFF00)>>8);;
	
	crc = CRC_Verify(payload , LengPayLoad+1);
	payload[9] = (uint8_t)(crc&0xFF);
	payload[10] = (uint8_t)((crc&0xFF00)>>8);
	
	/* send package */
	Comm_SendByte(Driver_Start_SMS);
	Comm_SendArr(payload, LengSMS);
	Comm_SendByte(Driver_End_SMS);

	/* free memory */
	free(payload);
}

void parse_sms(uint8_t* sms)
{
	uint16_t crc, crc_cal;
	
	crc = (sms[LengSMS - 1]<<8)|sms[LengSMS - 2];
	crc_cal = CRC_Verify(sms, LengPayLoad+1);
	
	if(crc != crc_cal)
		return;
	
	if(!getSttDriver() && sms[0] != CMD_SET_PID)
	{
		sms[0] = CMD_FAULT_CLEAR;
	}
	switch (sms[0])
	{
		case CMD_FAULT_CLEAR:
			driverResponseFaulClear();
			break;
		case CMD_SET_SST_DRIVER1:
			setupDriver1((uint8_t* )&sms[1]);
			driverResponseSttDriver1();
			break;
		case CMD_SET_SST_DRIVER2:
			setupDriver2((uint8_t* )&sms[1]);
			driverResponseSttDriver2();
			break;
		case CMD_RESET_ENCODER:
			if(sms[3] == 1)
				resetEncoder(ENCODERL);
			
			if(sms[5] == 1)
				resetEncoder(ENCODERR);
			
			driverResponseSonarEnc();
			break;
		case CMD_SET_PID:
			setupPID((uint8_t* )&sms[1]);
			driverResponsePID(MOTOR_LEFT);
			break;
		case CMD_RESET_IMU:
			setupDriver2((uint8_t* )&sms[1]);
			clear_flag_adxl345_init();
			//driverResponseRestIMU();
			driverResponseSttDriver2();
			break;
		case CMD_REQUEST_ACC_IMU:
			setupDriver2((uint8_t* )&sms[1]);
			driverResponseIMU();
			break;
		case CMD_REQUEST_SONAR_ENC:
			setupDriver2((uint8_t* )&sms[1]);
			driverResponseSonarEnc();
			break;
		case CMD_SET_LED:
				if(sms[1]) // master controll
				{
					set_fmc_led();
					LED_RED = sms[2];
					LED_BLUE = sms[3];
					LED_GREEN = sms[4];
				}
				else
				{
					clear_fmc_led();
				}
				driverResponseSttDriver2();
			break;
		case CMD_REQUSET_INFORDRIVER1:
			driverResponseSttDriver1();
			break;
		case CMD_REQUSET_INFORDRIVER2:
			driverResponseSttDriver2();
			break;
		case CMD_RP_PIDLEFT:
			setupDriver2((uint8_t* )&sms[1]);
			driverResponsePID(MOTOR_LEFT);
			break;
		case CMD_RP_PIDRIGHT:
			setupDriver2((uint8_t* )&sms[1]);
			driverResponsePID(MOTOR_RIGHT);
			break;
		default:
			break;
	}
	
	setRunTime();
}

void comm_parse(void)
{
	uint8_t c;
	static uint8_t len =0, flagStart =0;
	static uint64_t timDly=0;

	while(ringbuf_elements(&commbuff))
	{
		c= ringbuf_get(&commbuff);
		
		if(flagStart)
		{
			if (len < LengSMS)
			{
				datSMS[len++]= c;
			}
			else
			{
				if( c == Rasp_End_SMS)
				{
					parse_sms(datSMS);
				}
				len =0;
				flagStart =0;
				timDly=0;
			}
		}
		else
		{
			if(c == Rasp_Start_SMS) { flagStart =1;}
		}
	}
	
	if(len != LengSMS)
	{
		if(!timDly) {timDly = millis();}
		
		if(millis() - timDly > 500)
		{
			len =0;
			flagStart =0;
			timDly=0;
		}
	}
}
