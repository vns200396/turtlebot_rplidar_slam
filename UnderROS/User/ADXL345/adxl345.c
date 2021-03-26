/******************************************************************************
 * Development team:
*          hanv14, nhungvc, huypv12, tungbq, quannv15@viettel.com.vn
*******************************************************************************/

#include "stm32f10x_i2c.h"
#include "adxl345.h"
#include "math.h"

#define TIME_OUT_READ_ADXL  2 // (ms), f = 400000Hz

uint8_t flag_check_adxl =0;

void i2c1_init(void)
{
	GPIO_InitTypeDef gpio_init_struct;
	I2C_InitTypeDef	i2c_init_struct;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB, ENABLE);
	
	gpio_init_struct.GPIO_Pin = G_SENSOR_SDA_PIN | G_SENSOR_SCL_PIN;
	gpio_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
	gpio_init_struct.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_Init(G_SENSOR_PORT, &gpio_init_struct);
	
	
	i2c_init_struct.I2C_Mode = I2C_Mode_I2C;
	i2c_init_struct.I2C_DutyCycle = I2C_DutyCycle_2;
	i2c_init_struct.I2C_OwnAddress1 = 0;
	i2c_init_struct.I2C_Ack = I2C_Ack_Enable;
	i2c_init_struct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	i2c_init_struct.I2C_ClockSpeed = 400000; 
	
	I2C_Init(I2C1, &i2c_init_struct);
	I2C_AcknowledgeConfig(I2C1, ENABLE);
	I2C_Cmd(I2C1, ENABLE);
}

void i2c_start(void)
{
	uint32_t last_time = millis();
	// neu adxl loi thi bo qua
	if(flag_check_adxl == 1){
		return;
	}
	// Wait until I2Cx is not busy anymore
	while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY)){
		if(millis() - last_time >= TIME_OUT_READ_ADXL){
			flag_check_adxl = 1;
			return;
		}
	}
	
	// Generate start condition
	I2C_GenerateSTART(I2C1, ENABLE);

	// Wait for I2C EV5.
	// It means that the start condition has been correctly released
	// on the I2C bus (the bus is free, no other devices is communicating))
	last_time = millis();
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)){
		if(millis() - last_time >= TIME_OUT_READ_ADXL){
			flag_check_adxl = 1;
			return;
		}
	}
	flag_check_adxl = 0;
}


void i2c_stop(void)
{
	uint32_t last_time = millis();
	
	if(flag_check_adxl == 1){
		return;
	}
	I2C_GenerateSTOP(I2C1, ENABLE);
	    // Wait until I2C stop condition is finished
  while (I2C_GetFlagStatus(I2C1, I2C_FLAG_STOPF)){
		if(millis() - last_time >= TIME_OUT_READ_ADXL){
			flag_check_adxl = 1;
			return;
		}
	}
	flag_check_adxl = 0;
}

void i2c_address_direction(uint8_t address, uint8_t direction)
{
	uint32_t last_time = millis();
		if(flag_check_adxl == 1){
		return;
	}
	// Send slave address
	I2C_Send7bitAddress(I2C1, address, direction);

	// Wait for I2C EV6
	// It means that a slave acknowledges his address
	if (direction == I2C_Direction_Transmitter)
	{
		while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)){
			if(millis() - last_time >= TIME_OUT_READ_ADXL){
				flag_check_adxl = 1;
				return;
			}
		}
	}
	else if (direction == I2C_Direction_Receiver)
	{
		while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)){
			if(millis() - last_time >= TIME_OUT_READ_ADXL){
				flag_check_adxl = 1;
				return;
			}
		}
	}
	flag_check_adxl = 0;
}

void i2c_transmit(uint8_t byte)
{
	uint32_t last_time = millis();
	if(flag_check_adxl == 1){
		return;
	}
	// Send data byte
	I2C_SendData(I2C1, byte);
	// Wait for I2C EV8_2.
	// It means that the data has been physically shifted out and
	// output on the bus)
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)){
		if(millis() - last_time >= TIME_OUT_READ_ADXL){
			flag_check_adxl = 1;
			return;
		}
	}
	flag_check_adxl = 0;
}

uint8_t i2c_receive_ack(void)
{
	uint32_t last_time = millis();
	if(flag_check_adxl == 1){
		return 0;
	}
	// Enable ACK of received data
	I2C_AcknowledgeConfig(I2C1, ENABLE);
	// Wait for I2C EV7
	// It means that the data has been received in I2C data register
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED)){
		if(millis() - last_time >= TIME_OUT_READ_ADXL){
			flag_check_adxl = 1;
			return 0;
		}
	}
	
	flag_check_adxl = 0;
	// Read and return data byte from I2C data register
	return I2C_ReceiveData(I2C1);
}

uint8_t i2c_receive_nack(void)
{
	uint32_t last_time = millis();
	if(flag_check_adxl == 1){
		return 0;
	}
	// Disable ACK of received data
	I2C_AcknowledgeConfig(I2C1, DISABLE);
	// Wait for I2C EV7
	// It means that the data has been received in I2C data register
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED)){
		if(millis() - last_time >= TIME_OUT_READ_ADXL){
			flag_check_adxl = 1;
			return 0;
		}
	}
	flag_check_adxl = 0;
	// Read and return data byte from I2C data register
	return I2C_ReceiveData(I2C1);
}


uint8_t read_i2c(uint16_t device_address, uint16_t device_register)
{
	uint8_t data;
	i2c_start();
	i2c_address_direction(device_address << 1, I2C_Direction_Transmitter);
	i2c_transmit(device_register);
	i2c_stop();
	i2c_start();
	i2c_address_direction(device_address << 1, I2C_Direction_Receiver);
	data = i2c_receive_nack();
	i2c_stop();
	return data;
}

void write_i2c(uint16_t device_address, uint16_t device_register, uint8_t device_data)
{
		i2c_start();
		i2c_address_direction(device_address << 1, I2C_Direction_Transmitter);
		i2c_transmit(device_register);
		i2c_transmit(device_data);
		i2c_stop();
}

int16_t get_acceleration_rawX(void){
	int16_t rawX = 0;
	uint8_t accel_data1, accel_data2;
	//protected code
	accel_data1 = read_i2c(0x53, 0x32);
	accel_data2 = read_i2c(0x53, 0x33);
	rawX = (accel_data2 << 8) | accel_data1;
	return (rawX);
}

int16_t get_acceleration_rawY(void){
	int16_t rawY = 0;
	uint8_t accel_data1, accel_data2;
	//protected code
	accel_data1 = read_i2c(0x53,0x34);
	accel_data2 = read_i2c(0x53,0x35);
	rawY = (accel_data2 << 8) | accel_data1;
	return (rawY);
}

int16_t get_acceleration_rawZ(void){
	int16_t rawZ = 0;
	uint8_t accel_data1, accel_data2;
	accel_data1 = read_i2c(0x53,0x36);
	accel_data2 = read_i2c(0x53,0x37);
	rawZ = (accel_data2 << 8) | accel_data1;
	return (rawZ);
}

void get_raw_data(int16_t *buf) {
	uint8_t accel_register[6];
	uint8_t i;
	uint8_t j;

	uint8_t addr_register = 0x32;//x0 register

	for (i = 0; i < 6; i++) {
		accel_register[i] = read_i2c(0x53, addr_register);
		addr_register++;
	}
	for (i = 0; i < 3; i++) {
		j = i << 1;
		buf[i] = accel_register[j] + (accel_register[j + 1] << 8);
	}
}

int get_acceleration_X(void)
{
	signed int short raw;
	signed int acceleration;
	raw = (signed int short) get_acceleration_rawX();
	acceleration = (signed int)(((signed int)raw) * 3.9);
	return acceleration;
}

int get_acceleration_Y(void)
{
	signed int short raw;
	signed int acceleration; //= (signed int)(((signed int)raw) * 3.9);
	raw = (signed int short) get_acceleration_rawY();
	acceleration = (signed int)(((signed int)raw) * 3.9);
	return acceleration;
}

int get_acceleration_Z(void)
{
	signed int short raw;
	signed int acceleration; // = (signed int)(((signed int)raw) * 3.9);
	raw = (signed int short) get_acceleration_rawZ();
	acceleration = (signed int)(((signed int)raw) * 3.9);
	return acceleration;
}

int getPitch(void)
{				
	double result;
	result = atan(get_acceleration_Y()/sqrt(pow(get_acceleration_X(), 2) + pow(get_acceleration_Z(), 2)));
	result = (result * 180) / PI;
	return result;
}

int getRoll(void)
{
	double result;
	result = atan(-get_acceleration_X()/get_acceleration_Z());
	result = (result * 180) / PI;
	return result;
}

uint8_t adxl345_get_id(void)
{
	return (read_i2c(0x53,0x00));
}

void set_power_mode(uint8_t  powerMode)
{
	uint8_t i2c_data = read_i2c(0x53,0x2d);
	if (powerMode == 1){
		i2c_data = i2c_data | (powerMode << 3);
	} else if (powerMode == 0){
		i2c_data &= ~(1<<3);
	}
	i2c_data = i2c_data | (powerMode<<3);
	write_i2c(0x53,0x2d,i2c_data);
}

uint8_t flagADXL345_init=ADXL345_INIT_BUSY;
uint8_t event_init =0;
uint8_t get_sst_adxl345_init(void)
{
	return flagADXL345_init;
}

void clear_flag_adxl345_init(void)
{
	flagADXL345_init =ADXL345_INIT_BUSY;
	if(event_init < 5)
	{
		event_init =0;
	}
}

uint8_t adxl345_parse(void)
{
	static uint32_t timedly =0;
	uint8_t id = 0;
	uint8_t result = ADXL345_INIT_BUSY;
	
	switch (event_init)
	{
		case 0:
			i2c1_init();
		  timedly = millis();
			event_init=1;
			break;
		case 1:
			// wait ack
			if(millis() - timedly > 500)
			{
				timedly =0;
				event_init++;
			}
			break;
		case 2:
			id = adxl345_get_id();
			if (id != 0xE5) {
//			my_printf("[ADXL]id is wrong\r\n");
			  event_init=6;
				return false;
			}
//	my_printf("[ADXL] id is correct\r\n");
		  timedly = millis();

			event_init++;
			break;
		case 3:
			// wait ack
			if(millis() - timedly > 500)
			{
				timedly =0;
				event_init++;
			}
			break;
		case 4:
			//16 bit, full resolution
			write_i2c(0x53,0x31,0x0B);
			//data rate 100Hz default
			//writeI2C0(0x53,0x31,0x0B);
			set_power_mode(0x01); // note fix
			//Delay_ms(1000);
			//ADXL345_Calib();
			event_init++;
			break;
		case 5:
			//proceed
		  flagADXL345_init=ADXL345_INIT_OK;
			result = flagADXL345_init;
			break;
		case 6:
			//proceed
		  flagADXL345_init=ADXL345_INIT_FAIL;
			result = flagADXL345_init;
			break;
		default:
			event_init =0;
			break;
	}
	return result;
}
