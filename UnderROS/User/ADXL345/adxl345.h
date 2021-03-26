#ifndef __ADXL345_H
#define __ADXL345_H

#include "mylib.h"


#define G_SENSOR_SDA_PIN 			GPIO_Pin_7
#define G_SENSOR_SCL_PIN			GPIO_Pin_6
#define G_SENSOR_PORT					GPIOB

#define ADXL345_INIT_FAIL				0x00
#define ADXL345_INIT_OK					0x01
#define ADXL345_INIT_BUSY				0x02

void i2c1_init(void);
void i2c_start(void);
void i2c_stop(void);
void i2c_address_direction(uint8_t address, uint8_t direction);
void i2c_transmit(uint8_t byte);
uint8_t i2c_receive_ack(void);
uint8_t i2c_receive_nack(void);
uint8_t read_i2c(uint16_t device_address, uint16_t device_register);
void write_i2c(uint16_t device_address, uint16_t device_register, uint8_t device_data);
int16_t get_acceleration_rawX(void);
int16_t get_acceleration_rawY(void);
int16_t get_acceleration_rawZ(void);
void get_raw_data(int16_t *buf);
int get_acceleration_X(void);
int get_acceleration_Y(void);
int get_acceleration_Z(void);
int getPitch(void);
int getRoll(void);
uint8_t adxl345_get_id(void);
void set_power_mode(uint8_t  powerMode);
uint8_t get_sst_adxl345_init(void);
void clear_flag_adxl345_init(void);
uint8_t adxl345_parse(void);
#endif
