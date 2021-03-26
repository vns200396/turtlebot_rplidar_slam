#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "ros/ros.h"
#include "turtle_driver/MotorWheel.h"

#include <boost/thread/condition_variable.hpp>
#include <boost/lexical_cast.hpp>
#include <stdint.h>
#include <string>


#define RingBufferSize				128

#define Rasp_Start_SMS		        0xAA
#define Driver_Start_SMS 	        0x55

#define Rasp_End_SMS			    Driver_Start_SMS
#define Driver_End_SMS		        Rasp_Start_SMS

#define LengSMS							(uint8_t )11
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


#define TimeRequest					(double )0.01

#define COMM_OK 0x00
#define COMM_FAIL 0x01
#define COMM_TIMEOUT 0x02
#define COMM_WAIT 0x03


typedef struct _driver_cmd_packet_t
{
	uint8_t cmd_flag;
	uint8_t data[8];
	uint16_t crc;
} __attribute__((packed)) driver_cmd_packet_t;

typedef struct _driverinformation1_t
{
	uint8_t ina;
	uint8_t inb;
	uint8_t inc;
	uint8_t ind;
	uint16_t speedLeft;
	uint16_t speedRight;
} __attribute__((packed)) DriverInformation1_TypDef;


typedef struct _driverinformation2_t
{
	uint8_t StatusMotorLeft;
	uint8_t StatusMotorRight;
	uint8_t DirectionMotorLeft;
	uint8_t DirectionMotorRight;
	uint16_t SpeedRPMLeft;
	uint16_t SpeedRPMRight;
}__attribute__((packed)) DriverInformation2_TypDef;

typedef struct _PID_Setup_TypDef
{
	uint16_t KP;
	uint16_t KI;
	uint16_t KD;
	uint16_t SampleTime;
}__attribute__((packed)) PID_Setup_TypDef;

typedef struct _CMD_Request_IMU_TypDef
{
	uint8_t Status_IMU;
	uint16_t Acceleration_RawX;
	uint16_t Acceleration_RawY;
	uint16_t Acceleration_RawZ;
	uint8_t RawData[6];
}__attribute__((packed)) IMU_TypDef;

typedef struct _ENC_SONAR_TypDef
{
	uint16_t EncoderLeft;
	uint16_t EncoderRight;
	uint16_t Distance;
	uint16_t TimeSonar;
}__attribute__((packed)) ENC_SONAR_TypDef;

namespace serial {
  class Serial;
}

#define WHEELSPAN 320

class Controller
{
private:
	#define LEN_Script 31

	const char *port_;
	int baud_;
	bool connected_;
	serial::Serial *serial_;

	double remaingtime;
	uint8_t flagStart =0;
	uint8_t len_msg=0;
	uint8_t datasms[LengSMS];

	uint8_t scription[LEN_Script+1];
	uint8_t head_script=0;
	uint8_t tail_script=0;
	uint8_t cycle_request= CMD_SET_SST_DRIVER2;

	MotorWheel* _wheelLeft;
	MotorWheel* _wheelRight;

	DriverInformation1_TypDef driverinfor1;
	DriverInformation2_TypDef driverinfor2;

	void debug(void);

  	void send_command(uint8_t cmd, const void * payload, uint8_t paysize);
  	uint8_t waitDriverResponse(driver_cmd_packet_t * payload, uint32_t timeout);
	uint8_t parseSMS(uint8_t * sms, driver_cmd_packet_t * payload);

  	void cmdSetStatusDriver1(bool sstL, bool sstR, bool dirLeft, bool dirRight, uint16_t pwmL, uint16_t pwmR);
	void cmdSetStatusDriver2(uint8_t cmd,bool sstL, bool sstR, bool dirLeft, bool dirRight, uint16_t speedrpmleft, uint16_t speedrpmright);
	void setPID(double kp, double ki, double kd, double sampletime = 0.01);
	void setLed(void);

	void getStatusDriver1(DriverInformation1_TypDef * driverinfor, uint8_t arrdat[], uint8_t len);
	void getStatusDriver2(DriverInformation2_TypDef * driverinfor, uint8_t arrdat[], uint8_t len);
	void get_ACC_IMU(IMU_TypDef * imu_str, uint8_t arrdat[], uint8_t len);
	void getEncSonar(ENC_SONAR_TypDef * enc_sonar, uint8_t arrdat[], uint8_t len);

	unsigned int _wheelspanMM;


	unsigned char _carStat;
	unsigned char setCarStat(unsigned char stat);

	unsigned char _switchMotorsStat;
	unsigned char setSwitchMotorsStat(unsigned char switchMotorsStat);

	unsigned int _radiusMM;
	unsigned int setRadiusMM(unsigned int radiusMM);

	// Base Actions
	unsigned int setMotorAll(unsigned int speedMMPS=0,bool dir=DIR_ADVANCE);
	unsigned int setMotorAllStop();
	unsigned int setMotorAllAdvance(unsigned int speedMMPS=0);
	unsigned int setMotorAllBackoff(unsigned int speedMMPS=0);

	unsigned int setCarAdvanceBase(unsigned int speedMMPSL=0,unsigned int speedMMPSR=0);
	unsigned int setCarBackoffBase(unsigned int speedMMPSL=0,unsigned int speedMMPSR=0);

	unsigned int setCarRotateAngle(unsigned int speedMMPS=0,float radian=0);

	unsigned int setCarStraightDistance(unsigned int speedMMPS=0,unsigned long distance=0);

	unsigned int setCarArcBase(unsigned int speedMMPS=0,unsigned int radiusMM=WHEELSPAN);
	unsigned int setCarArcTime(unsigned int speedMMPS=0,unsigned int radiusMM=WHEELSPAN,
	unsigned long duration=5000,unsigned int uptime=500);
	unsigned int setCarArcAngle(unsigned int speedMMPS=0,unsigned int radiusMM=WHEELSPAN,
	float radian=0,unsigned int uptime=500);

	unsigned int CRC_Verify(uint8_t* cBuffer, unsigned int iBufLen);
	Controller();
public:
  	Controller(const char *port, int baud, MotorWheel* wheelLeft, MotorWheel* wheelRight, unsigned int wheelspanMM=WHEELSPAN);

	IMU_TypDef imu;
	ENC_SONAR_TypDef encoder_sonar;

	uint8_t LedMode;
	uint8_t LedRed;
	uint8_t LedBlue;
	uint8_t LedGreen;


	void requestCommand(uint8_t cmd);
	void spin(void);

	unsigned int getWheelspanMM() const;
	unsigned int setWheelspanMM(unsigned int wheelspan);

	// Car Direction Control
	unsigned char switchMotors();
	unsigned char switchMotorsReset();

  	// Simple Actions
	unsigned int setCarStop();
	unsigned int setCarAdvance(unsigned int speedMMPS=0);
	unsigned int setCarBackoff(unsigned int speedMMPS=0);
	unsigned int setCarRotateLeft(unsigned int speedMMPS=0);
	unsigned int setCarRotateRight(unsigned int speedMMPS=0);

	unsigned int setCarUpperLeft(unsigned int speedMMPS=0,unsigned int radiusMM=(WHEELSPAN>>1));
	unsigned int setCarLowerLeft(unsigned int speedMMPS=0,unsigned int radiusMM=(WHEELSPAN>>1));
	unsigned int setCarLowerRight(unsigned int speedMMPS=0,unsigned int radiusMM=(WHEELSPAN>>1));
	unsigned int setCarUpperRight(unsigned int speedMMPS=0,unsigned int radiusMM=(WHEELSPAN>>1));

	// Staight Distance
	unsigned int setCarAdvanceDistance(unsigned int speedMMPS=0,unsigned long distance=0);
	unsigned int setCarBackoffDistance(unsigned int speedMMPS=0,unsigned long distance=0);

	// Rotate(Spin) Angle(degree or radian)
	unsigned int setCarRotateLeftAngle(unsigned int speedMMPS=0,float radian=0);
	unsigned int setCarRotateRightAngle(unsigned int speedMMPS=0,float radian=0);


	// ARC Path
	unsigned int setCarUpperLeftTime(unsigned int speedMMPS=0,unsigned int radiusMM=WHEELSPAN,unsigned long duration=5000,unsigned int uptime=500);
	unsigned int setCarLowerLeftTime(unsigned int speedMMPS=0,unsigned int radiusMM=WHEELSPAN,unsigned long duration=5000,unsigned int uptime=500);
	unsigned int setCarUpperRightTime(unsigned int speedMMPS=0,unsigned int radiusMM=WHEELSPAN,unsigned long duration=5000,unsigned int uptime=500);
	unsigned int setCarLowerRightTime(unsigned int speedMMPS=0,unsigned int radiusMM=WHEELSPAN,unsigned long duration=5000,unsigned int uptime=500);
 

	unsigned int setCarUpperLeftAngle(unsigned int speedMMPS=0,unsigned int radiusMM=WHEELSPAN,float radian=0,unsigned int uptime=500);
	unsigned int setCarLowerLeftAngle(unsigned int speedMMPS=0,unsigned int radiusMM=WHEELSPAN,float radian=0,unsigned int uptime=500);
	unsigned int setCarUpperRightAngle(unsigned int speedMMPS=0,unsigned int radiusMM=WHEELSPAN,float radian=0,unsigned int uptime=500);
	unsigned int setCarLowerRightAngle(unsigned int speedMMPS=0,unsigned int radiusMM=WHEELSPAN,float radian=0,unsigned int uptime=500); 

	// Single Wheel Control
	unsigned int wheelLeftSetSpeedMMPS(unsigned int speedMMPS=0,bool dir=DIR_ADVANCE);
	unsigned int wheelLeftGetSpeedMMPS() const;
	unsigned int wheelRightSetSpeedMMPS(unsigned int speedMMPS=0,bool dir=DIR_ADVANCE);
	unsigned int wheelRightGetSpeedMMPS() const;

	// PID Control
	bool PIDEnable(float kc=KC,float taui=TAUI,float taud=TAUD,double interval=1);
	bool PIDRegulate();
	void delayMS(unsigned int ms=100, bool debug=false);

	// Progressive Speed Control
	unsigned int getCarSpeedMMPS() const;
	unsigned int setCarSpeedMMPS(unsigned int speedMMPS=0,unsigned int ms=1000);
	unsigned int setCarSpeedMMPSArc(unsigned int speedMMPS=0,unsigned int radiusMM=WHEELSPAN,unsigned int ms=1000);
	//unsigned int setCarSpeedMMPSD(unsigned int speedMMPSL=0,unsigned int speedMMPSR=0,unsigned int ms=1000);
	unsigned int setCarSlow2Stop(unsigned int ms=1000);

	enum {STAT_UNKNOWN,
			STAT_STOP,
			STAT_ADVANCE,
			STAT_BACKOFF,
			STAT_ROTATELEFT,
			STAT_ROTATERIGHT,
			STAT_UPPERLEFT,
			STAT_LOWERLEFT,
			STAT_LOWERRIGHT,
			STAT_UPPERRIGHT,
	};
	unsigned char getCarStat() const;

	enum {
		MOTORS_FB,
		MOTORS_BF,
	};
	unsigned char getSwitchMotorsStat() const;
	
	unsigned int getRadiusMM() const;

	/* IMU */
	int16_t get_acceleration_rawX(void);
	int16_t get_acceleration_rawY(void);
	int16_t get_acceleration_rawZ(void);
	void get_raw_data(int16_t *buf);
	int get_acceleration_X(void);
	int get_acceleration_Y(void);
	int get_acceleration_Z(void);
	int getPitch(void);
	int getRoll(void);
};

#pragma pack()

#endif  