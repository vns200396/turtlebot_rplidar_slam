#ifndef __MOTOR_H
#define __MOTOR_H

#include "mylib.h"
#include "led.h"
#include "pid_controller.h"

#define INA_Pin 				GPIO_Pin_12
#define INB_Pin  				GPIO_Pin_13
#define INC_Pin  				GPIO_Pin_14
#define IND_Pin  				GPIO_Pin_15

#define INA  						PBout(12)
#define INB  						PBout(13)
#define INC  						PBout(14)
#define IND 						PBout(15)

#define A  						1
#define B  						2
#define C  						3
#define D 						4


#define MotorPort				GPIOB

#define DIS 						0
#define EN 							1

#define MAX_SPEED				(uint16_t)7199

#define REVERT					0
#define FORWARD					1

#define	ENCODERL				1
#define	ENCODERR				2

#define MOTOR_LEFT	 0x01
#define MOTOR_RIGHT	 0x02


/* Motor GA25 */
#define REDUCTION_RATIO 	172
#define  MAX_SPEEDRPM 		22363				
#define  MIN_SPEEDRPM 		10	
#define  CPR 11


#define  SEC_PER_MIN 60
#define  MICROS_PER_SEC 1000000

#define  SPEEDPPS2SPEEDRPM(freq) ((unsigned int)(freq)*SEC_PER_MIN/(CPR)) //(freq*SEC_PER_MIN/CPR)
#define CIRMM (uint16_t )330	// mm

typedef struct 
{
	uint8_t L1;
	uint8_t L2;
	uint8_t L3;
	uint8_t L4;
	uint16_t PWM_EncL;
	uint16_t PWM_EncR;
}DriverInformation1_TypDef;

typedef struct 
{
	uint8_t StatusMotorLeft;
	uint8_t StatusMotorRight;
	uint8_t DirectionMotorLeft;
	uint8_t DirectionMotorRight;
	uint16_t SpeedRPMLeft;
	uint16_t SpeedRPMRight;
}DriverInformation2_TypDef;

typedef struct 
{
	uint16_t KP;
	uint16_t KI;
	uint16_t KD;
	uint16_t SampleTime;
}PID_Setup_TypDef;

typedef struct
{
	uint8_t motor;
	
	PID_TypDef pid_param;
	
	uint8_t currStatus; //desired direction
	uint8_t currDirection; //desired direction
	uint16_t desiredPWM; //desired PWM
	uint16_t currentPWM; //current PWM
	
	uint8_t event_process;
	
	uint16_t lastEncoder;
	uint16_t nowEncoder;
	uint16_t pulse;
	uint16_t freq;
	int speedRPMInput; //RPM: Round Per Minute
	int speedRPMOutput; //RPM: Round Per Minute
	int speedRPMDesired; //RPM: Round Per Minute
	
	float speed2DutyCycle;
	bool pidCtrl;
}MOTORWHEEL_TypeDef;


void motor_init(void);
uint8_t getSttDriver(void);
uint8_t getStatusIN(uint8_t _in);
uint16_t getEncoder(uint8_t enc);
void resetEncoder(uint8_t enc);
void setRunTime(void);
void runtimeDriver(void);
void setStatus(MOTORWHEEL_TypeDef* _motor, uint8_t sst);
uint8_t getStatus(uint8_t motor);
void setDir(MOTORWHEEL_TypeDef* _motor, uint8_t dir);
uint8_t getDir(uint8_t motor);
uint16_t getCirMM(void);
void runPWM(MOTORWHEEL_TypeDef* _motor, uint16_t pwm);
bool PIDSetup(MOTORWHEEL_TypeDef* _motor,float kc,float taui,float taud,int sampleTime);
bool PIDEnable(MOTORWHEEL_TypeDef* _motor,float kc,float taui,float taud,int sampleTime);
void PIDSetSpeedRMPDesired(MOTORWHEEL_TypeDef* _motor, int speedRPM);
void PIDRegulate(MOTORWHEEL_TypeDef* _motor, uint8_t doRegulate);
void setSpeedRPM(MOTORWHEEL_TypeDef* _motor, int speedRPM);
uint16_t getSpeedRPS(uint8_t motor);
uint16_t getSpeedRPM(uint8_t motor);
uint8_t geared_getRatio(void);
float getGearedSpeedRPM(uint8_t motor);
void setGearedSpeedRPM(MOTORWHEEL_TypeDef* _motor, float gearedSpeedRPM);
int getSpeedCMPM(uint8_t motor);
void setSpeedCMPM(MOTORWHEEL_TypeDef* _motor, uint16_t cm);
int getSpeedMMPS(uint8_t motor);
int setSpeedMMPS(MOTORWHEEL_TypeDef* _motor, int mm);
void process_motorPID(MOTORWHEEL_TypeDef* _motor);
void setupDriver1(uint8_t* datConfig);
void setupDriver2(uint8_t* datConfig);
void setupPIDLeft(uint8_t* datConfig);
void setupPIDRight(uint8_t* datConfig);
void setupPID(uint8_t* datConfig);
void getParamPID(uint8_t motor, PID_Setup_TypDef* pid_pram);
#endif	/* __MOTOR_H */
