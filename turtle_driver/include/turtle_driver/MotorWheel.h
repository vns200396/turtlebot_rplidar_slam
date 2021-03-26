#ifndef MotorWheel_H
#define MotorWheel_H

#include <pid_controller.h>


#define  KC           0.31
#define  TAUI         0.02
#define  TAUD         0.00
#define	 SAMPLETIME	  5

#define DIS 						0
#define EN 							1


#define MAX_PWM				    (uint16_t)239

#define REVERT					0
#define FORWARD					1

#define DIR_BACKOFF					false
#define DIR_ADVANCE					true

#define START_MOTOR					0
#define STOP_MOTOR					1

#define	ENCODERL				1
#define	ENCODERR				2

#define MOTOR_LEFT	 0x01
#define MOTOR_RIGHT	 0x02


/* Motor GA25 */
#define REDUCTION_RATIO 	172
#define  MAX_SPEEDRPM 		22363				
#define  CPR 11

#ifndef PI
#define PI 3.1416
#endif


#define  SEC_PER_MIN 60
#define  SPEEDPPS2SPEEDRPM(freq) ((unsigned int)(freq)*SEC_PER_MIN/(CPR)) //(freq*SEC_PER_MIN/CPR)
#define  CIRMM (uint16_t )330	// mm



class Motor: public PID {
public:
	Motor();
	
	void updateEncoder(uint16_t now_encoder);
	uint16_t getLastEnc()const;

	uint16_t runPWM(uint16_t PWM, bool DIR,bool saveDir=true);
	uint16_t getSpeedPWM(void);
	void setPWM(uint16_t PWM);
	uint16_t getPWM() const;
	uint16_t advancePWM(uint16_t PWM);
	uint16_t backoffPWM(uint16_t PWM);

	bool setDesiredDir(bool DIR);
	bool getDesiredDir() const;
	bool reverseDesiredDir();

	bool setCurrDir(bool DIR);
	bool getCurrDir() const;

	void setCurrSpeedRPM(uint16_t speed);
	uint16_t getCurrSpeedRPM() const;

	int getFreq() const;
	//int getAccRPMM() const;		// Acceleration, Round Per Min^2
	//unsigned int getSpeedRPM() const;
	//unsigned int setSpeedRPM(int speedRPM,bool dir);
	// direction sensitive 201208
	int getSpeedRPM() const;
	unsigned int setSpeedRPM(int speedRPM,bool DIR);	// preserve
	int setSpeedRPM(int speedRPM);
	//void simpleRegulate();

	bool PIDSetup(float kc=KC,float taui=TAUI,float taud=TAUD,double sampleTime=1000);
	bool PIDGetStatus() const;
	bool PIDEnable(float kc=KC,float taui=TAUI,float taud=TAUD,double sampleTime=1000);
	bool PIDDisable();
	bool PIDReset();
	bool PIDRegulate(bool doRegulate=true);
	unsigned int PIDSetSpeedRPMDesired(unsigned int speedRPM);
	unsigned int PIDGetSpeedRPMDesired() const;

	int PIDSpeed2DutyCycle();
	int PIDSetSpeedOutput(void);

	void delayMS(unsigned int ms,bool debug=false);

	//int getAccPPSS() const;
	int getSpeedPPS() const;
	long getCurrPulse() const;
	long setCurrPulse(long _pulse);
	long resetCurrPulse();

	void debugger();

private:
	uint16_t pwm =0;
	bool currdirection;
	uint16_t last_encoder =0;
	double lastTimeUpdateEncoder=0;
	double timePWM=0;
	//unsigned char pinIRQ;		// moved to isr
	//unsigned char pinIRQB;

	//bool currDirection;		// current direction
	bool desiredDirection;	// desired direction
	unsigned int speedPWM;	// current PWM

	uint16_t currentSpeedRPM;
	int freq;
	int currPPS;
	int speedRPMInput;		// RPM: Round Per Minute
	int speedRPMOutput;		// RPM
	int speedRPMDesired;	// RPM

	//float PWMEC;
	float speed2DutyCycle;
/*
	// the followings are defined in struct ISRvars, 
	// because ISR must be a global function, without parameters and no return value

	volatile unsigned int speedPPS;	// PPS: Pulses Per Second
	volatile unsigned long pulseStartMicros;
	volatile unsigned long pulseEndMicros;
 */
	bool pidCtrl;
	void ComputePPS(void);
	//Motor();

};


class GearedMotor: public Motor {	// RPM
public:
	GearedMotor(unsigned int _ratio=REDUCTION_RATIO);
	//float getGearedAccRPMM() const;		// Acceleration, Round Per Min^2
	float getGearedSpeedRPM() const;
	float setGearedSpeedRPM(float gearedSpeedRPM,bool dir);
	// direction sensitive 201208
	float setGearedSpeedRPM(float gearedSpeedRPM);
	unsigned int getRatio() const;
	unsigned int setRatio(unsigned int ratio=REDUCTION_RATIO);
private:
	unsigned int _ratio;
};


class MotorWheel: public GearedMotor {	// 
public:
	MotorWheel(unsigned int ratio=REDUCTION_RATIO,unsigned int cirMM=CIRMM);

	unsigned int getCirMM() const;
	unsigned int setCirMM(unsigned int cirMM=CIRMM);
/*
	int getAccCMPMM() const;	// Acceleration, CM Per Min^2
	unsigned int getSpeedCMPM() const;	// cm/min
	unsigned int setSpeedCMPM(int cm,bool dir);
	int getAccMMPSS() const;	// Acceleration, MM Per Sec^2
	unsigned int getSpeedMMPS() const; // mm/s
	unsigned int setSpeedMMPS(int mm,bool dir);
 */
	// direction sensitive 201208
	//int getAccCMPMM() const;	// Acceleration, CM Per Min^2
	int getSpeedCMPM() const;	// cm/min
	int setSpeedCMPM(int cm,bool dir);	// preserve
	int setSpeedCMPM(int cm);
	//int getAccMMPSS() const;	// Acceleration, MM Per Sec^2
	int getSpeedMMPS() const; // mm/s
	int setSpeedMMPS(int mm,bool dir); // preserve
	int setSpeedMMPS(int mm);

	//unsigned int runTime(unsigned int speedMMPS,bool dir,unsigned int TimeMS);
	//unsigned int runDistance(unsigned int speedMMPS,bool dir,unsigned int distanceMM);
	int getSpeedDesiredMMSP(void);
private:
	unsigned int _cirMM;
	uint16_t speedDesiredMMPS=0;
};


double mapf(double val, double in_min, double in_max, double out_min, double out_max) ;
long map(long val, long in_min, long in_max, long out_min, long out_max);
#endif
