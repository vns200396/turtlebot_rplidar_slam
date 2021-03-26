#include "motor.h"


MOTORWHEEL_TypeDef	motorRight;
MOTORWHEEL_TypeDef	motorLeft;

uint8_t flagRunTimeDriver =0;
uint64_t runtimeDly=0;

uint8_t getStatusIN(uint8_t _in)
{
	uint8_t ret=0;
	
	switch(_in)
	{
		case A:
			if(INA)
				ret = 1;
			break;
		case B:
			if(INB)
				ret = 1;
			break;
		case C:
			if(INC)
				ret = 1;
			break;
		case D:
			if(IND)
				ret = 1;
			break;
		default:
			break;
	}
	
	return ret;
}

static void setStatusM1(uint8_t _ina, uint8_t _inb)
{
	uint8_t ina;
	uint8_t inb;
	
	ina = getStatusIN(A);
	if((ina != _ina) && (_ina < (FORWARD+1)))
	{
		INA = _ina;
	}
	
	inb = getStatusIN(B);
	if((inb != _inb) && (_inb < (FORWARD+1)))
	{
		INB = _inb;
	}
}

static void setStatusM2(uint8_t _inc, uint8_t _ind)
{
	uint8_t inc;
	uint8_t ind;
	
	inc = getStatusIN(C);
	if((inc != _inc) && (_inc < (FORWARD+1)))
	{
		INC = _inc;
	}
	
	ind = getStatusIN(D);
	if((ind != _ind) && (_ind < (FORWARD+1)))
	{
		IND = _ind;
	}
}

static void setSpeed(uint16_t speedMotor1, uint16_t speedMotor2)
{
	if((TIM4->CCR3 != speedMotor1) && (speedMotor1 <= MAX_SPEED))
	{
		TIM4->CCR3 = speedMotor1;
	}
	
	if((TIM4->CCR4 != speedMotor2) && (speedMotor2 <= MAX_SPEED))
	{
		TIM4->CCR4 = speedMotor2;
	}
}

static void setPWM(const uint8_t _motor, uint16_t pwm)
{
	if(pwm <= MAX_PWM)
	{
		if(_motor == MOTOR_LEFT)
		{
				TIM4->CCR3 = pwm;
		}
		
		if(_motor == MOTOR_RIGHT)
		{
				TIM4->CCR4 = pwm;
		}
	}
}


uint16_t getEncoder(uint8_t enc)
{
	if(enc == MOTOR_LEFT)
		return TIM_GetCounter(TIM3);
	
	return TIM_GetCounter(TIM2);
}

void resetEncoder(uint8_t enc)
{
	if(enc == MOTOR_LEFT)
		{TIM_ResetCounter(TIM3);}
	else
		{TIM_ResetCounter(TIM2);}
}

void setStatus(MOTORWHEEL_TypeDef* _motor, uint8_t sst)
{
	_motor->currStatus = sst;
	
	if(_motor->motor == MOTOR_LEFT)
	{
		if(sst == DIS)
			setStatusM1(REVERT, REVERT);
	}
	
	if(_motor->motor == MOTOR_RIGHT)
	{
		if(sst == DIS)
			setStatusM2(REVERT, REVERT);
	}
}

uint8_t getStatus(uint8_t motor)
{
	if(motor == MOTOR_LEFT)
			return motorLeft.currStatus;
	
	return motorRight.currStatus;
}

void setDir(MOTORWHEEL_TypeDef* _motor, uint8_t dir)
{
	_motor->currDirection = dir;
	
	if(_motor->currStatus == DIS)
	{
		return;
	}
	
  if(_motor->motor == MOTOR_LEFT)
	{
		if(dir == REVERT)
			setStatusM1(REVERT, FORWARD);
		else
			setStatusM1(FORWARD, REVERT);
	}
	
	if(_motor->motor == MOTOR_RIGHT)
	{
		if(dir == REVERT)
			setStatusM2(FORWARD, REVERT);
		else
			setStatusM2(REVERT, FORWARD);
	}
}

uint8_t getDir(uint8_t motor)
{
	if(motor == MOTOR_LEFT)
			return motorLeft.currDirection;
	
	return motorRight.currDirection;
}

uint16_t getCirMM(void)
{
	return CIRMM;
}

void runPWM(MOTORWHEEL_TypeDef* _motor, uint16_t pwm)
{
	if (pwm <= MIN_PWM)
	{
		pwm =0;
	}		
	_motor->desiredPWM = pwm;
	if(_motor->currentPWM != _motor->desiredPWM)
	{
		setPWM(_motor->motor, pwm);
		_motor->currentPWM = _motor->desiredPWM;
	}
}



void PIDSetSpeedRMPDesired(MOTORWHEEL_TypeDef* _motor, int speedRPM)
{
	if(speedRPM > MAX_SPEEDRPM)
		_motor->speedRPMDesired = MAX_SPEEDRPM;
	else
		_motor->speedRPMDesired = speedRPM;
}

uint16_t computeEncoder(MOTORWHEEL_TypeDef* _motor)
{
	uint16_t pulse;
	_motor->nowEncoder = getEncoder(_motor->motor);
	
	if(_motor->currDirection == REVERT)
	{
		if(_motor->nowEncoder >= _motor->lastEncoder)
		{
			pulse = _motor->nowEncoder - _motor->lastEncoder;
		}
		else
		{
			pulse = _motor->nowEncoder + 65535 - _motor->lastEncoder;
		}
	}
	else
	{
		if(_motor->nowEncoder <= _motor->lastEncoder)
		{
				pulse =  _motor->lastEncoder - _motor->nowEncoder;
		}
		else
		{
				pulse = _motor->lastEncoder + 65535 - _motor->nowEncoder ;
		}
	}
	_motor->lastEncoder = _motor->nowEncoder;
	return pulse;
}

bool PIDSetup(MOTORWHEEL_TypeDef* _motor,float kc,float taui,float taud,int sampleTime) 
{
	PID_Reset(&_motor->pid_param);
	PID_SetSampleTime(&_motor->pid_param,sampleTime);
	PID_SetTunings(&_motor->pid_param, kc,taui,taud);
	PID_SetOutputLimits(&_motor->pid_param,0,MAX_SPEEDRPM);
	PID_SetMode(&_motor->pid_param,MANUAL);
	return true;
}

bool PIDEnable(MOTORWHEEL_TypeDef* _motor,float kc,float taui,float taud,int sampleTime)
{
	PIDSetup(_motor,kc,taui,taud,sampleTime);
	return true;
}


void PIDRegulate(MOTORWHEEL_TypeDef* _motor, uint8_t doRegulate)
{
	double sampletimeInsec;
	
	if(!_motor->pidCtrl)
			return;
	
	_motor->pulse = computeEncoder(_motor);
	
	sampletimeInsec = (_motor->pid_param.SampleTime*1.0)/1000;
	_motor->freq = _motor->pulse/sampletimeInsec;
	
	_motor->pid_param.Input = SPEEDPPS2SPEEDRPM(_motor->freq);
	
	if(_motor->pid_param.Input > MAX_SPEEDRPM)
	{
		_motor->pid_param.Input = MAX_SPEEDRPM;
	}

	_motor->pid_param.Setpoint = _motor->speedRPMDesired;

	PID_Compute(&_motor->pid_param);

	
	if(doRegulate)
	{
		_motor->speed2DutyCycle = _motor->pid_param.Output;
		
		if(_motor->speed2DutyCycle > MAX_SPEEDRPM) _motor->speed2DutyCycle = MAX_SPEEDRPM;

		runPWM(_motor,map(_motor->speed2DutyCycle, 0, MAX_SPEEDRPM, MIN_PWM, MAX_PWM));
	}
}

void setSpeedRPM(MOTORWHEEL_TypeDef* _motor, int speedRPM)
{
	PIDSetSpeedRMPDesired(_motor, speedRPM);
}

uint16_t getSpeedRPS(uint8_t motor) 
{
	if(motor == MOTOR_LEFT)
			 	return motorLeft.freq;
	
	 return motorRight.freq;
}

uint16_t getSpeedRPM(uint8_t motor) 
{
	if(motor == MOTOR_LEFT)
			 	return SPEEDPPS2SPEEDRPM(motorLeft.freq);
	
	 return SPEEDPPS2SPEEDRPM(motorRight.freq);
}

uint8_t geared_getRatio(void)
{
	return (uint8_t)REDUCTION_RATIO;
}

float getGearedSpeedRPM(uint8_t motor)
{
	return (float)getSpeedRPM(motor)/geared_getRatio();
}

void setGearedSpeedRPM(MOTORWHEEL_TypeDef* _motor, float gearedSpeedRPM)
{
	int result;
	
	result = round(gearedSpeedRPM*geared_getRatio());
	setSpeedRPM(_motor, abs(result));
}


int getSpeedCMPM(uint8_t motor)
{
	return getGearedSpeedRPM(motor)*getCirMM()/10;
}

void setSpeedCMPM(MOTORWHEEL_TypeDef* _motor, uint16_t cm)
{
	setGearedSpeedRPM(_motor, cm*10.0/getCirMM());
}

int getSpeedMMPS(uint8_t motor)
{
	return (int )getSpeedCMPM(motor)/6;//(mm/sec)/(cm/min) = 6
}

int setSpeedMMPS(MOTORWHEEL_TypeDef* _motor, int mm)
{
		setSpeedCMPM(_motor, mm*6);
	return getSpeedMMPS(_motor->motor);
}

void process_motorPID(MOTORWHEEL_TypeDef* _motor)
{
	switch (_motor->event_process)
	{
		case 0:
			PIDRegulate(_motor, 1);
			_motor->event_process=1;
			_motor->pid_param.lastTime = millis();
			break;
		case 1:
			if(millis() - _motor->pid_param.lastTime >  _motor->pid_param.SampleTime)
			{
				_motor->event_process=0;
			}
			break;
		default:
			_motor->event_process=0;
			break;
	}
}

void setupDriver1(uint8_t* datConfig)
{
	DriverInformation1_TypDef		driverInfor1;
	memmove((uint8_t *)&driverInfor1,datConfig, sizeof(driverInfor1));
	
	setStatusM1(driverInfor1.L1, driverInfor1.L2);
	setStatusM2(driverInfor1.L3, driverInfor1.L4);
	setSpeed(driverInfor1.PWM_EncL, driverInfor1.PWM_EncR);
}

void setupDriver2(uint8_t* datConfig)
{
	DriverInformation2_TypDef		driverInfor2;
	memmove((uint8_t *)&driverInfor2,datConfig, sizeof(driverInfor2));
	
	setStatus(&motorLeft, driverInfor2.StatusMotorLeft);
	setDir(&motorLeft, driverInfor2.DirectionMotorLeft);
	setSpeedRPM(&motorLeft, driverInfor2.SpeedRPMLeft);
	
	setStatus(&motorRight, driverInfor2.StatusMotorRight);
	setDir(&motorRight, driverInfor2.DirectionMotorRight);
	setSpeedRPM(&motorRight, driverInfor2.SpeedRPMRight);
}


void setupPIDLeft(uint8_t* datConfig)
{
	PID_Setup_TypDef pid_pram;
	float kp,ki,kd;
	
	memmove((uint8_t *)&pid_pram,datConfig, sizeof(pid_pram));

	kp = (float)(pid_pram.KP)/100;
	ki = (float)(pid_pram.KI)/100;
	kd = (float)(pid_pram.KD)/100;

	PIDSetup(&motorLeft,kp,ki,kd,pid_pram.SampleTime);
}

void setupPIDRight(uint8_t* datConfig)
{
	PID_Setup_TypDef pid_pram;
	float kp,ki,kd;
	
	memmove((uint8_t *)&pid_pram,datConfig, sizeof(pid_pram));

	kp = (float)(pid_pram.KP)/100;
	ki = (float)(pid_pram.KI)/100;
	kd = (float)(pid_pram.KD)/100;
	
	PIDSetup(&motorRight,kp,ki,kd,pid_pram.SampleTime);
}

void setupPID(uint8_t* datConfig)
{
	setupPIDLeft(datConfig);
	setupPIDRight(datConfig);
}

void getParamPID(uint8_t motor, PID_Setup_TypDef* pid_pram)
{
	if(motor == MOTOR_LEFT)
	{
		pid_pram->KP = (uint16_t )(motorLeft.pid_param.Kp*100);
		pid_pram->KI = (uint16_t )(motorLeft.pid_param.Ki*100);
		pid_pram->KD = (uint16_t )(motorLeft.pid_param.Kd*100);
		pid_pram->SampleTime = motorLeft.pid_param.SampleTime;
	}
	
	pid_pram->KP = (uint16_t )(motorRight.pid_param.Kp*100);
	pid_pram->KI = (uint16_t )(motorRight.pid_param.Ki*100);
	pid_pram->KD = (uint16_t )(motorRight.pid_param.Kd*100);
	pid_pram->SampleTime = motorRight.pid_param.SampleTime;
}

uint8_t getSttDriver(void)
{
	double sumleft,sumright;
	
	sumleft = motorLeft.pid_param.Kp + motorLeft.pid_param.Ki + motorLeft.pid_param.Kd;
	sumright = motorRight.pid_param.Kp + motorRight.pid_param.Ki + motorRight.pid_param.Kd;
	
	if(sumleft == 0 || sumright == 0)
	{
		// need to config
		return 0;
	}
	
	return 1;
}

void motor_init(void)
{
	GPIO_InitTypeDef motorTypDef;
	
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);
	
	motorTypDef.GPIO_Pin = INA_Pin  | INB_Pin  | INC_Pin  | IND_Pin ;
	motorTypDef.GPIO_Mode = GPIO_Mode_Out_PP;
	motorTypDef.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(MotorPort, &motorTypDef);
	
	motorTypDef.GPIO_Pin = GPIO_Pin_13;
	motorTypDef.GPIO_Mode = GPIO_Mode_Out_PP;
	motorTypDef.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &motorTypDef);
	
	tim_init();
	
	setStatusM1(REVERT, REVERT);
	setStatusM2(REVERT, REVERT);
	setSpeed(DIS, DIS);
	resetEncoder(ENCODERL);
	resetEncoder(ENCODERR);
	
	PIDEnable(&motorLeft,0,0,0,10);
	motorLeft.motor =MOTOR_LEFT;
	motorLeft.pidCtrl =1;

	PIDEnable(&motorRight,0,0,0,10);
	motorRight.motor =MOTOR_RIGHT;
	motorRight.pidCtrl =1;
}

void setRunTime(void)
{
	flagRunTimeDriver =1;
	runtimeDly = millis();
}

uint8_t standby_mode =0;
void runtimeDriver(void)
{
	uint8_t sstA, sstB, sstC, sstD;
	static uint64_t dlyTick=0;
//	setDir(&motorLeft, motorLeft.currDirection);
//	setDir(&motorRight, motorRight.currDirection);
	
	process_motorPID(&motorLeft);
	process_motorPID(&motorRight);
	
	if(motorLeft.currentPWM == 0 && motorRight.currentPWM == 0)
	{
		if(!standby_mode)
		{
			if(!dlyTick) dlyTick = millis();
			
			if(millis() - dlyTick > 1500) standby_mode =1;
		}
	}
	else
	{
		standby_mode =0;
		dlyTick=0;
	}
	
	if(flagRunTimeDriver && (getSttDriver()))
	{
		if(millis() - runtimeDly > 500)
		{
			flagRunTimeDriver =0;
		}
		
		sstA = getStatusIN(A);
		sstB = getStatusIN(B);
		sstC = getStatusIN(C);
		sstD = getStatusIN(D);
		
		if(((sstA == sstB) && (sstC == sstD)) || (standby_mode))
		{
			turnLed(BLUE);
		}
		else
		{
			turnLed(GREEN);
		}
	}
	else
	{
		setStatusM1(REVERT, REVERT);
		setStatusM2(REVERT, REVERT);
		turnLed(RED);
	}
}

