#include<MotorWheel.h>


Motor::Motor()
		:PID(&speedRPMInput,&speedRPMOutput,&speedRPMDesired,KC,TAUI,TAUD) {

	PIDDisable();
}


double mapf(double val, double in_min, double in_max, double out_min, double out_max) 
{
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


long map(long val, long in_min, long in_max, long out_min, long out_max)
{
  // if input is smaller/bigger than expected return the min/max out ranges value
  if (val < in_min)
    return out_min;
  else if (val > in_max)
    return out_max;

  // map the input to the output range.
  // round up if mapping bigger ranges to smaller ranges
  else  if ((in_max - in_min) > (out_max - out_min))
    return (val - in_min) * (out_max - out_min + 1) / (in_max - in_min + 1) + out_min;
  // round down if mapping smaller ranges to bigger ranges
  else
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void Motor::updateEncoder(uint16_t now_encoder)
{
    int pulse;
	double now;
	double tim;

	//ROS_INFO_STREAM("\r\n ");
    if(currdirection == DIR_BACKOFF)
    {
		//ROS_INFO_STREAM("direct  DIR_BACKOFF");
		if(now_encoder > last_encoder)	// Revert
		{
			pulse = last_encoder +  65535 - now_encoder;
		}
		else
		{
			pulse = last_encoder - now_encoder;
		}
    }
	else
	{
		//ROS_INFO_STREAM("direct  DIR_ADVANCE");
		if(last_encoder <= now_encoder) //Forward
		{
			pulse = now_encoder - last_encoder ;
		}
		else
		{
			pulse = now_encoder + 65535 - last_encoder;
		}
	}

	now = ros::Time::now().toSec();
	tim = now - lastTimeUpdateEncoder;
	lastTimeUpdateEncoder = ros::Time::now().toSec();
	//tim = roundf(tim * 1000)/1000;
	if(tim == 0)
	{
			ROS_INFO_STREAM("time false: " << tim);	
			ros::shutdown();
	}
	freq = pulse/tim;
	// ROS_INFO_STREAM("pulse " << pulse);
	// ROS_INFO_STREAM("time " << tim);	
	// ROS_INFO_STREAM("freq " << freq);	
	// ROS_INFO_STREAM("last encoder " << last_encoder);	
	// ROS_INFO_STREAM("now encoder " << now_encoder);
	// ROS_INFO_STREAM("\r\n ");
	last_encoder= now_encoder;
}

uint16_t Motor::getLastEnc() const
{
	return last_encoder;
}


uint16_t Motor::runPWM(uint16_t PWM,bool DIR,bool saveDir) {
	speedPWM=PWM;
	if(saveDir) desiredDirection=DIR;
	return PWM;
}


uint16_t Motor::getSpeedPWM(void)
{
	return speedPWM;
}

uint16_t Motor::advancePWM(uint16_t PWM) {
	return runPWM(PWM,DIR_ADVANCE);
}

uint16_t Motor::backoffPWM(uint16_t PWM) {
	return runPWM(PWM,DIR_BACKOFF);
}

void Motor::setPWM(uint16_t PWM)
{
	pwm = PWM;
}

uint16_t Motor::getPWM() const {
	return pwm;
}

bool Motor::setCurrDir(bool dir) {
	//runPWM(getPWM(),dir);	// error
	currdirection=dir;
	return getCurrDir();
}

bool Motor::getCurrDir() const {
	return currdirection;
}

void Motor:: setCurrSpeedRPM(uint16_t speed) 
{
	currentSpeedRPM = speed;
}

uint16_t Motor:: getCurrSpeedRPM() const
{
	return currentSpeedRPM;
}

int Motor::getFreq() const {
	return freq;
}

bool Motor::setDesiredDir(bool dir) {
	//runPWM(getPWM(),dir);	// error
	desiredDirection=dir;
	return getDesiredDir();
}

bool Motor::getDesiredDir() const {
	return desiredDirection;
}

bool Motor::reverseDesiredDir() {
	runPWM(getPWM(),!getDesiredDir());
	return getDesiredDir();
}



unsigned int Motor::PIDGetSpeedRPMDesired() const {
	return speedRPMDesired;
}

unsigned int Motor::PIDSetSpeedRPMDesired(unsigned int speedRPM) {
	if(speedRPM>MAX_SPEEDRPM) speedRPMDesired=MAX_SPEEDRPM;
	else speedRPMDesired=speedRPM;
	return PIDGetSpeedRPMDesired();
}

unsigned int Motor::setSpeedRPM(int speedRPM,bool dir) {
	PIDSetSpeedRPMDesired(speedRPM);
	setDesiredDir(dir);
	return abs(getSpeedRPM());
}

int Motor::getSpeedRPM() const {

	//  if(getCurrDir()==DIR_ADVANCE)
	//  	return SPEEDPPS2SPEEDRPM(freq);
	//  return -SPEEDPPS2SPEEDRPM(freq);
	return getCurrSpeedRPM();
}


int Motor::setSpeedRPM(int speedRPM) {
	if(speedRPM>=0) return setSpeedRPM(speedRPM,DIR_ADVANCE);
	else return setSpeedRPM(abs(speedRPM),DIR_BACKOFF);
}


bool Motor::PIDGetStatus() const {
	return pidCtrl;
}
bool Motor::PIDEnable(float kc,float taui,float taud,double sampleTime) {
	PIDSetup(kc,taui,taud,sampleTime);
	lastTimeUpdateEncoder = ros::Time::now().toSec();
	return pidCtrl=true;
}
bool Motor::PIDDisable() {
	return pidCtrl=false;
}
bool Motor::PIDReset() {
	if(PIDGetStatus()==false) return false;
	//PID::Reset();
	return true;
}

bool Motor::PIDSetup(float kc,float taui,float taud,double sampleTime) {
	PID::SetTunings(kc,taui,taud);
	PID::SetOutputLimits(0,MAX_SPEEDRPM);
	PID::SetSampleTime(sampleTime);
	PID::SetMode(MANUAL);
	return true;
}

bool Motor::PIDRegulate(bool doRegulate) {
	if(PIDGetStatus()==false) return false;

	speedRPMInput=SPEEDPPS2SPEEDRPM(freq);

	PID::Compute();
	if(doRegulate) {
		
		speed2DutyCycle = speedRPMOutput;

		if(speed2DutyCycle>=MAX_SPEEDRPM) speed2DutyCycle=MAX_SPEEDRPM;
		else if(speed2DutyCycle < 0)  speed2DutyCycle=0;

		if(speed2DutyCycle>=0) {
			runPWM(map(speed2DutyCycle,0,MAX_SPEEDRPM,0,MAX_PWM),getDesiredDir(),false);
		} else {
			runPWM(map(abs(speed2DutyCycle),0,MAX_SPEEDRPM,0,MAX_PWM),!getDesiredDir(),false);
		}
		return true;
	}
	return false;
}

int Motor::PIDSpeed2DutyCycle() {
	return speed2DutyCycle;
}

int Motor::PIDSetSpeedOutput() {
	return speedRPMOutput;
}

GearedMotor::GearedMotor(unsigned int ratio):
				Motor(),_ratio(ratio) {
		
}

unsigned int GearedMotor::getRatio() const {
	return _ratio;
}

unsigned int GearedMotor::setRatio(unsigned int ratio) {
	_ratio=ratio;
	return getRatio();
}


float GearedMotor::getGearedSpeedRPM() const {
	//return (float)Motor::getSpeedRPM()/REDUCTION_RATIO;
	//return (float)Motor::getSpeedRPM()/_ratio;
	return (float)Motor::getSpeedRPM()/getRatio();
}

float GearedMotor::setGearedSpeedRPM(float gearedSpeedRPM,bool dir) {
	//Motor::setSpeedRPM(abs(gearedSpeedRPM*REDUCTION_RATIO),dir);
	Motor::setSpeedRPM(abs(round(gearedSpeedRPM*_ratio)),dir);
	return getGearedSpeedRPM();
}

// direction sensitive, 201208
float GearedMotor::setGearedSpeedRPM(float gearedSpeedRPM) {
	//Motor::setSpeedRPM(gearedSpeedRPM*_ratio);
	Motor::setSpeedRPM(round(gearedSpeedRPM*_ratio));
	return getGearedSpeedRPM();
}

MotorWheel::MotorWheel(unsigned int ratio,unsigned int cirMM):
						GearedMotor(ratio),_cirMM(cirMM) {

}

unsigned int MotorWheel::getCirMM() const {
	return _cirMM;
}

unsigned int MotorWheel::setCirMM(unsigned int cirMM) {
	if(cirMM>0) _cirMM=cirMM;
	return getCirMM();
}

// direction sensitive, 201208
int MotorWheel::getSpeedCMPM() const {
	//return int(GearedMotor::getGearedSpeedRPM()*CIR);
	return int(GearedMotor::getGearedSpeedRPM()*getCirMM()/10);
}
int MotorWheel::setSpeedCMPM(unsigned int cm,bool dir) {
	//GearedMotor::setGearedSpeedRPM(cm/CIR,dir);
	GearedMotor::setGearedSpeedRPM(cm*10.0/getCirMM(),dir);
	return getSpeedCMPM();
}
// direction sensitive, 201208
int MotorWheel::setSpeedCMPM(int cm) {
	//GearedMotor::setGearedSpeedRPM(cm/CIR,dir);
	GearedMotor::setGearedSpeedRPM(cm*10.0/getCirMM());
	return getSpeedCMPM();
}

// direction sensitive, 201208
int MotorWheel::getSpeedMMPS() const {
	return int(getSpeedCMPM()/6);//(mm/sec)/(cm/min) = 6
}

int MotorWheel::setSpeedMMPS(unsigned int mm,bool dir) {
	speedDesiredMMPS = mm;
	setSpeedCMPM(mm*6,dir);
	return getSpeedMMPS();
}
// direction sensitive, 201208
int MotorWheel::setSpeedMMPS(int mm) {
	speedDesiredMMPS=mm;
	setSpeedCMPM(mm*6);
	return getSpeedMMPS();
}

void Motor::debugger() {
	PID::debuggerPID();
}

int MotorWheel::getSpeedDesiredMMSP(void)
{
	return speedDesiredMMPS;
}
