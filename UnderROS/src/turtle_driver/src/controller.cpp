
#include "turtle_driver/controller.h"
#include "serial/serial.h"
 #include <ros/console.h>

#include <boost/bind.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <unistd.h>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <string.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>


Controller::Controller(const char *port, int baud, 
                       MotorWheel* wheelLeft,MotorWheel* wheelRight,unsigned int wheelspanMM)
                        :port_(port), baud_(baud), connected_(false), serial_(NULL),
                        _wheelLeft(wheelLeft),_wheelRight(wheelRight),_wheelspanMM(wheelspanMM)
{
    if(!serial_) serial_ = new serial::Serial();


    try
    {
        serial::Timeout to(serial::Timeout::simpleTimeout(500));
        serial_->setTimeout(to);
        serial_->setPort(port_);
        serial_->setBaudrate(baud_);
        serial_->open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
    }

    if(serial_->isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
    }
}

//  Controller::~Controller()
//  {

//  }


//  void Controller::connect(void)
//  {

//  }

/**
  * @brief  Calculate CRC checksum
  * @param  Pointer to payload
  * @param  Length payload
  * @retval Result CRC
  */
unsigned int Controller::CRC_Verify(uint8_t* cBuffer, unsigned int iBufLen)
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


void Controller::send_command(uint8_t cmd, const void * payload, uint8_t paysize)
{
    uint8_t syncByte;
    uint8_t buff[LengPayLoad+1]="";
    unsigned int crc;

    syncByte = Rasp_Start_SMS;
    serial_->write((uint8_t *)&syncByte, 1);

    buff[0] = cmd;

    if(paysize > LengPayLoad)
    {
        ROS_INFO_STREAM("paysize too large");
        return;
    }
    else if (payload && paysize)
    {
        memmove((uint8_t *)&buff[1], payload, paysize);
    }

    serial_->write((uint8_t *)buff, 9);
    crc = CRC_Verify((uint8_t *)buff, LengPayLoad+1);

  //  ROS_INFO_STREAM("crc " << crc);
    serial_->write((uint8_t *)&crc, 2);

    syncByte = Rasp_End_SMS;
    serial_->write((uint8_t *)&syncByte, 1);
}


uint8_t Controller::parseSMS(uint8_t * sms, driver_cmd_packet_t * payload)
{
    uint8_t* pck = (uint8_t *)payload;
    uint16_t crc_cal;
    uint16_t crc;


    crc_cal = CRC_Verify(sms, LengPayLoad+1);
	crc = (sms[LengSMS - 1]<<8)|sms[LengSMS - 2];

	if(crc != crc_cal)
	{
        ROS_INFO_STREAM("crc is not invalid!");
		ROS_INFO("crc receiv %x", crc);
    	ROS_INFO("crc cal %x", crc_cal);

		return COMM_FAIL;
	}


	for(int i =0; i < LengPayLoad+1; i++)
	{
		pck[i] = sms[i];
	}

	return COMM_OK;
}


uint8_t Controller::waitDriverResponse(driver_cmd_packet_t * payload, uint32_t timeout)
{
	uint8_t result=COMM_FAIL;
    double time_now =  ros::Time::now().toSec();

    if((time_now - remaingtime) < timeout)
    {
        if(serial_->available())
        {
            uint8_t incommingByte; 
            serial_->read((uint8_t *)& incommingByte,1);

			remaingtime =  ros::Time::now().toSec();

            if(flagStart)
            {
                if(len_msg < LengSMS)
                {
					//ROS_INFO("datasms %x", incommingByte);
					//ROS_INFO("len_msg %d", len_msg);
                   datasms[len_msg++] = incommingByte;
                }
                else
                {
					//ROS_INFO_STREAM("Receiv done!");
                    if(incommingByte == Driver_End_SMS)
                    {
						result = parseSMS((uint8_t *)datasms, payload);
                    }
                    if(result == COMM_FAIL)
                    {
                        ROS_INFO_STREAM("SMS Fail ");
						for (size_t i = 0; i < LengSMS; i++)
						{
            				ROS_INFO("%x ", datasms[i]);
						}
            			ROS_INFO("%x ", incommingByte);
						ROS_INFO("len_msg %d", len_msg);
						ros::shutdown();
                    }
					flagStart =0;
					len_msg =0;
					return result;
                }
            }
            else
            {
                if(incommingByte == Driver_Start_SMS)
                {
                    flagStart = 1;
                    //ROS_INFO_STREAM("Start sms");
                }
            }
        }
		return COMM_WAIT;
    }
	else
	{
		remaingtime =  ros::Time::now().toSec();
	}
	flagStart =0;
	len_msg =0;
	ROS_INFO_STREAM("Receiv Timeout!");
    return COMM_TIMEOUT;
}

void Controller::requestCommand(uint8_t cmd)
{
	scription[head_script]= cmd;
	head_script = (head_script + 1) & LEN_Script;
}


void Controller::cmdSetStatusDriver1(bool sstL, bool sstR, bool dirLeft, bool dirRight, uint16_t pwmL, uint16_t pwmR)
{
    DriverInformation1_TypDef sst_driver;

    if(sstL == START_MOTOR)
    {
        if(dirLeft == DIR_ADVANCE)
        {
            sst_driver.ina = FORWARD;
            sst_driver.inb = REVERT;
        }
        else
        {
            sst_driver.ina = REVERT;
            sst_driver.inb = FORWARD;
        }
    }
    else
    {
        sst_driver.ina = REVERT;
        sst_driver.inb = REVERT;
    }

    if(sstR == START_MOTOR)
    {
        if(dirLeft == DIR_ADVANCE)
        {
            sst_driver.inc = REVERT;
            sst_driver.ind = FORWARD;
        }
        else
        {
            sst_driver.inc = FORWARD;
            sst_driver.ind = REVERT;
        }
    }
    else
    {
        sst_driver.inc = REVERT;
        sst_driver.ind = REVERT;
    }

    sst_driver.speedLeft = pwmL;
    sst_driver.speedRight = pwmR;

    send_command(CMD_SET_SST_DRIVER1, (uint8_t *)&sst_driver, sizeof(sst_driver));
}

void Controller::cmdSetStatusDriver2(uint8_t cmd, bool sstL, bool sstR, bool dirLeft, bool dirRight, uint16_t speedrpmleft, uint16_t speedrpmright)
{
    DriverInformation2_TypDef sst_driver;

    if(sstL == START_MOTOR) sst_driver.StatusMotorLeft = 1;
    else 
		sst_driver.StatusMotorLeft = 0;

    if(sstR == START_MOTOR) sst_driver.StatusMotorRight = 1;
    else 
		sst_driver.StatusMotorRight = 0;

	if(dirLeft == DIR_ADVANCE) sst_driver.DirectionMotorLeft = 1;
    else 
		sst_driver.DirectionMotorLeft = 0;

    if(dirRight == DIR_ADVANCE) sst_driver.DirectionMotorRight =1;
    else 
		sst_driver.DirectionMotorRight = 0;

    sst_driver.SpeedRPMLeft = speedrpmleft;
    sst_driver.SpeedRPMRight = speedrpmright;

	switch (cmd)
	{
		case CMD_SET_SST_DRIVER2:
			cycle_request = CMD_REQUEST_ACC_IMU;
			break;
		case CMD_REQUEST_ACC_IMU:
			cycle_request = CMD_REQUEST_SONAR_ENC;
			break;
		case CMD_REQUEST_SONAR_ENC:
			cycle_request = CMD_SET_SST_DRIVER2;
			break;
		default:
			break;
	}

	send_command(cmd, (uint8_t *)&sst_driver, sizeof(sst_driver));
}

void Controller::setPID(double kp, double ki, double kd, double sampletime)
{
	PID_Setup_TypDef pid_setup;

	pid_setup.KP = round(kp*100);
	pid_setup.KI = round(ki*100);
	pid_setup.KD = round(kd*100);
	pid_setup.SampleTime = round(sampletime*1000);
	send_command(CMD_SET_PID, (uint8_t *)&pid_setup, sizeof(pid_setup));
}

void Controller::setLed(void)
{
	uint8_t buff[LengPayLoad]="";

	buff[0] = LedMode;
	buff[1] = LedRed;
	buff[2] = LedBlue;
	buff[3] = LedGreen;

	send_command(CMD_SET_LED, (uint8_t *)buff, LengPayLoad);
}

void Controller::getStatusDriver1(DriverInformation1_TypDef * driverinfor,uint8_t arrdat[], uint8_t len)
{
	uint8_t * pkg = (uint8_t *)driverinfor;

	if(len != LengPayLoad) return;

	for(int i=0; i < len ; i++)
	{
		pkg[i]= arrdat[i];
	}

	if(driverinfor->ina != driverinfor->inb)
	{
		if(driverinfor->ina == REVERT)
		{
			_wheelLeft->setCurrDir(DIR_ADVANCE);
		}
		else
		{
			_wheelLeft->setCurrDir(DIR_BACKOFF);
		}
	}

	if(driverinfor->inc != driverinfor->ind)
	{
		if(driverinfor->inc == REVERT)
		{
			_wheelRight->setCurrDir(DIR_ADVANCE);
		}
		else
		{
			_wheelRight->setCurrDir(DIR_BACKOFF);
		}
	}


	//ROS_INFO_STREAM("Wheel Left		------------------------------------------------------- ");
	//ROS_INFO_STREAM("encoder left " << status_driver.speedLeft);
	_wheelLeft->updateEncoder(driverinfor->speedLeft);
	//ROS_INFO_STREAM("Wheel Right 	------------------------------------------------------- ");
	//ROS_INFO_STREAM("encoder right " << status_driver.speedRight);
	_wheelRight->updateEncoder(driverinfor->speedRight);
}

void Controller::getStatusDriver2(DriverInformation2_TypDef * driverinfor,uint8_t arrdat[], uint8_t len)
{
	uint8_t * pkg = (uint8_t *)driverinfor;

	if(len != LengPayLoad) return;

	for(int i=0; i < len ; i++)
	{
		pkg[i]= arrdat[i];
	}


		if(driverinfor->DirectionMotorLeft == FORWARD)
		{
			_wheelLeft->setCurrDir(DIR_ADVANCE);
		}
		else
		{
			_wheelLeft->setCurrDir(DIR_BACKOFF);
		}


		if(driverinfor->DirectionMotorRight == FORWARD)
		{
			_wheelRight->setCurrDir(DIR_ADVANCE);
		}
		else
		{
			_wheelRight->setCurrDir(DIR_BACKOFF);
		}



	//ROS_INFO_STREAM("Wheel Left		------------------------------------------------------- ");
	//ROS_INFO_STREAM("encoder left " << status_driver.speedLeft);
	_wheelLeft->setCurrSpeedRPM(driverinfor->SpeedRPMLeft);
	//ROS_INFO_STREAM("Wheel Right 	------------------------------------------------------- ");
	//ROS_INFO_STREAM("encoder right " << status_driver.speedRight);
	_wheelRight->setCurrSpeedRPM(driverinfor->SpeedRPMRight);
}

void Controller::get_ACC_IMU(IMU_TypDef * imu_str, uint8_t arrdat[], uint8_t len)
{
	uint8_t * imu_pkg = (uint8_t *)imu_str;

	if(len != LengPayLoad) return;

	for(int i=0; i < len ; i++)
	{
		imu_pkg[i]= arrdat[i];
	}
}

int16_t Controller::get_acceleration_rawX(void){
	int16_t rawX = 0;
	
	rawX = imu.Acceleration_RawX;
	return (rawX);
}

int16_t Controller::get_acceleration_rawY(void){
	int16_t rawY = 0;

	rawY = imu.Acceleration_RawY;
	return (rawY);
}

int16_t Controller::get_acceleration_rawZ(void){
	int16_t rawZ = 0;

	rawZ = imu.Acceleration_RawZ;
	return (rawZ);
}

int Controller::get_acceleration_X(void)
{
	signed int short raw;
	signed int acceleration;
	raw = (signed int short) get_acceleration_rawX();
	acceleration = (signed int)(((signed int)raw) * 3.9);
	return acceleration;
}

int Controller::get_acceleration_Y(void)
{
	signed int short raw;
	signed int acceleration; //= (signed int)(((signed int)raw) * 3.9);
	raw = (signed int short) get_acceleration_rawY();
	acceleration = (signed int)(((signed int)raw) * 3.9);
	return acceleration;
}

int Controller::get_acceleration_Z(void)
{
	signed int short raw;
	signed int acceleration; // = (signed int)(((signed int)raw) * 3.9);
	raw = (signed int short) get_acceleration_rawZ();
	acceleration = (signed int)(((signed int)raw) * 3.9);
	return acceleration;
}

int Controller::getPitch(void)
{				
	double result;
	result = atan(get_acceleration_Y()/sqrt(pow(get_acceleration_X(), 2) + pow(get_acceleration_Z(), 2)));
	result = (result * 180) / PI;
	return result;
}

int Controller::getRoll(void)
{
	double result;
	result = atan(-get_acceleration_X()/get_acceleration_Z());
	result = (result * 180) / PI;
	return result;
}

void Controller::getEncSonar(ENC_SONAR_TypDef * enc_sonar, uint8_t arrdat[], uint8_t len)
{
	uint8_t * enc_sonar_pkg = (uint8_t *)enc_sonar;

	if(len != LengPayLoad) return;

	for(int i=0; i < len ; i++)
	{
		enc_sonar_pkg[i]= arrdat[i];
	}
}

void Controller::spin(void)
{
	static double time_begin = ros::Time::now().toSec();
	double time_now = ros::Time::now().toSec();
	double remaining = time_now - time_begin;
	uint8_t result = COMM_TIMEOUT;
	uint8_t cmd;
	driver_cmd_packet_t driver_packet;

	remaining = roundf(1000*remaining)/1000;
	if(remaining >= TimeRequest)
	{
		//ROS_INFO_STREAM("TimeRequest " << remaining);
		time_begin = ros::Time::now().toSec();
		//cmdSetStatusDriver1(START_MOTOR, START_MOTOR, _wheelLeft->getDesiredDir(), _wheelRight->getDesiredDir(), _wheelLeft->getPWM(), _wheelRight->getPWM());

		if(head_script != tail_script)
		{
			cmd = scription[tail_script];
			tail_script = (tail_script + 1) & LEN_Script;
		}
		else
		{
			cmd = cycle_request;
		}

		switch (cmd)
		{
		case CMD_SET_SST_DRIVER2:
		case CMD_REQUEST_ACC_IMU:
		case CMD_REQUEST_SONAR_ENC:
		case CMD_RP_PIDLEFT:
		case CMD_RP_PIDRIGHT:
		case CMD_REQUSET_INFORDRIVER1:
		case CMD_REQUSET_INFORDRIVER2:
		case CMD_RESET_IMU:
		case CMD_RESET_ENCODER:
			cmdSetStatusDriver2(cmd,
						START_MOTOR, START_MOTOR,
						_wheelLeft->getDesiredDir(), _wheelRight->getDesiredDir(), 
						_wheelLeft->PIDGetSpeedRPMDesired(), _wheelRight->PIDGetSpeedRPMDesired());
			break;
		case CMD_SET_SST_DRIVER1:
			cmdSetStatusDriver1(START_MOTOR, START_MOTOR, 
					_wheelLeft->getDesiredDir(), _wheelRight->getDesiredDir(), 
					_wheelLeft->getPWM(), _wheelRight->getPWM());
			break;
		case CMD_SET_PID:
			setPID(_wheelLeft->GetKp(), _wheelLeft->GetKi(), _wheelLeft->GetKd(), _wheelLeft->GetSampleTime());
			break;
		case CMD_SET_LED:
			setLed();
			break;
		default:
			break;
		}

	}

	result = waitDriverResponse(&driver_packet , 1);

	if(result == COMM_OK)
	{
		switch (driver_packet.cmd_flag)
		{
		case CMD_FAULT_CLEAR:

			break;
		case CMD_REQUSET_INFORDRIVER1:
		case CMD_SET_SST_DRIVER1:
			getStatusDriver1(&driverinfor1,driver_packet.data, LengPayLoad);
			break;
		case CMD_REQUSET_INFORDRIVER2:
		case CMD_SET_SST_DRIVER2:
			getStatusDriver2(&driverinfor2,driver_packet.data, LengPayLoad);
			break;
		case CMD_RESET_IMU:

			break;
		case CMD_REQUEST_ACC_IMU:
			get_ACC_IMU(&imu, driver_packet.data, LengPayLoad);
			break;
		case CMD_REQUEST_SONAR_ENC:
			getEncSonar(&encoder_sonar, driver_packet.data, LengPayLoad);
			break;
		case CMD_SET_LED:
			setLed();
			break;
		default:
			break;
		}
		debug();
	}

	// PIDRegulate();
}

unsigned int Controller::getWheelspanMM() const {
	return _wheelspanMM;
}

unsigned int Controller::setWheelspanMM(unsigned int wheelspanMM) {
	_wheelspanMM=wheelspanMM;
	return getWheelspanMM();
}

unsigned char Controller::getSwitchMotorsStat() const {
	return _switchMotorsStat;
}

unsigned char Controller::setSwitchMotorsStat(unsigned char switchMotorsStat) {
	if(MOTORS_FB<=switchMotorsStat && switchMotorsStat<=MOTORS_BF)
		_switchMotorsStat=switchMotorsStat;
	return getSwitchMotorsStat();
}

unsigned char Controller::switchMotors() {
	if(getSwitchMotorsStat()==MOTORS_FB) setSwitchMotorsStat(MOTORS_BF);
	else setSwitchMotorsStat(MOTORS_FB);
	MotorWheel* temp=_wheelRight;
	_wheelRight=_wheelLeft;
	_wheelLeft=temp;
	return getSwitchMotorsStat();
}

unsigned char Controller::switchMotorsReset() {
	if(getSwitchMotorsStat()==MOTORS_BF) switchMotors();
	return getSwitchMotorsStat();
}

unsigned char Controller::getCarStat() const {
	return _carStat;
}

unsigned char Controller::setCarStat (unsigned char carStat) {
    if(STAT_UNKNOWN<=carStat && carStat<=STAT_UPPERRIGHT)
        return _carStat=carStat;
    else
        return STAT_UNKNOWN;
}


unsigned int Controller::getRadiusMM() const {
	switch(getCarStat()) {
		case STAT_ADVANCE:
		case STAT_BACKOFF:
			return 0; break;
		case STAT_ROTATELEFT:
		case STAT_ROTATERIGHT:
			return getWheelspanMM()>>1; break;
	}
	return _radiusMM;
}

unsigned int Controller::setRadiusMM(unsigned int radiusMM) {
	_radiusMM=radiusMM;
	return getRadiusMM();
}

unsigned int Controller::wheelRightSetSpeedMMPS(unsigned int speedMMPS,bool dir) {
	return _wheelRight->setSpeedMMPS(speedMMPS,dir);
}

unsigned int Controller::wheelRightGetSpeedMMPS() const {
	return _wheelRight->getSpeedMMPS();
}

unsigned int Controller::wheelLeftSetSpeedMMPS(unsigned int speedMMPS,bool dir) {
	return _wheelLeft->setSpeedMMPS(speedMMPS,dir);
}

unsigned int Controller::wheelLeftGetSpeedMMPS() const {
	return _wheelLeft->getSpeedMMPS();
}


unsigned int Controller::setMotorAll(unsigned int speedMMPS,bool dir) {
	wheelLeftSetSpeedMMPS(speedMMPS,dir);
	wheelRightSetSpeedMMPS(speedMMPS,dir);
	return wheelRightGetSpeedMMPS();
}

unsigned int Controller::getCarSpeedMMPS() const {
	unsigned int speedMMPSL=wheelLeftGetSpeedMMPS();
	unsigned int speedMMPSR=wheelRightGetSpeedMMPS();
	return (speedMMPSL+speedMMPSR)>>1;
}

unsigned int Controller::setMotorAllStop() {
	return setMotorAll(0,DIR_ADVANCE);
}

unsigned int Controller::setMotorAllAdvance(unsigned int speedMMPS) {
	return setMotorAll(speedMMPS,DIR_ADVANCE);
}

unsigned int Controller::setMotorAllBackoff(unsigned int speedMMPS) {
	return setMotorAll(speedMMPS,DIR_BACKOFF);
}

unsigned int Controller::setCarStop() {
	setCarStat(STAT_STOP);
	return setMotorAll(0,DIR_ADVANCE);
}

unsigned int Controller::setCarAdvanceBase(unsigned int speedMMPSL,unsigned int speedMMPSR) {
	wheelLeftSetSpeedMMPS(speedMMPSL, DIR_ADVANCE);
	wheelRightSetSpeedMMPS(speedMMPSR, DIR_ADVANCE);
	return getCarSpeedMMPS();
}

unsigned int Controller::setCarBackoffBase(unsigned int speedMMPSL,unsigned int speedMMPSR) {
	wheelLeftSetSpeedMMPS(speedMMPSL,DIR_ADVANCE);
	wheelRightSetSpeedMMPS(speedMMPSR,DIR_BACKOFF);
	return getCarSpeedMMPS();
}

unsigned int Controller::setCarAdvance(unsigned int speedMMPS) {
	setCarStat(STAT_ADVANCE);
	return setCarAdvanceBase(speedMMPS,speedMMPS);
}
unsigned int Controller::setCarBackoff(unsigned int speedMMPS) {
	setCarStat(STAT_BACKOFF);
	return setCarBackoffBase(speedMMPS,speedMMPS);
}


unsigned int Controller::setCarRotateLeft(unsigned int speedMMPS) {
	setCarStat(STAT_ROTATELEFT);
	return setMotorAllBackoff(speedMMPS);
}

unsigned int Controller::setCarRotateRight(unsigned int speedMMPS) {
	setCarStat(STAT_ROTATERIGHT);
	return setMotorAllAdvance(speedMMPS);
}

unsigned int Controller::setCarArcBase(unsigned int speedMMPS,unsigned int radiusMM) {
	unsigned int delta=(int)((float)getWheelspanMM()/(radiusMM<<1)*speedMMPS);
	unsigned int V1=speedMMPS-delta;
	unsigned int V2=speedMMPS+delta;

	setRadiusMM(radiusMM);
	switch(getCarStat()) {
		case STAT_UPPERLEFT:
			setCarAdvanceBase(V1,V2); break;
		case STAT_LOWERLEFT:
			setCarBackoffBase(V1,V2); break;
		case STAT_UPPERRIGHT:
			setCarAdvanceBase(V2,V1); break;
		case STAT_LOWERRIGHT:
			setCarBackoffBase(V2,V1); break;
	}
	return getCarSpeedMMPS();
}

unsigned int Controller::setCarUpperLeft(unsigned int speedMMPS,unsigned int radiusMM) {
	setCarStat(STAT_UPPERLEFT);
	return setCarArcBase(speedMMPS,radiusMM);
}

unsigned int Controller::setCarLowerLeft(unsigned int speedMMPS,unsigned int radiusMM) {
	setCarStat(STAT_LOWERLEFT);
	return setCarArcBase(speedMMPS,radiusMM);
}

unsigned int Controller::setCarLowerRight(unsigned int speedMMPS,unsigned int radiusMM) {
	setCarStat(STAT_LOWERRIGHT);
	return setCarArcBase(speedMMPS,radiusMM);
}

unsigned int Controller::setCarUpperRight(unsigned int speedMMPS,unsigned int radiusMM) {
	setCarStat(STAT_UPPERRIGHT);
	return setCarArcBase(speedMMPS,radiusMM);
}

unsigned int Controller::setCarSpeedMMPS(unsigned int speedMMPS,unsigned int ms) {
	unsigned int carStat=getCarStat();
	unsigned int currSpeed=getCarSpeedMMPS();

	unsigned int (Controller::*carAction)(unsigned int speedMMPS);
	switch(carStat) {
		case STAT_ADVANCE:
			 carAction=&Controller::setCarAdvance; break;
		case STAT_BACKOFF:
			 carAction=&Controller::setCarBackoff; break;
		case STAT_ROTATELEFT:
			 carAction=&Controller::setCarRotateLeft; break;
		case STAT_ROTATERIGHT:
			 carAction=&Controller::setCarRotateRight; break;
		case STAT_UPPERLEFT:
		case STAT_LOWERLEFT:
		case STAT_LOWERRIGHT:
		case STAT_UPPERRIGHT:
			return setCarSpeedMMPSArc(speedMMPS,getRadiusMM(),ms);break;
		default:
			return currSpeed; break;
	}

	// if(ms<100 || abs(speedMMPS-currSpeed)<10) {
	// 	return (this->*carAction)(speedMMPS);
	// }

	// for(int time=0,speed=currSpeed;time<=ms;time+=50) {
	// 	speed=map(time,0,ms,currSpeed,speedMMPS);
	// 	(this->*carAction)(speed);
	 //	delayMS(50);
	 //}

	(this->*carAction)(speedMMPS);
	return getCarSpeedMMPS();
}

unsigned int Controller::setCarSpeedMMPSArc(unsigned int speedMMPS,unsigned int radiusMM,unsigned int ms) {
	unsigned int carStat=getCarStat();
	unsigned int currSpeed=getCarSpeedMMPS();

	unsigned int (Controller::*carAction)(unsigned int speedMMPS,unsigned int radiusMM);
	
	switch(carStat) {
		case STAT_ADVANCE:
		case STAT_BACKOFF:
		case STAT_ROTATELEFT:
		case STAT_ROTATERIGHT:
			return setCarSpeedMMPS(speedMMPS,ms); break;
		case STAT_UPPERLEFT:
			carAction=&Controller::setCarUpperLeft; break;
		case STAT_LOWERLEFT:
			carAction=&Controller::setCarLowerLeft; break;
		case STAT_LOWERRIGHT:
			carAction=&Controller::setCarLowerRight; break;
		case STAT_UPPERRIGHT:
			carAction=&Controller::setCarUpperRight; break;
		default:
			return currSpeed; break;
	}

	// if(ms<100 || abs(speedMMPS-currSpeed)<10) {
	// 	return (this->*carAction)(speedMMPS,radiusMM);
	// }

	// for(int time=0,speed=currSpeed;time<=ms;time+=50) {
	// 	speed=map(time,0,ms,currSpeed,speedMMPS);
	// 	(this->*carAction)(speed,radiusMM);
	// 	delayMS(50);
	// }

	(this->*carAction)(speedMMPS,radiusMM);
	return getCarSpeedMMPS();
}


unsigned int Controller::setCarSlow2Stop(unsigned int ms) {
	unsigned char carStat=getCarStat();
	//if(STAT_UNKNOWN<=STAT_UPPERLEFT || carStat<=STAT_LOWERRIGHT) {
	if(STAT_UPPERLEFT<=carStat && carStat<=STAT_UPPERRIGHT) {
		return setCarSpeedMMPSArc(0,getRadiusMM(),ms);
	}
	return setCarSpeedMMPS(0,ms);
}
 

bool Controller::PIDEnable(float kc,float taui,float taud,double interval) {
	return _wheelLeft->PIDEnable(kc,taui,taud,interval) && 
			_wheelRight->PIDEnable(kc,taui,taud,interval);
}
bool Controller::PIDRegulate() {
	 if(_wheelLeft->PIDRegulate() && _wheelRight->PIDRegulate())
	 {
		return true;
	 }
	return false;
}


void Controller::debug(void)
{
	static double time_begin = ros::Time::now().toSec();
	double time_now = ros::Time::now().toSec();

	if(time_now - time_begin > 1)
	{
		time_begin = ros::Time::now().toSec();

	 	ROS_INFO("Desired speed Wheel Left: %d mm/s", _wheelLeft->getSpeedDesiredMMSP());
	 	ROS_INFO("Desired speed Wheel Right: %d mm/s",  _wheelRight->getSpeedDesiredMMSP());
	 	ROS_INFO("Current speed Wheel Left: %d mm/s", _wheelLeft->getSpeedMMPS());
	 	ROS_INFO("Current speed Wheel Right: %d mm/s", _wheelRight->getSpeedMMPS());
	 	ROS_INFO("Encoder Wheel Left: %d ", encoder_sonar.EncoderLeft);
	 	ROS_INFO("Encoder Wheel Right: %d ", encoder_sonar.EncoderRight);
	 	ROS_INFO("Distance obstacle: %d cm", encoder_sonar.Distance);
		ROS_INFO_STREAM("IMU Pitch: " << getPitch());
		ROS_INFO_STREAM("IMU Roll: " << getRoll());
		ROS_INFO_STREAM("\r\n");
	}
}

