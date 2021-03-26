#include <pid_controller.h>



/*Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
PID::PID(int* Input, int* Output, int* Setpoint,
        double Kp, double Ki, double Kd, int POn)
{
    myOutput = Output;
    myInput = Input;
    mySetpoint = Setpoint;
    inAuto = true;

    PID::SetOutputLimits(0,22363);				//default output limit corresponds to
												//the arduino pwm limits

    SampleTime = 0.01;							//default Controller Sample Time is 0.01 seconds

    PID::SetTunings(Kp, Ki, Kd, POn);

    //lastTime = ros::Time::now().toSec() - SampleTime;
}

/*Constructor (...)*********************************************************
 *    To allow backwards compatability for v1.1, or for people that just want
 *    to use Proportional on Error without explicitly saying so
 ***************************************************************************/

PID::PID(int* Input, int* Output, int* Setpoint,
        double Kp, double Ki, double Kd)
    :PID::PID(Input, Output, Setpoint, Kp, Ki, Kd, P_ON_E)
{

}

bool PID::Compute()
{
    if(inAuto)  return false;

	double now = ros::Time::now().toSec();

    now = (now*100)/100;
	double tim = now - lastTime;
    
    if(tim >= SampleTime)
    {
        double error = *mySetpoint - *myInput;
        double dInput = *myInput - lastInput;
        outputSum += ki*error;

        if(outputSum > outMax) outputSum= outMax;
        else if(outputSum < outMin) outputSum= outMin;

        /*Add Proportional on Error, if P_ON_E is specified*/
        double output;
        if(pOnE) output = kp * error;
        else output = 0;

        /*Compute Rest of PID Output*/
        output += outputSum - kd * dInput;

        if(output > outMax) output = outMax;
        else if(output < outMin) output = outMin;

        *myOutput = output;

        /*Remember some variables for next time*/
        lastInput = *myInput;
        lastTime = now;
        return true;
    }
}

/* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted.
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/
void PID::SetTunings(double Kp, double Ki, double Kd, int POn)
{
   if (Kp<0 || Ki<0 || Kd<0) return;

   pOn = POn;
   pOnE = POn == P_ON_E;

   dispKp = Kp; dispKi = Ki; dispKd = Kd;

   kp = Kp;
   ki = Ki * SampleTime;
   kd = Kd / SampleTime;
}

/* SetTunings(...)*************************************************************
 * Set Tunings using the last-rembered POn setting
 ******************************************************************************/
void PID::SetTunings(double Kp, double Ki, double Kd){
    SetTunings(Kp, Ki, Kd, pOn); 
}

/* SetSampleTime(...) *********************************************************
 * sets the period, in Milliseconds, at which the calculation is performed
 ******************************************************************************/
void PID::SetSampleTime(double NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      SampleTime = NewSampleTime;
   }
}

/* SetOutputLimits(...)****************************************************
 *     This function will be used far more often than SetInputLimits.  while
 *  the input to the controller will generally be in the 0-1023 range (which is
 *  the default already,)  the output will be a little different.  maybe they'll
 *  be doing a time window and will need 0-8000 or something.  or maybe they'll
 *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
 *  here.
 **************************************************************************/
void PID::SetOutputLimits(int Min, int Max)
{
   if(Min >= Max) return;
   outMin = Min;
   outMax = Max;

   if(inAuto)
   {
	   if(*myOutput > outMax) *myOutput = outMax;
	   else if(*myOutput < outMin) *myOutput = outMin;

	   if(outputSum > outMax) outputSum= outMax;
	   else if(outputSum < outMin) outputSum= outMin;
   }
}


/* SetMode(...)****************************************************************
 * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
 * when the transition from manual to auto occurs, the controller is
 * automatically initialized
 ******************************************************************************/
void PID::SetMode(int Mode)
{
	if (Mode!=0 && !inAuto)
	{	//we were in manual, and we just got set to auto.
		//reset the controller internals
		PID::Initialize();
	}
	inAuto = (Mode!=0);
}

/* Initialize()****************************************************************
 *	does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.
 ******************************************************************************/
void PID::Initialize()
{
   outputSum = *myOutput;
   lastInput = *myInput;
   if(outputSum > outMax) outputSum = outMax;
   else if(outputSum < outMin) outputSum = outMin;
}


/* Status Funcions*************************************************************
 * Just because you set the Kp=-1 doesn't mean it actually happened.  these
 * functions query the internal state of the PID.  they're here for display
 * purposes.  this are the functions the PID Front-end uses for example
 ******************************************************************************/
double PID::GetKp(){ return  dispKp; }
double PID::GetKi(){ return  dispKi;}
double PID::GetKd(){ return  dispKd;}
double PID::GetSampleTime(){ return  SampleTime;}
int PID::GetMode(){ return  inAuto ? AUTOMATIC : MANUAL;}
// int PID::GetDirection(){ return controllerDirection;}

void  PID::debuggerPID()
{
    double p,i,d;
    uint8_t mode= GetMode();

    p = GetKp();
    i = GetKi();
    d = GetKd();

    ROS_INFO_STREAM("PID Infor KP " << p << " KI " << i << " KD " << d);
    
    if(mode == AUTOMATIC)
        ROS_INFO_STREAM("PID AUTOMATIC ");
    
    ROS_INFO_STREAM("Sample time:  " << SampleTime);
    ROS_INFO_STREAM("Set point:  " << *mySetpoint );
    ROS_INFO_STREAM("outputSum:  " << outputSum);
    ROS_INFO_STREAM("lastInput:  " << lastInput);
}
