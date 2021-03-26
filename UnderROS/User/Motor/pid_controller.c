#include <pid_controller.h>



/*Constructor (...)*********************************************************
 *    To allow backwards compatability for v1.1, or for people that just want
 *    to use Proportional on Error without explicitly saying so
 ***************************************************************************/

void PID_config(PID_TypDef* pid_t, int input, int output, int setpoint, double kp, double ki, double kd)
{
    pid_t->Output = output;
    pid_t->Input = input;
    pid_t->Setpoint = setpoint;
    pid_t->inAuto = true;

    PID_SetOutputLimits(pid_t,0,22363);				//default output limit corresponds to
												//the arduino pwm limits

    pid_t->SampleTime = 10;							//default Controller Sample Time is 0.01 seconds

    PID_SetTunings(pid_t,kp, ki, kd);

    pid_t->lastTime = millis();
}

void PID_Reset(PID_TypDef* pid_t)
{
    pid_t->Output = 0;
    pid_t->Input = 0;
    pid_t->Setpoint = 0;
    pid_t->inAuto = true;

    PID_SetOutputLimits(pid_t,0,22363);				//default output limit corresponds to
												//the arduino pwm limits

    pid_t->SampleTime = 10;							//default Controller Sample Time is 0.01 seconds

    PID_SetTunings(pid_t,0, 0, 0);

		pid_t->outputSum = pid_t->lastInput =0;

    pid_t->lastTime = millis();
}

bool PID_Compute(PID_TypDef* pid_t)
{
    if(pid_t->inAuto)  return false;
    
    if(millis() - pid_t->lastTime  >= pid_t->SampleTime)
    {
        int error = pid_t->Setpoint - pid_t->Input;
        int dInput = pid_t->Input - pid_t->lastInput;
        pid_t->outputSum += pid_t->Ki*error;

        if(pid_t->outputSum > pid_t->outMax) pid_t->outputSum= pid_t->outMax;
        else if(pid_t->outputSum < pid_t->outMin) pid_t->outputSum= pid_t->outMin;

        /*Add Proportional on Error, if P_ON_E is specified*/
        int output;
        if(pid_t->pOnE) output = pid_t->Kp * error;
        else output = 0;

        /*Compute Rest of PID Output*/
        output += pid_t->outputSum - pid_t->Kd * dInput;

        if(output > pid_t->outMax) output = pid_t->outMax;
        else if(output < pid_t->outMin) output = pid_t->outMin;

        pid_t->Output = output;

        /*Remember some variables for next time*/
        pid_t->lastInput = pid_t->Input;
        pid_t->lastTime = millis();
        return true;
    }
		return false;
}



/* SetTunings(...)*************************************************************
 * Set Tunings using the last-rembered POn setting
 ******************************************************************************/
void PID_SetTunings(PID_TypDef* pid_t, double kp, double ki, double kd)
{
   if (kp<0 || ki<0 || kd<0) return;

   pid_t->pOn = P_ON_E;
   pid_t->pOnE = pid_t->pOn == P_ON_E;

   pid_t->dispKp = kp; pid_t->dispKi = ki; pid_t->dispKd = kd;

		double SampleTimeInSec = ((double)pid_t->SampleTime)/1000;
		 pid_t->Kp = kp;
		 pid_t->Ki = ki * SampleTimeInSec;
		 pid_t->Kd = kd / SampleTimeInSec;	
}

/* SetSampleTime(...) *********************************************************
 * sets the period, in Milliseconds, at which the calculation is performed
 ******************************************************************************/
void PID_SetSampleTime(PID_TypDef* pid_t,int NewSampleTime)
{
	 pid_t->lastTime = millis(); 

   if (NewSampleTime > 0)
   {
      double ratio  = (double)NewSampleTime
                      / (double)pid_t->SampleTime;
      pid_t->Ki *= ratio;
      pid_t->Kd /= ratio;
      pid_t->SampleTime = (unsigned long)NewSampleTime;
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
void PID_SetOutputLimits(PID_TypDef* pid_t, int Min, int Max)
{
   if(Min >= Max) return;
   pid_t->outMin = Min;
   pid_t->outMax = Max;

   if(pid_t->inAuto)
   {
	   if(pid_t->Output > pid_t->outMax) pid_t->Output = pid_t->outMax;
	   else if(pid_t->Output < pid_t->outMin) pid_t->Output = pid_t->outMin;

	   if(pid_t->outputSum > pid_t->outMax) pid_t->outputSum= pid_t->outMax;
	   else if(pid_t->outputSum < pid_t->outMin) pid_t->outputSum= pid_t->outMin;
   }
}


/* SetMode(...)****************************************************************
 * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
 * when the transition from manual to auto occurs, the controller is
 * automatically initialized
 ******************************************************************************/
void PID_SetMode(PID_TypDef* pid_t,int Mode)
{
	if (Mode!=0 && !pid_t->inAuto)
	{	//we were in manual, and we just got set to auto.
		//reset the controller internals
		PID_Initialize(pid_t);
	}
	pid_t->inAuto = (Mode!=0);
}

/* Initialize()****************************************************************
 *	does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.
 ******************************************************************************/
void PID_Initialize(PID_TypDef* pid_t)
{
   pid_t->outputSum = pid_t->Output;
   pid_t->lastInput = pid_t->Input;
   if(pid_t->outputSum > pid_t->outMax) pid_t->outputSum = pid_t->outMax;
   else if(pid_t->outputSum < pid_t->outMin) pid_t->outputSum = pid_t->outMin;
}




