#include "PID_Beta6.h"
//#include "fuzzy_table.h"







/* Standard Constructor (...)***********************************************
 *    constructor used by most users.  the parameters specified are those for
 * for which we can't set up reliable defaults, so we need to have the user
 * set them.
 ***************************************************************************/ 
void PID_SC(PID_Parameter_t* param, int Input, int Output, int Setpoint, float Kc, float TauI, float TauD)
{
	
  ConstructorCommon(param,Input, Output, Setpoint, Kc, TauI, TauD);  
  param->UsingFeedForward = false;
  Reset(param);
}





/* Overloaded Constructor(...)**********************************************
 *    This one is for more advanced users.  it's essentially the same as the
 * standard constructor, with one addition.  you can link to a Feed Forward bias,
 * which lets you implement... um.. Feed Forward Control.  good stuff.
 ***************************************************************************/
void PID_OC(PID_Parameter_t* param, int Input, int Output, int Setpoint, int *FFBias, float Kc, float TauI, float TauD)
{

  ConstructorCommon(param,Input, Output, Setpoint, Kc, TauI, TauD);  
  param->UsingFeedForward = true;			  //tell the controller that we'll be using an external
  param->myBias = FFBias;                              //bias, and where to find it
  Reset(param);
}


/* ConstructorCommon(...)****************************************************
 *    Most of what is done in the two constructors is the same.  that code
 * was put here for ease of maintenance and (minor) reduction of library size
 ****************************************************************************/
void ConstructorCommon(PID_Parameter_t* param, int Input, int Output, int Setpoint, float Kc, float TauI, float TauD)
{
  SetInputLimits(param,param->InputMin, param->InputMax);		//default the limits to the 
  SetOutputLimits(param,param->OutputMin, param->OutputMax);		//full ranges of the I/O

  param->tSample = Controller_Sample_Time;			//default Controller Sample Time is 1 second

  SetTunings(param,Kc, TauI, TauD);

  param->nextCompTime = millis();
  param->inAuto = true;
  param->myOutput = Output;
  param->myInput = Input;
  param->mySetpoint = Setpoint;

  param->Err =0;
	param->lastErr =0;
	param->prevErr = 0;
}
		
		
/* SetInputLimits(...)*****************************************************
 *	I don't see this function being called all that much (other than from the
 *  constructor.)  it needs to be here so we can tell the controller what it's
 *  input limits are, and in most cases the 0-1023 default should be fine.  if
 *  there's an application where the signal being fed to the controller is
 *  outside that range, well, then this function's here for you.
 **************************************************************************/
void SetInputLimits(PID_Parameter_t* param,int INMin, int INMax)
{
	//after verifying that mins are smaller than maxes, set the values
	if(INMin >= INMax) return;
	
	
	param->inMin = INMin;
	param->inSpan = INMax - INMin;
}
		

/* SetOutputLimits(...)****************************************************
 *     This function will be used far more often than SetInputLimits.  while
 *  the input to the controller will generally be in the 0-1023 range (which is
 *  the default already,)  the output will be a little different.  maybe they'll
 *  be doing a time window and will need 0-8000 or something.  or maybe they'll
 *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
 *  here.
 **************************************************************************/
void SetOutputLimits(PID_Parameter_t* param,int OUTMin, int OUTMax)
{
	//after verifying that mins are smaller than maxes, set the values
	if(OUTMin >= OUTMax) return;

	param->outMin = OUTMin;
	param->outSpan = OUTMax - OUTMin;
}


/* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted. 
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/
void SetTunings(PID_Parameter_t* param,const float Kc,const float TauI,const float TauD)
{
	float tSampleInSec= 0.0;
	float tempTauR= 0.0;
	
	//verify that the tunings make sense
	if (Kc == 0 || TauI < 0 || TauD < 0) return;

	//we're going to do some funky things to the input numbers so all
	//our math works out, but we want to store the numbers intact
	//so we can return them to the user when asked.
	param->P_Param = Kc;
	param->I_Param = TauI;
	param->D_Param = TauD;

	//convert Reset Time into Reset Rate, and compensate for Calculation frequency
	tSampleInSec = ((float)param->tSample/1000);
	if (TauI == 0) 
		tempTauR = 0.0;
	else 
		tempTauR = (1/TauI) * tSampleInSec;

	
	param->kc = Kc;
	param->taur = tempTauR;
	param->taud = TauD/tSampleInSec;

	param->cof_A = param->kc*(1 + param->taur + param->taud);
	param->cof_B = param->kc*(1 + 2 * param->taud);
	param->cof_C = param->kc*param->taud;
}



/* Reset()*********************************************************************
 *	does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.  this shouldn't have to be called from the
 *  outside. In practice though, it is sometimes helpful to start from scratch,
 *  so it was made publicly available
 ******************************************************************************/
void Reset(PID_Parameter_t* param)
{

	if(param->UsingFeedForward)
	  param->bias = (*param->myBias - param->outMin) / param->outSpan;
	else
	  param->bias = (param->myOutput - param->outMin) / param->outSpan;

}




/* SetMode(...)****************************************************************
 * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
 * when the transition from manual to auto occurs, the controller is
 * automatically initialized
 ******************************************************************************/
void SetMode(PID_Parameter_t* param, int Mode)
{
	if (Mode!=0 && !param->inAuto)
	{	//we were in manual, and we just got set to auto.
		//reset the controller internals
		Reset(param);
	}
	param->inAuto = (Mode!=0);
}



/* SetSampleTime(...)*******************************************************
 * sets the frequency, in Milliseconds, with which the PID calculation is performed	
 ******************************************************************************/
void SetSampleTime(PID_Parameter_t* param,int NewSampleTime)
{
	if (NewSampleTime > 0)
	{ 
		//convert the time-based tunings to reflect this change
		param->taur *= ((float)NewSampleTime)/((float) param->tSample);
		param->taud *= ((float)NewSampleTime)/((float) param->tSample);
		param->tSample = (unsigned long)NewSampleTime;

		param->cof_A = param->kc * (1 + param->taur + param->taud);
		param->cof_B = param->kc * (1 + 2 * param->taud);
		param->cof_C = param->kc * param->taud;
	}
}


/* Compute() **********************************************************************
 *     This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   pid Output needs to be computed
 *
 *  Some notes for people familiar with the nuts and bolts of PID control:
 *  - I used the Ideal form of the PID equation.  mainly because I like IMC
 *    tunings.  lock in the I and D, and then just vary P to get more 
 *    aggressive or conservative
 *
 *  - While this controller presented to the outside world as being a Reset Time
 *    controller, when the user enters their tunings the I term is converted to
 *    Reset Rate.  I did this merely to avoid the div0 error when the user wants
 *    to turn Integral action off.
 *    
 *  - Derivative on Measurement is being used instead of Derivative on Error.  The
 *    performance is identical, with one notable exception.  DonE causes a kick in
 *    the controller output whenever there's a setpoint change. DonM does not.
 *
 *  If none of the above made sense to you, and you would like it to, go to:
 *  http://www.controlguru.com .  Dr. Cooper was my controls professor, and is
 *  gifted at concisely and clearly explaining PID control
 *********************************************************************************/
void PID_Compute(PID_Parameter_t* param)
{
	param->justCalced=false;
	if (!param->inAuto) return; //if we're in manual just leave;

	unsigned long now = millis();

	//millis() wraps around to 0 at some point.  depending on the version of the 
	//Arduino Program you are using, it could be in 9 hours or 50 days.
	//this is not currently addressed by this algorithm.
	
									
	//...Perform PID Computations if it's time...
//	if (now>=param->nextCompTime)							
//	{
  
		param->Err = param->mySetpoint - param->myInput;
		//if we're using an external bias (i.e. the user used the 
		//overloaded constructor,) then pull that in now
		if(param->UsingFeedForward)
		{
			param->bias = *param->myBias - param->outMin;
		}


		// perform the PID calculation.  
		//float output = bias + kc * ((Err - lastErr)+ (taur * Err) + (taud * (Err - 2*lastErr + prevErr)));
//		noInterrupts();
		int output = param->bias + (param->cof_A * param->Err - param->cof_B * param->lastErr + param->cof_C * param->prevErr);
//		interrupts();

		//make sure the computed output is within output constraints
		if (output < -param->outSpan) output = -param->outSpan;
		else if (output > param->outSpan) output = param->outSpan;
		

		param->prevErr = param->lastErr;
		param->lastErr = param->Err;


		//scale the output from percent span back out to a real world number
			param->myOutput = output;

//		param->nextCompTime += param->tSample;				// determine the next time the computation
//		if(param->nextCompTime < now) param->nextCompTime = now + param->tSample;	// should be performed	

		param->justCalced=true;  //set the flag that will tell the outside world that the output was just computed

//	}								
}

