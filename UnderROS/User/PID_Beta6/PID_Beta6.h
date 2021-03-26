#ifndef __PID_BETA6_H
#define __PID_BETA6_H


#include "mylib.h"

#define AUTO	1
#define MANUAL	0
#define LIBRARY_VERSION	0.6


#define Input_Min 	-16
#define Input_Max 	16

#define Output_Min 	0
#define Output_Max 	3000


// Wheel: Vmax 43223.07893; (165.1*(250Pi/3))

#define Controller_Sample_Time (uint16_t)100

typedef struct __PID_Parameter_t
{
	//scaled, tweaked parameters we'll actually be using
	float kc;                    // * (P)roportional Tuning Parameter
	float taur;                  // * (I)ntegral Tuning Parameter
	float taud;                  // * (D)erivative Tuning Parameter

	float cof_A;
	float cof_B;
	float cof_C;

	//nice, pretty parameters we'll give back to the user if they ask what the tunings are
	float P_Param;
	float I_Param;
	float D_Param;


	int myInput;					// * Pointers to the Input, Output, and Setpoint variables
	int myOutput;				//   This creates a hard link between the variables and the 
	int mySetpoint;			//   PID, freeing the user from having to constantly tell us
												//   what these values are.  with pointers we'll just know.

	int *myBias;					// * Pointer to the External FeedForward bias, only used 
												//   if the advanced constructor is used
	bool UsingFeedForward;		// * internal flag that tells us if we're using FeedForward or not

	unsigned long nextCompTime;    // * Helps us figure out when the PID Calculation needs to
																 //   be performed next
																 //   to determine when to compute next
	unsigned long tSample;        // * the frequency, in milliseconds, with which we want the
																//   the PID calculation to occur.
	bool inAuto;                  // * Flag letting us know if we are in Automatic or not

																 //   the derivative required for the D term
	//float accError;              // * the (I)ntegral term is based on the sum of error over
																 //   time.  this variable keeps track of that
	float bias;                    // * the base output from which the PID operates

	int Err;
	int lastErr;
	int prevErr;

	float inMin, inSpan;         // * input and output limits, and spans.  used convert
	float outMin, outSpan;       //   real world numbers into percent span, with which
															 //   the PID algorithm is more comfortable.

	bool justCalced;			// * flag gets set for one cycle after the pid calculates


	int InputMin;
	int InputMax;

	int OutputMin;
	int OutputMax;
	
}PID_Parameter_t;





void PID_SC(PID_Parameter_t* param, int Input, int Output, int Setpoint, float Kc, float TauI, float TauD);
void PID_OC(PID_Parameter_t* param, int Input, int Output, int Setpoint, int *FFBias, float Kc, float TauI, float TauD);

void ConstructorCommon(PID_Parameter_t* param,
												int Input,
												int Output,
												int Setpoint,
												float Kc, float TauI, float TauD);
												
void SetInputLimits(PID_Parameter_t* param,int INMin, int INMax);
void SetOutputLimits(PID_Parameter_t* param,int OUTMin, int OUTMax);
void SetTunings(PID_Parameter_t* param,const float Kc, const float TauI, const float TauD);
void Reset(PID_Parameter_t* param);
void SetMode(PID_Parameter_t* param, int Mode);
void SetSampleTime(PID_Parameter_t* param,int NewSampleTime);
void PID_Compute(PID_Parameter_t* param);
#endif
