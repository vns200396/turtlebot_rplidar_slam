#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "stm32f10x.h"
#include "mylib.h"


#define AUTOMATIC	1
#define MANUAL	0
#define P_ON_M 0
#define P_ON_E 1

typedef struct _PID_TypeDefine
{
		float dispKp;				// * we'll hold on to the tuning parameters in user-entered 
		double dispKi;				//   format for display purposes
		double dispKd;				//

		double Kp;                  // * (P)roportional Tuning Parameter
		double Ki;                  // * (I)ntegral Tuning Parameter
		double Kd;                  // * (D)erivative Tuning Parameter

		int pOn;

		int Input;              
		int Output;              
		int Setpoint;         
								
		unsigned long  lastTime;
		uint32_t outputSum, lastInput;

		unsigned long  SampleTime;
		double outMin, outMax;
		bool inAuto, pOnE;
}PID_TypDef;

void PID_config(PID_TypDef* pid_t, int input, int output, int setpoint, double kp, double ki, double kd);        // * constructor.  links the PID to the Input, Output, and Setpoint.  Initial tuning parameters are also set here
void PID_SetMode(PID_TypDef* pid_t,int Mode);               // * sets PID to either Manual (0) or Auto (non-0)
void PID_Reset(PID_TypDef* pid_t);
bool PID_Compute(PID_TypDef* pid_t);                       // * performs the PID calculation.  it should be
																				//   called every time loop() cycles. ON/OFF and
																				//   calculation frequency can be set using SetMode
																				//   SetSampleTime respectively

void PID_SetOutputLimits(PID_TypDef* pid_t, int Min, int Max); // * clamps the output to a specific range. 0-MAX PWM by default, but
																														//   it's likely the user will want to change this depending on
																														//   the application
//available but not commonly used functions ********************************************************
void PID_SetTunings(PID_TypDef* pid_t, double kp, double ki, double kd);       // * While most users will set the tunings once in the         	    //   constructor, this function gives the user the option
																				//   of changing tunings during runtime for Adaptive control

//void SetTunings(double, double, double, int);      // * overload for specifying proportional mode
										     
void PID_SetSampleTime(PID_TypDef* pid_t, int NewSampleTime);              // * sets the frequency, in seconds, with which 
																				//   the PID calculation is performed.
void PID_Initialize(PID_TypDef* pid_t);

#endif
