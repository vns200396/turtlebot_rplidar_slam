#ifndef __MY_PID_H
#define	__MY_PID_H

#include "mylib.h"

typedef struct
{
	float KP;
	
	float KI;
	
	float KD;
	
	float P_Parameter;
	
	float I_Parameter;
	
	float D_Parameter;
	
	bool justcalc;
	
	int error;
	int previousError;
	
	unsigned long next_compute_time;
	
	int PIDvalue;
	
}MyPID_TypeDef;


void calculatePID(MyPID_TypeDef* mypid);
#endif
