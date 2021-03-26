#include "MyPID.h"





void calculatePID(MyPID_TypeDef* mypid)
{
	if(mypid->justcalc) return;
	
	mypid->P_Parameter = mypid->error;
	mypid->I_Parameter = mypid->I_Parameter + mypid->error;
	mypid->D_Parameter = mypid->error - mypid->previousError;
	mypid->PIDvalue = (mypid->KP*mypid->P_Parameter) + (mypid->KI*mypid->I_Parameter) + (mypid->KD*mypid->D_Parameter);
	mypid->previousError = mypid->error;
	
	mypid->justcalc = true;
}

