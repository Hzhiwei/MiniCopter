#ifndef __ALGORITHM_H
#define __ALGORITHM_H


#include "stm32f3xx_hal.h"


typedef struct
{
	float Error;
	float P;
	float I;
	float D;
	float Pout;
	float Iout;
	float Dout;
	float PIDout;
	float MaxI;
	float MaxPID;
	float LastError;
	float LastPIDout;
}PID_TypeDef;


void PID_InitConfig(PID_TypeDef *PID, float P, float I, float D, float MaxI, float MaxPID);
void PID_Calculate(PID_TypeDef *PID, float Target, float Real);


#endif
