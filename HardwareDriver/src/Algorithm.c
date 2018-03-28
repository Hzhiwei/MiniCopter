#include "Algorithm.h"


void PID_InitConfig(PID_TypeDef *PID, float P, float I, float D, float MaxI, float MaxPID)
{
	PID->P = P;
	PID->I = I;
	PID->D = D;
	PID->MaxI = MaxI;
	PID->MaxPID = MaxPID;
	PID->Iout = 0;
	PID->LastError = 0;
}


void PID_Calculate(PID_TypeDef *PID, float Target, float Real)
{
	PID->LastError = PID->Error;
	PID->LastPIDout = PID->PIDout;
	PID->Error = Target - Real;
	PID->Pout = PID->Error * PID->P;
	PID->Iout += PID->Error * PID->I;
	PID->Iout = PID->Iout > PID->MaxI ? PID->MaxI : PID->Iout;
	PID->Iout = PID->Iout < -PID->MaxI ? -PID->MaxI : PID->Iout;
	PID->Dout = (PID->Error - PID->LastError) * PID->D;
	PID->PIDout = PID->Pout + PID->Iout + PID->Dout;
	PID->PIDout = PID->PIDout > PID->MaxPID ? PID->MaxPID : PID->PIDout;
	PID->PIDout = PID->PIDout < -PID->MaxPID ? -PID->MaxPID : PID->PIDout;
}

