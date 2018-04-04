#include "task_Control.h"
#include "cmsis_os.h"
#include "mpu6500.h"
#include "LC12S.h"
#include "Algorithm.h"


static PID_TypeDef PitchOUTPID, PitchINPID, RollOUTPID, RollINPID, YawOUTPID, YawINPID;


void task_Control(const void *Parameters)
{
	TickType_t tick;
	Euler_Type CurrentEuler;
	Acce_Type CurrentAcc;
	Gyro_Type CurrentGyro;
	protocolDetail pd = 
	{
		.locked = 1,
		.headMode = 1,
		.power = 0,
		.LR = 0,
		.FB = 0,
		.SP = 0,
		.adjust = 0
	};
	
	PID_InitConfig(&PitchOUTPID, 5, 0, 0, 0, 50);
	PID_InitConfig(&PitchINPID, 2.0, 0, 8, 0, 50);
	PID_InitConfig(&RollOUTPID, 5, 0, 0, 0, 50);
	PID_InitConfig(&RollINPID, 2.0, 0, 8, 0, 50);
	PID_InitConfig(&YawOUTPID, 2.5, 0, 0, 0, 30);
	PID_InitConfig(&YawINPID, 2, 0, 0, 0, 25);
	
	
	while(1)
	{
		tick = xTaskGetTickCount();
		
		MPU6500_eMPLEular(&CurrentEuler, &CurrentAcc, &CurrentGyro, tick);
		LC12S_ReceiveAnalyzeAndGetData(&pd);
		
		vTaskDelayUntil(&tick, 5);
	}
}


