#include "task_Control.h"
#include "cmsis_os.h"
#include "mpu6500.h"
#include "LC12S.h"


Euler_Type CurrentEuler;


void task_Control(const void *Parameters)
{
	TickType_t tick;
	
	while(1)
	{
		tick = xTaskGetTickCount();
		
		//MPU6500_GetEular(&CurrentEuler, tick);
		LC12S_ReceiveAnalyze();
		
		vTaskDelayUntil(&tick, 5);
	}
}


