#include "task_Control.h"
#include "cmsis_os.h"
#include "mpu6500.h"
#include "LC12S.h"
#include "Motor.h"
#include "Algorithm.h"


#define LOSTSTOP	20


static PID_TypeDef PitchOUTPID, PitchINPID, RollOUTPID, RollINPID, YawOUTPID, YawINPID;
static uint16_t lostCounter = 0;
static Euler_Type CurrentEuler;
static Acce_Type CurrentAcc;
static Gyro_Type CurrentGyro;
static ReceiveProtocolDetail rpd = 
	{
		.locked = 1,
		.headMode = 1,
		.power = 0,
		.LR = 0,
		.FB = 0,
		.SP = 0,
		.adjust = 0
	};

static void Motor_Control(void);


void task_Control(const void *Parameters)
{
	TickType_t tick;
	uint16_t LEDCounter = 0;
	SendProtocolDetail spd =
	{
		.flyStatus = 123
	};
	
	PID_InitConfig(&PitchOUTPID, 1, 0, 0, 0, 20);
	PID_InitConfig(&PitchINPID, 1.5, 0, 2, 0, 50);
	PID_InitConfig(&RollOUTPID, 1.5, 0, 0, 0, 20);
	PID_InitConfig(&RollINPID, 1, 0, 2, 0, 50);
	PID_InitConfig(&YawOUTPID, 2.5, 0, 0, 0, 30);
	PID_InitConfig(&YawINPID, 2, 0, 0, 0, 30);
	
	
	while(1)
	{
		tick = xTaskGetTickCount();
		
		MPU6500_eMPLEular(&CurrentEuler, &CurrentAcc, &CurrentGyro, tick);
		if(LC12S_ReceiveAnalyzeAndGetData(&rpd))
		{
			lostCounter = 0;
		}
		else if(lostCounter < LOSTSTOP)
		{
			++lostCounter;
		}
		Motor_Control();
		
		//LC12S_Send(&spd);
		
		if(LEDCounter < 60)
		{
			++LEDCounter;
		}
		else
		{
			LEDCounter = 0;
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_8);
		}
		
		vTaskDelayUntil(&tick, 5);
	}
}


static void Motor_Control(void)
{
	static uint8_t motorSpeed[4];
	static int16_t tempSpeed[4];
	
	if((lostCounter >= LOSTSTOP)
		|| (rpd.locked)
		|| (rpd.power == 0))
	{
		Motor_SetSpeed(0, 0);
		Motor_SetSpeed(1, 0);
		Motor_SetSpeed(2, 0);
		Motor_SetSpeed(3, 0);
		
		return;
	}
	
	PID_Calculate(&PitchOUTPID, -rpd.LR / 4.0f, CurrentEuler.Pitch);
	PID_Calculate(&PitchINPID, PitchOUTPID.PIDout, CurrentGyro.y);
	PID_Calculate(&RollOUTPID, -rpd.FB / 4.0f, CurrentEuler.Roll);
	PID_Calculate(&RollINPID, RollOUTPID.PIDout, CurrentGyro.x);
	PID_Calculate(&YawOUTPID, -rpd.SP / 4.0f, CurrentEuler.Yaw);
	PID_Calculate(&YawINPID, YawOUTPID.PIDout, CurrentGyro.z);
	
	tempSpeed[0] = (int16_t)rpd.power + PitchINPID.PIDout - RollINPID.PIDout + YawINPID.PIDout;
	tempSpeed[1] = (int16_t)rpd.power - PitchINPID.PIDout - RollINPID.PIDout - YawINPID.PIDout;
	tempSpeed[2] = (int16_t)rpd.power - PitchINPID.PIDout + RollINPID.PIDout + YawINPID.PIDout;
	tempSpeed[3] = (int16_t)rpd.power + PitchINPID.PIDout + RollINPID.PIDout - YawINPID.PIDout;
	
	motorSpeed[0] = tempSpeed[0] < 0 ? 0 : tempSpeed[0];
	motorSpeed[0] = motorSpeed[0] > 100 ? 100 : motorSpeed[0];
	motorSpeed[1] = tempSpeed[1] < 0 ? 0 : tempSpeed[1];
	motorSpeed[1] = motorSpeed[1] > 100 ? 100 : motorSpeed[1];
	motorSpeed[2] = tempSpeed[2] < 0 ? 0 : tempSpeed[2];
	motorSpeed[2] = motorSpeed[2] > 100 ? 100 : motorSpeed[2];
	motorSpeed[3] = tempSpeed[3] < 0 ? 0 : tempSpeed[3];
	motorSpeed[3] = motorSpeed[3] > 100 ? 100 : motorSpeed[3];
	
//	motorSpeed[0] = 0;
//	motorSpeed[1] = 0;
//	motorSpeed[2] = 0;
//	motorSpeed[3] = 0;
	
	Motor_SetSpeed(0, motorSpeed[0]);
	Motor_SetSpeed(1, motorSpeed[1]);
	Motor_SetSpeed(2, motorSpeed[2]);
	Motor_SetSpeed(3, motorSpeed[3]);
}






