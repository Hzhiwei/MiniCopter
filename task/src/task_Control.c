#include "task_Control.h"
#include "cmsis_os.h"
#include "mpu6500.h"
#include "LC12S.h"
#include "Motor.h"
#include "Algorithm.h"
#include "Flash.h"
#include "externParam.h"


#define RCROCKERRATE	4.0f

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
	float EulerOffet[3];
	SendProtocolDetail spd =
	{
		.flyStatus = 123
	};
	
	PID_InitConfig(&PitchOUTPID, 10.5, 0, 10, 0, 100);
	PID_InitConfig(&PitchINPID, 0.25, 0, 1, 0, 25);
	PID_InitConfig(&RollOUTPID, 10.5, 0, 10, 0, 100);
	PID_InitConfig(&RollINPID, 0.25, 0, 2, 0, 25);
	PID_InitConfig(&YawOUTPID, 5, 0, 0, 0, 30);
	PID_InitConfig(&YawINPID, 2, 0, 0, 0, 50);
	
	//Flash_Read_Mpu6500EulerOffet(EulerOffet);
	MPU6500_SetEulerOffset(EulerOffet);
	
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
		
		if(LEDCounter < 60)
		{
			++LEDCounter;
		}
		else
		{
			LEDCounter = 0;
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_8);
		}
		
        HAL_IWDG_Refresh(&hiwdg);
		
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
	
	PID_Calculate(&PitchOUTPID, -rpd.FB / RCROCKERRATE, CurrentEuler.Pitch);
	PID_Calculate(&PitchINPID, PitchOUTPID.PIDout, CurrentGyro.y);
	PID_Calculate(&RollOUTPID, -rpd.LR / RCROCKERRATE, CurrentEuler.Roll);
	PID_Calculate(&RollINPID, RollOUTPID.PIDout, CurrentGyro.x);
	PID_Calculate(&YawOUTPID, -rpd.SP / RCROCKERRATE, CurrentEuler.Yaw);
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






