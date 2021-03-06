#include "task_Control.h"
#include "cmsis_os.h"
#include "mpu6500.h"
#include "Bluetooth.h"
#include "Motor.h"
#include "Algorithm.h"
#include "Flash.h"
#include "externParam.h"
#include <math.h>


#define RCROCKERRATE	7.5f

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
static SendProtocolDetail spd =
	{
		.flyMode = 0,
		.offsetWriteStatus = 0,
		.LRoffset = 1,
		.FBoffset = 2
	};
	
//飞行状态	0 停止	1 正常飞行	2 等待抛飞
static uint8_t flyStatus = 0;

static void Motor_Control(void);

void task_Control(const void *Parameters)
{
	TickType_t tick;
	uint8_t linking = 0;
	uint16_t LEDCounter = 0;
	float *EulerOffet;
	uint8_t offsetAdjustFlag = 0;
	
	PID_InitConfig(&PitchOUTPID, 10.5, 0, 0, 0, 100);
	PID_InitConfig(&PitchINPID, 0.25, 0, 1, 0, 25);
	PID_InitConfig(&RollOUTPID, 10.5, 0, 0, 0, 100);
	PID_InitConfig(&RollINPID, 0.25, 0, 2, 0, 25);
	PID_InitConfig(&YawOUTPID, 2.5, 0, 0, 0, 30);
	PID_InitConfig(&YawINPID, 2, 0, 0, 0, 35);
	
	uint16_t sendCounter = 0;
	
	EulerOffet = MPU6500_GetEulerOffsetPoint();
	Flash_Read_Mpu6500EulerOffet(EulerOffet);
	
	while(1)
	{
		tick = xTaskGetTickCount();
		
		//获取欧拉角
		MPU6500_eMPLEular(&CurrentEuler, &CurrentAcc, &CurrentGyro, tick);
		
		//分析无线数据
		if(!Bluetooth_ReceiveAnalyzeAndGetData(&rpd))
		{
			lostCounter = 0;
		}
		else if(lostCounter < LOSTSTOP)
		{
			++lostCounter;
		}
		
		//根据无线帧率判断连接状态
		if(lostCounter >= LOSTSTOP)
		{
			linking = 0;
		}
		else
		{
			linking = 1;
		}
		
		
		if(linking)
			//无线连接
		{
			//电机PID
			Motor_Control();
			
			//10个周期发送一次数据
			if(sendCounter <= 10)
			{
				sendCounter++;
			}
			else
			{
				sendCounter = 0;
				
				spd.FBoffset = (int8_t)(EulerOffet[0] * 10);
				spd.LRoffset = (int8_t)(EulerOffet[1] * 10);
				Bluetooth_Send(&spd);
			}
			
			//偏移调节写入
			switch(rpd.adjust)
			{
				case 1 :
				{
					if((EulerOffet[0] <= 10) && (offsetAdjustFlag == 0))
					{
						EulerOffet[0] += 0.1;
						offsetAdjustFlag = 1;
						spd.offsetWriteStatus = 1;
					}
					break;
				}
				case 2 :
				{
					if((EulerOffet[0] >= -10) && (offsetAdjustFlag == 0))
					{
						EulerOffet[0] -= 0.1;
						offsetAdjustFlag = 1;
						spd.offsetWriteStatus = 1;
					}
					break;
				}
				case 3 :
				{
					if((EulerOffet[1] >= -10) && (offsetAdjustFlag == 0))
					{
						EulerOffet[1] -= 0.1;
						offsetAdjustFlag = 1;
						spd.offsetWriteStatus = 1;
					}
					break;
				}
				case 4 :
				{
					if((EulerOffet[1] <= 10) && (offsetAdjustFlag == 0))
					{
						EulerOffet[1] += 0.1;
						offsetAdjustFlag = 1;
						spd.offsetWriteStatus = 1;
					}
					break;
				}
				case 5 :
				{
					Flash_Write_Mpu6500EulerOffet(EulerOffet);
					spd.offsetWriteStatus = 0;
					break;
				}
				default :
				{
					offsetAdjustFlag = 0;
					break;
				}
			}
			
		}
		else
			//无线断线
		{
			Motor_SetSpeed(0, 0);
			Motor_SetSpeed(1, 0);
			Motor_SetSpeed(2, 0);
			Motor_SetSpeed(3, 0);
			Motor_SetSpeed(4, 0);
			Motor_SetSpeed(5, 0);
		}
		
		//LED闪烁
		if(LEDCounter < 60)
		{
			++LEDCounter;
		}
		else
		{
			LEDCounter = 0;
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_8);
		}
		
		//看门狗喂狗
    HAL_IWDG_Refresh(&hiwdg);
		
		vTaskDelayUntil(&tick, 5);
	}
}


static void Motor_Control(void)
{
	static uint8_t motorSpeed[6];
	static int16_t tempSpeed[6];
	static float targetYaw = 0.0f;
	
	targetYaw -= rpd.SP / RCROCKERRATE;
	
	targetYaw = targetYaw < CurrentEuler.Yaw - YAWDIFFENCE ? CurrentEuler.Yaw - YAWDIFFENCE : targetYaw;
	targetYaw = targetYaw > CurrentEuler.Yaw + YAWDIFFENCE ? CurrentEuler.Yaw + YAWDIFFENCE : targetYaw;
	
	if((lostCounter >= LOSTSTOP)
		|| (rpd.locked)
		|| (rpd.power == 0))
	{
		Motor_SetSpeed(0, 0);
		Motor_SetSpeed(1, 0);
		Motor_SetSpeed(2, 0);
		Motor_SetSpeed(3, 0);
		Motor_SetSpeed(4, 0);
		Motor_SetSpeed(5, 0);
		
		return;
	}
	
	PID_Calculate(&PitchOUTPID, -rpd.FB / RCROCKERRATE, CurrentEuler.Pitch);
	PID_Calculate(&PitchINPID, PitchOUTPID.PIDout, CurrentGyro.y);
	PID_Calculate(&RollOUTPID, -rpd.LR / RCROCKERRATE, CurrentEuler.Roll);
	PID_Calculate(&RollINPID, RollOUTPID.PIDout, CurrentGyro.x);
	PID_Calculate(&YawOUTPID, targetYaw, CurrentEuler.Yaw);
	PID_Calculate(&YawINPID, YawOUTPID.PIDout, CurrentGyro.z);
	
	
#if FOURAXIS
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
	
	Motor_SetSpeed(0, motorSpeed[0]);
	Motor_SetSpeed(1, motorSpeed[1]);
	Motor_SetSpeed(2, motorSpeed[2]);
	Motor_SetSpeed(3, motorSpeed[3]);
#else
	float Hypotenuse = sqrt(PitchINPID.PIDout * PitchINPID.PIDout + RollINPID.PIDout * RollINPID.PIDout);
	float delta = acos(RollINPID.PIDout / Hypotenuse);
	if(PitchINPID.PIDout < 0)
	{
		delta = 6.28319f - delta;
	}
	
	tempSpeed[0] = (int16_t)rpd.power + cos(2.094395f - delta) * Hypotenuse + YawINPID.PIDout;
	tempSpeed[1] = (int16_t)rpd.power + cos(4.188790f - delta) * Hypotenuse + YawINPID.PIDout;
	tempSpeed[2] = (int16_t)rpd.power + cos(5.235988f - delta) * Hypotenuse - YawINPID.PIDout;
	tempSpeed[3] = (int16_t)rpd.power + cos(1.047198f - delta) * Hypotenuse - YawINPID.PIDout;
	tempSpeed[4] = (int16_t)rpd.power + cos(3.141593f - delta) * Hypotenuse - YawINPID.PIDout;
	tempSpeed[5] = (int16_t)rpd.power + cos(0.000000f - delta) * Hypotenuse + YawINPID.PIDout;
	
	motorSpeed[0] = tempSpeed[0] < 0 ? 0 : tempSpeed[0];
	motorSpeed[0] = motorSpeed[0] > 100 ? 100 : motorSpeed[0];
	motorSpeed[1] = tempSpeed[1] < 0 ? 0 : tempSpeed[1];
	motorSpeed[1] = motorSpeed[1] > 100 ? 100 : motorSpeed[1];
	motorSpeed[2] = tempSpeed[2] < 0 ? 0 : tempSpeed[2];
	motorSpeed[2] = motorSpeed[2] > 100 ? 100 : motorSpeed[2];
	motorSpeed[3] = tempSpeed[3] < 0 ? 0 : tempSpeed[3];
	motorSpeed[3] = motorSpeed[3] > 100 ? 100 : motorSpeed[3];
	motorSpeed[4] = tempSpeed[4] < 0 ? 0 : tempSpeed[4];
	motorSpeed[4] = motorSpeed[4] > 100 ? 100 : motorSpeed[4];
	motorSpeed[5] = tempSpeed[5] < 0 ? 0 : tempSpeed[5];
	motorSpeed[5] = motorSpeed[5] > 100 ? 100 : motorSpeed[5];
	
	Motor_SetSpeed(0, motorSpeed[0]);
	Motor_SetSpeed(1, motorSpeed[1]);
	Motor_SetSpeed(2, motorSpeed[2]);
	Motor_SetSpeed(3, motorSpeed[3]);
	Motor_SetSpeed(4, motorSpeed[4]);
	Motor_SetSpeed(5, motorSpeed[5]);
#endif	

}






