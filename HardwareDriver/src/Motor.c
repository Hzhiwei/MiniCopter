#include "Motor.h"
#include "externParam.h"


void Motor_Init(void)
{
	htim2.Instance->CCR1 = 0;
	htim2.Instance->CCR2 = 0;
	htim2.Instance->CCR3 = 0;
	htim2.Instance->CCR4 = 0;
	
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
	
	HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_4);
}


void Motor_SetSpeed(uint8_t index, uint8_t speed)
{
	switch(index)
	{
		case 1 : htim2.Instance->CCR1 = speed;	break;
		case 2 : htim2.Instance->CCR2 = speed;	break;
		case 3 : htim2.Instance->CCR3 = speed;	break;
		case 4 : htim2.Instance->CCR4 = speed;	break;
		default : break;
	}
}
