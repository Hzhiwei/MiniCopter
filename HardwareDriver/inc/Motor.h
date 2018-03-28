#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f3xx_hal.h"



void Motor_Init(void);
void Motor_SetSpeed(uint8_t index, uint8_t speed);


#endif
