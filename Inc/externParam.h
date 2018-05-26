#ifndef __EXTERNPARAM_H
#define __EXTERNPARAM_H

#include "stm32f3xx_hal.h"

#define FOURAXIS		0

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart1;
extern PCD_HandleTypeDef hpcd_USB_FS;
extern IWDG_HandleTypeDef hiwdg;


#endif
