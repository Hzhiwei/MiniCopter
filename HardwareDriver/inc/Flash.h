#ifndef __FLASH_H
#define __FLASH_H

#include "stm32f3xx_hal.h"


void Flash_Read_Mpu6500EulerOffet(float Offset[3]);
void Flash_Write_Mpu6500EulerOffet(float Offset[3]);


#endif
