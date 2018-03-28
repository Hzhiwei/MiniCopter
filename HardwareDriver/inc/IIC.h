#ifndef __IIC_H
#define __IIC_H


#include "stm32f3xx_hal.h"


void IIC_InitConfig(void);
uint8_t IIC_SingleSend(uint8_t addr, uint8_t reg, uint8_t data);
uint8_t IIC_SingleRead(uint8_t addr, uint8_t reg, uint8_t *data);
uint8_t IIC_SendBuffer(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data);
uint8_t IIC_ReadBuffer(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data);


#endif
