#ifndef __BLUETOOTH_H
#define __BLUETOOTH_H


#include "stm32f3xx_hal.h"

//压缩前的数据细节
typedef struct 
{
	//1 加锁	2 解锁
	uint8_t locked;
	//1 无头模式	2 正常模式
	uint8_t headMode;
	//电机油门
	uint8_t power;
	//左右倾斜	左负右正
	int8_t  LR;
	//前后倾斜	后负前正
	int8_t  FB;
	//自旋	左负右正
	int8_t  SP;
	//左右倾斜偏移调节
	/*
	0：无作用
	1：左偏0.1度
	2：右偏0.1度
	3：前偏0.1度
	4：后偏0.1度
	5：写入Flash
	*/
	int8_t adjust;			
}ReceiveProtocolDetail;

//发送协议
typedef struct 
{
	//飞行状态	0 停止	1 飞行
	uint8_t flyStatus;			
}SendProtocolDetail;


uint8_t LC12S_Init(uint16_t Net);
void LC12S_Send(const SendProtocolDetail *pd);
uint8_t LC12S_ReceiveAnalyze(void);
uint8_t LC12S_ReceiveAnalyzeAndGetData(ReceiveProtocolDetail *pd);
void LC12S_GetData(ReceiveProtocolDetail *pd);


#endif
