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

//通信协议的各个标志位，数据分开的细节
typedef struct 
{
	//模式 0 停止	1飞行	2等待抛飞
	uint8_t mode;
	//左右偏移调节
	int8_t LRoffset;
	//前后偏移调节	
	int8_t FBoffset;			
}SendProtocolDetail;



uint8_t Bluetooth_Init(uint16_t Net);
void Bluetooth_Start(void);
uint8_t Bluetooth_ReceiveAnalyze(void);
uint8_t Bluetooth_ReceiveAnalyzeAndGetData(ReceiveProtocolDetail *pd);
void Bluetooth_GetData(ReceiveProtocolDetail *pd);

void Bluetooth_Send(const SendProtocolDetail *pd);


#endif
