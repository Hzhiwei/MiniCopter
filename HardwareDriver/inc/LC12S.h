#ifndef __LS12S_H
#define __LS12S_H


#include "stm32f3xx_hal.h"


//通信协议的各个标志位，数据分开的细节
/*
通信协议：

0xFF为转义标志，作用相当于字符串中的'\'
转义字符：
	0xFF 0x00		0xFF
	0xFF 0x01		帧头
	0xFF 0x02		帧尾

byte0 标志位：
		bit7 : 0 lock,1 unlock
		bit6 : 0 正常, 1 无头模式
		bit5 : 
		bit4 : 
		bit3 : 
		bit2 : 
		bit1 : 
		bit0 : 

byte1 油门：	0 ~ 100
byte2 左右偏转： -100 ~ 100，实际角度为数据值/4，左负右正
byte3 前后偏转： -100 ~ 100，实际角度为数据值/4，后负前正
byte4 自旋：     -100 ~ 100，左负右正
byte5 偏移调节： 
		bit7 : 0 无作用，1 左右偏移
		bit6 : 0 左偏0.1度，1 右偏0.1度
		bit5 : 0 无作用，1 前后偏移
		bit4 : 0 前偏0.1度，1 后偏0.1度
		bit3 : 0 无作用，1 左右偏移归零
		bit2 : 0 无作用，1 前后偏移归零
		bit1 : 0 无作用，1 偏移写入flash
		bit0 : 
byte6 校验和
*/
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
}protocolDetail;


uint8_t LC12S_Init(uint16_t Net);
void LC12S_Start(void);
uint8_t LC12S_ReceiveAnalyze(void);
uint8_t LC12S_ReceiveAnalyzeAndGetData(protocolDetail *pd);
void LC12S_GetData(protocolDetail *pd);


#endif
