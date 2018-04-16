#ifndef __BLUETOOTH_H
#define __BLUETOOTH_H


#include "stm32f3xx_hal.h"

//ѹ��ǰ������ϸ��
typedef struct 
{
	//1 ����	2 ����
	uint8_t locked;
	//1 ��ͷģʽ	2 ����ģʽ
	uint8_t headMode;
	//�������
	uint8_t power;
	//������б	������
	int8_t  LR;
	//ǰ����б	��ǰ��
	int8_t  FB;
	//����	������
	int8_t  SP;
	//������бƫ�Ƶ���
	/*
	0��������
	1����ƫ0.1��
	2����ƫ0.1��
	3��ǰƫ0.1��
	4����ƫ0.1��
	5��д��Flash
	*/
	int8_t adjust;			
}ReceiveProtocolDetail;

//����Э��
typedef struct 
{
	//����״̬	0 ֹͣ	1 ����
	uint8_t flyStatus;			
}SendProtocolDetail;


uint8_t LC12S_Init(uint16_t Net);
void LC12S_Send(const SendProtocolDetail *pd);
uint8_t LC12S_ReceiveAnalyze(void);
uint8_t LC12S_ReceiveAnalyzeAndGetData(ReceiveProtocolDetail *pd);
void LC12S_GetData(ReceiveProtocolDetail *pd);


#endif
