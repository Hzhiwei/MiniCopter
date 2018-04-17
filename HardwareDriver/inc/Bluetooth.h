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

//ͨ��Э��ĸ�����־λ�����ݷֿ���ϸ��
typedef struct 
{
	//ģʽ 0 ֹͣ	1����	2�ȴ��׷�
	uint8_t mode;
	//����ƫ�Ƶ���
	int8_t LRoffset;
	//ǰ��ƫ�Ƶ���	
	int8_t FBoffset;			
}SendProtocolDetail;



uint8_t Bluetooth_Init(uint16_t Net);
void Bluetooth_Start(void);
uint8_t Bluetooth_ReceiveAnalyze(void);
uint8_t Bluetooth_ReceiveAnalyzeAndGetData(ReceiveProtocolDetail *pd);
void Bluetooth_GetData(ReceiveProtocolDetail *pd);

void Bluetooth_Send(const SendProtocolDetail *pd);


#endif
