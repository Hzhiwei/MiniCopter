
#include <string.h>
#include "Bluetooth.h"
#include "externParam.h"
#include "cmsis_os.h"


#define RBLENGTH	64


//��Э��ı�־λ��ϣ�������һ��ѹ�������ڴ����������
/*
����ͨ��Э�飺

0xFFΪת���־�������൱���ַ����е�'\'
ת���ַ���
	0xFF 0x00		0xFF
	0xFF 0x01		֡ͷ
	0xFF 0x02		֡β

byte0 ��־λ��
		bit7 : 0 lock,1 unlock
		bit6 : 0 ����, 1 ��ͷģʽ
		bit5 : 
		bit4 : 
		bit3 : 
		bit2 : 
		bit1 : 
		bit0 : 

byte1 ���ţ�	0 ~ 100
byte2 ����ƫת�� -100 ~ 100��ʵ�ʽǶ�Ϊ����ֵ/4��������
byte3 ǰ��ƫת�� -100 ~ 100��ʵ�ʽǶ�Ϊ����ֵ/4����ǰ��
byte4 ������     -100 ~ 100��������
byte5 ƫ�Ƶ��ڣ� 
		bit7 : 0 �����ã�1 ����ƫ��
		bit6 : 0 ��ƫ0.1�ȣ�1 ��ƫ0.1��
		bit5 : 0 �����ã�1 ǰ��ƫ��
		bit4 : 0 ǰƫ0.1�ȣ�1 ��ƫ0.1��
		bit3 : 0 �����ã�1 ����ƫ�ƹ���
		bit2 : 0 �����ã�1 ǰ��ƫ�ƹ���
		bit1 : 0 �����ã�1 ƫ��д��flash
		bit0 : 
byte6 У���
*/
typedef union 
{
	struct combin
	{
		uint8_t mode;
		uint8_t power;
		int8_t LR;
		int8_t FB;
		int8_t SP;
		uint8_t adjust;
	}fusion;
	uint8_t array[sizeof(struct combin)];	//���һ���ֽ�ΪУ���
}ReceiveProtocolTransmit;


//��Э��ı�־λ��ϣ�������һ��ѹ�������ڴ���Ľṹ��
/*
byte0 ״�壺
		bit7 : 0 ֹͣ,1 ����
		bit6 : 
		bit5 : 
		bit4 : 
		bit3 : 
		bit2 : 
		bit1 : 
		bit0 : 
*/
typedef struct 
{
	uint8_t status;
	uint8_t sum;
}SendProtocolTransmit;


static ReceiveProtocolDetail currentStatus;


static uint8_t SendBuffer[32];
static uint8_t ReceiveBuffer[RBLENGTH];


static void Bluetooth_PT2PD(const SendProtocolDetail *pd, SendProtocolTransmit *pt);	


uint8_t Bluetooth_Init(uint16_t Net)
{
	return 0;
}


void Bluetooth_Start(void)
{
	HAL_UART_Receive_IT(&huart1, ReceiveBuffer, RBLENGTH);
}


uint8_t Bluetooth_ReceiveAnalyze(void)
{
	#define DATAREPOLENGTH	128	//���ݴ����λ��泤��
	#define NEXT(x)	(((x)+1)%DATAREPOLENGTH)
	#define PREVIOUS(x)	(((x)+DATAREPOLENGTH-1)%DATAREPOLENGTH)
	
	static uint8_t dataRepo[DATAREPOLENGTH];	//���ݴ����λ���
	static uint8_t dataStartPoint = 0;	//��һ�����������ݴ����λ��濪ʼָ��
	static uint8_t dataEndPoint = 0; //��һ�����������ݴ����λ������ָ��
	
	uint8_t dataArranged[sizeof(ReceiveProtocolTransmit)];	//�������õ�����
	
	//�������������ݴ����λ���
	for(uint8_t i = 0; i < RBLENGTH - huart1.RxXferCount; ++i)
	{
		dataRepo[dataEndPoint] = ReceiveBuffer[i];
		dataEndPoint = NEXT(dataEndPoint);
	}
	
	//���¿�ʼ����
	HAL_UART_AbortTransmit_IT(&huart1);
	HAL_UART_Receive_IT(&huart1, ReceiveBuffer, RBLENGTH);
	
	//����֡ͷ��֡β
	static uint8_t Flag = 0;	//����״̬��0������֡β��1����֡β�����֡ͷ��2��������֡
	static uint8_t frameStart;	//��֡ͷ������ݵĵ�һ�����ݵ�λ��
	static uint8_t frameEnd;	//��֡β������ݵ����һ�����ݵ���һ��λ��
	static uint8_t dataLength;
	dataLength = dataStartPoint <= dataEndPoint ? dataEndPoint - dataStartPoint : DATAREPOLENGTH - dataStartPoint - 1 + dataEndPoint;
	
	Flag = 0;
	if(dataLength >= sizeof(ReceiveProtocolTransmit) + 4)
	{
		for(int i = PREVIOUS(PREVIOUS(dataEndPoint)); i != dataStartPoint; i = PREVIOUS(i))
		{
			switch(Flag)
			{
				case 0 :
				{
					if((dataRepo[i] == 0xFF) && (dataRepo[NEXT(i)] == 0x02))
					{
						frameEnd = i;
						Flag = 1;
					}
					break;
				}
				case 1 :
				{
					if((dataRepo[i] == 0xFF) && (dataRepo[NEXT(i)] == 0x01))
					{
						frameStart = NEXT(NEXT(i));
						Flag = 2;
						goto ANALYZE;
					}
					break;
				}
			}
		}
	}
	else
	{
		return 1;
	}
	
	ANALYZE:
	if(Flag == 0)
	{
		return 2;
	}
	dataStartPoint = NEXT(frameEnd);
	if(Flag == 1)
	{
		return 3;
	}
	
	frameEnd = PREVIOUS(frameEnd);
	
	//У��
	uint8_t sum = 0;
	for(uint8_t i = frameStart; i != frameEnd; i = NEXT(i))
	{
		sum += dataRepo[i];
	}
	if(sum != dataRepo[frameEnd])
	{
		return 5;
	}
	
	//��������֡
	uint8_t dataCounter = 0;
	for(uint8_t i = frameStart; i != frameEnd; ++dataCounter, i = NEXT(i))
	{
		if(dataRepo[i] == 0xFF)
		{
			i = NEXT(i);
			if(dataCounter >= sizeof(ReceiveProtocolTransmit))
			{
				return 6;
			}
			switch(dataRepo[i])
			{
				case 0x00 : dataArranged[dataCounter] = 0xFF; break;
				default : break;
			}
		}
		else
		{
			dataArranged[dataCounter] = dataRepo[i];
		}
	}
	
	//��鳤��
	if(sizeof(ReceiveProtocolTransmit) != dataCounter)
	{
		return 7;
	}
	
	//���ݷ���
	ReceiveProtocolTransmit temp;
	for(uint8_t i = 0; i < sizeof(ReceiveProtocolTransmit); ++i)
	{
		temp.array[i] = dataArranged[i];
	}
	currentStatus.locked = temp.fusion.mode & 0x80;
	currentStatus.headMode = temp.fusion.mode & 0x40;
	currentStatus.power = temp.fusion.power;
	currentStatus.LR = temp.fusion.LR;
	currentStatus.FB = temp.fusion.FB;
	currentStatus.SP = temp.fusion.SP;
	currentStatus.adjust = temp.fusion.adjust;
	
	return 0;

	
	#undef DATAREPOLENGTH
	#undef NEXT
	#undef PREVIOUS
}


uint8_t Bluetooth_ReceiveAnalyzeAndGetData(ReceiveProtocolDetail *pd)
{
	if(Bluetooth_ReceiveAnalyze())
	{
		*pd = currentStatus;
		return 0;
	}
	else
	{
		*pd = currentStatus;
		return 1;
	}
}


void Bluetooth_GetData(ReceiveProtocolDetail *pd)
{
	*pd = currentStatus;
}


void Bluetooth_Send(const SendProtocolDetail *pd)
{
	SendProtocolTransmit pt;
	Bluetooth_PT2PD(pd, &pt);
	
	uint8_t lendgth = 0;
	SendBuffer[lendgth++] = 0xFF;
	SendBuffer[lendgth++] = 0x01;
	for(uint8_t i = 0; i < sizeof(SendProtocolTransmit); ++i)
	{
		if(*((uint8_t *)(&pt) + i) == 0xFF)
		{
			SendBuffer[lendgth++] = 0xFF;
			SendBuffer[lendgth++] = 0x00;
		}
		else
		{
			SendBuffer[lendgth++] = *((uint8_t *)(&pt) + i);
		}
	}
	SendBuffer[lendgth++] = 0xFF;
	SendBuffer[lendgth++] = 0x02;
	HAL_UART_Transmit_IT(&huart1, (uint8_t *)(&SendBuffer), lendgth);
}


//��SendProtocolDetailת��ΪSendProtocolTransmit
static void Bluetooth_PT2PD(const SendProtocolDetail *pd, SendProtocolTransmit *pt)
{
	memset(pt, 0, sizeof(SendProtocolTransmit));
	
	if(pd->flyStatus)
	{
		pt->status |= 0x80;
	}
	uint8_t sum = 0;
	for(uint8_t i = 0; i < sizeof(SendProtocolTransmit) - 1; ++i)
	{
		sum += *((uint8_t *)pt + i);
	}
	pt->sum = sum;
}





