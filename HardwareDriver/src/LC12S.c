
#include <string.h>
#include "LC12S.h"
#include "externParam.h"
#include "cmsis_os.h"


#define RBLENGTH	64


//��Э��ı�־λ��ϣ�������һ��ѹ�������ڴ����������
typedef union 
{
	struct combin
	{
		uint8_t mode;
		uint8_t power;
		int8_t LR;
		int8_t FB;
		uint8_t adjust;
	}fusion;
	uint8_t array[sizeof(struct combin) + 1];	//���һ���ֽ�ΪУ���
}protocolTransmit;


static protocolDetail currentStatus;


#define	LS12S_SET	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET)
#define	LS12S_RESET	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET)


static uint8_t SendBuffer[32];
static uint8_t ReceiveBuffer[RBLENGTH];


static uint8_t LS12SParam[18] = {0xAA,0x5A,0x00,0x05,0x00,0x00,0x00,0x00,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x12,0x00,0x00};


uint8_t LC12S_Init(uint16_t Net)
{
	LS12S_SET;
	osDelay(30);
	LS12S_RESET;
	osDelay(20);
	
	LS12SParam[4] = Net >> 8;
	LS12SParam[5] = Net;
	LS12SParam[17] = 0;
	for(uint8_t i = 0; i < 17; ++i)
	{
		LS12SParam[17] += LS12SParam[i];
	}
	if(HAL_UART_Transmit(&huart1, LS12SParam, 18, 100) != HAL_OK)
	{
		return 0;
	}
	if(HAL_UART_Receive(&huart1, ReceiveBuffer, 18, 100) != HAL_OK)
	{
		return 0;
	}
	
	uint8_t sum = 0;
	for(uint8_t i = 0; i < 17; ++i)
	{
		sum += ReceiveBuffer[i];
	}
	if(sum != ReceiveBuffer[17])
	{
		return 0;
	}
	
	LS12S_SET;
	
	return 1;
}


void LC12S_Start(void)
{
	HAL_UART_Receive_IT(&huart1, ReceiveBuffer, RBLENGTH);
	LS12S_SET;
}


uint8_t LC12S_ReceiveAnalyze(void)
{
	#define DATAREPOLENGTH	128	//���ݴ����λ��泤��
	#define NEXT(x)	(((x)+1)%DATAREPOLENGTH)
	#define PREVIOUS(x)	(((x)+DATAREPOLENGTH-1)%DATAREPOLENGTH)
	
	static uint8_t dataRepo[DATAREPOLENGTH];	//���ݴ����λ���
	static uint8_t dataStartPoint = 0;	//��һ�����������ݴ����λ��濪ʼָ��
	static uint8_t dataEndPoint = 0; //��һ�����������ݴ����λ������ָ��
	
	uint8_t dataArranged[sizeof(protocolTransmit)];	//�������õ�����
	
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
	uint8_t Flag = 0;	//����״̬��0������֡β��1����֡β�����֡ͷ��2��������֡
	uint8_t frameStart;	//��֡ͷ������ݵĵ�һ�����ݵ�λ��
	uint8_t frameEnd;	//��֡β������ݵ����һ�����ݵ���һ��λ��
	uint8_t dataLength = dataStartPoint <= dataEndPoint ? dataEndPoint - dataStartPoint : DATAREPOLENGTH - dataStartPoint - 1 + dataEndPoint;
	if(dataLength >= sizeof(protocolTransmit) + 4)
	{
		for(int i = PREVIOUS(PREVIOUS(dataEndPoint)); i != dataStartPoint; i = PREVIOUS(i))
		{
			switch(Flag)
			{
				case 0 :
				{
					if((dataRepo[i] == 0xFF) && (dataRepo[i + 1] == 0x02))
					{
						frameEnd = i;
						Flag = 1;
					}
					break;
				}
				case 1 :
				{
					if((dataRepo[i] == 0xFF) && (dataRepo[i + 1] == 0x01))
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
		return 0;
	}
	
	ANALYZE:
	if(Flag == 0)
	{
		return 0;
	}
	dataStartPoint = NEXT(frameEnd);
	if(Flag == 1)
	{
		return 0;
	}
	
	//��������֡
	uint8_t dataCounter = 0;
	for(uint8_t i = frameStart; i != frameEnd; ++dataCounter, i = NEXT(i))
	{
		if(dataRepo[i] == 0xFF)
		{
			i = NEXT(i);
			switch(i)
			{
				case 0x00 : dataArranged[dataCounter] = 0xFF; break;
				default : dataArranged[dataCounter] = dataRepo[i]; break;
			}
		}
		else
		{
			dataArranged[dataCounter] = dataRepo[i];
		}
	}
	
	//��鳤��
	if(sizeof(protocolTransmit) != dataCounter)
	{
		return 0;
	}
	
	//У��
	uint8_t sum = 0;
	for(uint8_t i = 0; i < sizeof(protocolTransmit) - 1; ++i)
	{
		sum += dataArranged[i];
	}
	if(sum != dataArranged[sizeof(protocolTransmit) - 1])
	{
		return 0;
	}
	
	//���ݷ���
	protocolTransmit temp;
	for(uint8_t i = 0; i < sizeof(protocolTransmit); ++i)
	{
		temp.array[i] = dataArranged[i];
	}
	currentStatus.locked = temp.fusion.mode & 0x80;
	currentStatus.headMode = temp.fusion.mode & 0x40;
	currentStatus.power = temp.fusion.power;
	currentStatus.LR = temp.fusion.LR;
	currentStatus.FB = temp.fusion.FB;
	currentStatus.adjust = temp.fusion.adjust;
	
	return 1;
	
	
	#undef DATAREPOLENGTH
	#undef NEXT
	#undef PREVIOUS
}







