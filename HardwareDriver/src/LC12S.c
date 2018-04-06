
#include <string.h>
#include "LC12S.h"
#include "externParam.h"
#include "cmsis_os.h"


#define RBLENGTH	64


//将协议的标志位结合，与数据一起压缩后并用于传输的联合体
/*
接受通信协议：

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
	uint8_t array[sizeof(struct combin) + 1];	//最后一个字节为校验和
}ReceiveProtocolTransmit;


//将协议的标志位结合，与数据一起压缩后并用于传输的结构体
/*
byte0 状体：
		bit7 : 0 停止,1 飞行
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


#define	LS12S_SET	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET)
#define	LS12S_RESET	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET)


static uint8_t SendBuffer[32];
static uint8_t ReceiveBuffer[RBLENGTH];


static uint8_t LS12SParam[18] = {0xAA,0x5A,0x00,0x05,0x00,0x00,0x00,0x00,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x12,0x00,0x00};


static void LC12S_PT2PD(const SendProtocolDetail *pd, SendProtocolTransmit *pt);	


uint8_t LC12S_Init(uint16_t Net)
{
	LS12S_SET;
	osDelay(30);
	LS12S_RESET;
	osDelay(100);
	
	LS12SParam[4] = Net >> 8;
	LS12SParam[5] = Net;
	LS12SParam[17] = 0;
	for(uint8_t i = 0; i < 17; ++i)
	{
		LS12SParam[17] += LS12SParam[i];
	}
	huart1.Instance->ISR = ~((uint16_t)0x0020);
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
	#define DATAREPOLENGTH	128	//数据处理环形缓存长度
	#define NEXT(x)	(((x)+1)%DATAREPOLENGTH)
	#define PREVIOUS(x)	(((x)+DATAREPOLENGTH-1)%DATAREPOLENGTH)
	
	static uint8_t dataRepo[DATAREPOLENGTH];	//数据处理环形缓存
	static uint8_t dataStartPoint = 0;	//下一个数字是数据处理环形缓存开始指针
	static uint8_t dataEndPoint = 0; //上一个数字是数据处理环形缓存结束指针
	
	uint8_t dataArranged[sizeof(ReceiveProtocolTransmit)];	//存放整理好的数据
	
	//复制数据至数据处理环形缓存
	for(uint8_t i = 0; i < RBLENGTH - huart1.RxXferCount; ++i)
	{
		dataRepo[dataEndPoint] = ReceiveBuffer[i];
		dataEndPoint = NEXT(dataEndPoint);
	}
	
	//重新开始接受
	HAL_UART_AbortTransmit_IT(&huart1);
	HAL_UART_Receive_IT(&huart1, ReceiveBuffer, RBLENGTH);
	
	//查找帧头或帧尾
	uint8_t Flag = 0;	//分析状态，0：查找帧尾，1查找帧尾后查找帧头，2发现完整帧
	uint8_t frameStart;	//除帧头后的数据的第一个数据的位置
	uint8_t frameEnd;	//除帧尾后的数据的最后一个数据的下一个位置
	uint8_t dataLength;
	Flag = 0;
	dataLength = dataStartPoint <= dataEndPoint ? dataEndPoint - dataStartPoint : DATAREPOLENGTH - dataStartPoint - 1 + dataEndPoint;
	
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
	
	//分析完整帧
	uint8_t dataCounter = 0;
	for(uint8_t i = frameStart; i != frameEnd; ++dataCounter, i = NEXT(i))
	{
		if(dataRepo[i] == 0xFF)
		{
			i = NEXT(i);
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
	
	//检查长度
	if(sizeof(ReceiveProtocolTransmit) != dataCounter)
	{
		return 0;
	}
	
	//校验
	uint8_t sum = 0;
	for(uint8_t i = 0; i < sizeof(ReceiveProtocolTransmit) - 1; ++i)
	{
		sum += dataArranged[i];
	}
	if(sum != dataArranged[sizeof(ReceiveProtocolTransmit) - 1])
	{
		return 0;
	}
	
	//数据分配
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
	
	return 1;
	
	
	#undef DATAREPOLENGTH
	#undef NEXT
	#undef PREVIOUS
}


uint8_t LC12S_ReceiveAnalyzeAndGetData(ReceiveProtocolDetail *pd)
{
	if(LC12S_ReceiveAnalyze())
	{
		*pd = currentStatus;
		return 1;
	}
	else
	{
		return 0;
	}
}


void LC12S_GetData(ReceiveProtocolDetail *pd)
{
	*pd = currentStatus;
}


void LC12S_Send(const SendProtocolDetail *pd)
{
	SendProtocolTransmit pt;
	LC12S_PT2PD(pd, &pt);
	
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


//将SendProtocolDetail转化为SendProtocolTransmit
static void LC12S_PT2PD(const SendProtocolDetail *pd, SendProtocolTransmit *pt)
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





