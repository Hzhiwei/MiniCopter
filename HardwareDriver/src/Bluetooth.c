
#include <string.h>
#include "Bluetooth.h"
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
typedef struct
{
	uint8_t mode;
	uint8_t power;
	int8_t LR;
	int8_t FB;
	int8_t SP;
	uint8_t adjust;
}ReceiveProtocolTransmit;


//将协议的标志位结合，与数据一起压缩后并用于传输的结构体
typedef struct 
{
	//模式 0 停止	1飞行	2等待抛飞
	uint8_t mode;
	//左右偏移调节
	int8_t LRoffset;
	//前后偏移调节	
	int8_t FBoffset;	
}SendProtocolTransmit;


static ReceiveProtocolDetail currentStatus;


static uint8_t SendBuffer[2 * sizeof(SendProtocolTransmit) + 4];
static uint8_t ReceiveBuffer[RBLENGTH];



uint8_t Bluetooth_Init(uint16_t Net)
{
	return 0;
}


/********************************	接收	*************************************/


void Bluetooth_Start(void)
{
	HAL_UART_Receive_IT(&huart1, ReceiveBuffer, RBLENGTH);
}


uint8_t Bluetooth_ReceiveAnalyze(void)
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
	HAL_UART_AbortReceive_IT(&huart1);
	HAL_UART_Receive_IT(&huart1, ReceiveBuffer, RBLENGTH);
	
	//查找帧头或帧尾
	static uint8_t Flag = 0;	//分析状态，0：查找帧尾，1查找帧尾后查找帧头，2发现完整帧
	static uint8_t frameStart;	//除帧头后的数据的第一个数据的位置
	static uint8_t frameEnd;	//除帧尾后的数据的最后一个数据的下一个位置
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
	
	//校验
	uint8_t sum = 0;
	for(uint8_t i = frameStart; i != frameEnd; i = NEXT(i))
	{
		sum += dataRepo[i];
	}
	if(sum != dataRepo[frameEnd])
	{
		return 5;
	}
	
	//分析完整帧
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
	
	//检查长度
	if(sizeof(ReceiveProtocolTransmit) != dataCounter)
	{
		return 7;
	}
	
	//数据分配
	ReceiveProtocolTransmit temp;
	for(uint8_t i = 0; i < sizeof(ReceiveProtocolTransmit); ++i)
	{
		((uint8_t *)&temp)[i] = dataArranged[i];
	}
	currentStatus.locked = temp.mode & 0x80;
	currentStatus.headMode = temp.mode & 0x40;
	currentStatus.power = temp.power;
	currentStatus.LR = temp.LR;
	currentStatus.FB = temp.FB;
	currentStatus.SP = temp.SP;
	currentStatus.adjust = temp.adjust;
	
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


/********************************	发送	*************************************/


void Bluetooth_Send(const SendProtocolDetail *pd)
{
	SendProtocolTransmit pt;
	
	memset(&pt, 0, sizeof(SendProtocolTransmit));
	
	pt.mode = pd->mode;
	pt.FBoffset = pd->FBoffset;
	pt.LRoffset = pd->LRoffset;
	
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
	
	SendBuffer[lendgth] = 0;
	for(uint8_t i = 2; i < lendgth; ++i)
	{
		SendBuffer[lendgth] += SendBuffer[i];
	}
	
	lendgth++;
	
	SendBuffer[lendgth++] = 0xFF;
	SendBuffer[lendgth++] = 0x02;
	
	HAL_UART_Transmit_IT(&huart1, (uint8_t *)(&SendBuffer), lendgth);
}





