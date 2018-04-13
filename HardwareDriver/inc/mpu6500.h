#ifndef __MPU6500_H
#define __MPU6500_H


#include "stm32f3xx_hal.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "invensense.h"
#include "invensense_adv.h"
#include "fusion_9axis.h"
#include "eMPL_outputs.h"
#include "mltypes.h"
#include "mpu.h"




//������ٶȽṹ��
typedef struct
{
	//��ȡԭʼֵ
	int16_t ox;
	int16_t oy;
	int16_t oz;
	//ʵ��ֵ
	float x;
	float y;
	float z;
}Acce_Type;


//������ٶȽṹ��
typedef struct
{
	//��ȡԭʼֵ
	int16_t ox;
	int16_t oy;
	int16_t oz;
	//ʵ��ֵ
	float x;
	float y;
	float z;
}Gyro_Type;


//����شŽṹ��
typedef struct
{
	//��ȡԭʼֵ
	int16_t ox;
	int16_t oy;
	int16_t oz;
	//ʵ��ֵ
	float x;
	float y;
	float z;
}Magn_Type;


//ŷ���ǽṹ��
typedef struct
{
	float Pitch;
	float Roll;
	float Yaw;
}Euler_Type;

//��Ԫ���ṹ��(4��1�еľ���)
typedef struct
{
	float q0;
	float q1;
	float q2;
	float q3;
}Quat_Type;


uint8_t MPU6500_SetEulerOffset(float offset[3]);
uint8_t MPU6500_GetEulerOffset(float offset[3]);
uint8_t MPU6500_check(void);
uint8_t MPU6500_InitConfig(void);
uint8_t MPU6500_GetIMUMotion(Acce_Type *A, Gyro_Type *G);
uint8_t MPU6500_eMPLEular(Euler_Type *E, Acce_Type *A, Gyro_Type *G, inv_time_t T);
uint8_t MPU6500_GetEular(Euler_Type *E, inv_time_t T);


#endif
