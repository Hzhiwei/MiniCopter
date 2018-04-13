#include "mpu6500.h"
#include "IIC.h"

#define MPU_ADDR 0x69

#define MPU_SELF_TESTX_REG		0X0D	//�Լ�Ĵ���X
#define MPU_SELF_TESTY_REG		0X0E	//�Լ�Ĵ���Y
#define MPU_SELF_TESTZ_REG		0X0F	//�Լ�Ĵ���Z
#define MPU_SELF_TESTA_REG		0X10	//�Լ�Ĵ���A
#define MPU_SAMPLE_RATE_REG		0X19	//����Ƶ�ʷ�Ƶ��
#define MPU_CFG_REG				0X1A	//���üĴ���
#define MPU_GYRO_CFG_REG		0X1B	//���������üĴ���
#define MPU_ACCEL_CFG_REG		0X1C	//���ٶȼ����üĴ���
#define MPU_MOTION_DET_REG		0X1F	//�˶���ֵⷧ���üĴ���
#define MPU_FIFO_EN_REG			0X23	//FIFOʹ�ܼĴ���
#define MPU_I2CMST_CTRL_REG		0X24	//IIC�������ƼĴ���
#define MPU_I2CSLV0_ADDR_REG	0X25	//IIC�ӻ�0������ַ�Ĵ���
#define MPU_I2CSLV0_REG			0X26	//IIC�ӻ�0���ݵ�ַ�Ĵ���
#define MPU_I2CSLV0_CTRL_REG	0X27	//IIC�ӻ�0���ƼĴ���
#define MPU_I2CSLV1_ADDR_REG	0X28	//IIC�ӻ�1������ַ�Ĵ���
#define MPU_I2CSLV1_REG			0X29	//IIC�ӻ�1���ݵ�ַ�Ĵ���
#define MPU_I2CSLV1_CTRL_REG	0X2A	//IIC�ӻ�1���ƼĴ���
#define MPU_I2CSLV2_ADDR_REG	0X2B	//IIC�ӻ�2������ַ�Ĵ���
#define MPU_I2CSLV2_REG			0X2C	//IIC�ӻ�2���ݵ�ַ�Ĵ���
#define MPU_I2CSLV2_CTRL_REG	0X2D	//IIC�ӻ�2���ƼĴ���
#define MPU_I2CSLV3_ADDR_REG	0X2E	//IIC�ӻ�3������ַ�Ĵ���
#define MPU_I2CSLV3_REG			0X2F	//IIC�ӻ�3���ݵ�ַ�Ĵ���
#define MPU_I2CSLV3_CTRL_REG	0X30	//IIC�ӻ�3���ƼĴ���
#define MPU_I2CSLV4_ADDR_REG	0X31	//IIC�ӻ�4������ַ�Ĵ���
#define MPU_I2CSLV4_REG			0X32	//IIC�ӻ�4���ݵ�ַ�Ĵ���
#define MPU_I2CSLV4_DO_REG		0X33	//IIC�ӻ�4д���ݼĴ���
#define MPU_I2CSLV4_CTRL_REG	0X34	//IIC�ӻ�4���ƼĴ���
#define MPU_I2CSLV4_DI_REG		0X35	//IIC�ӻ�4�����ݼĴ���

#define MPU_I2CMST_STA_REG		0X36	//IIC����״̬�Ĵ���
#define MPU_INTBP_CFG_REG		0X37	//�ж�/��·���üĴ���
#define MPU_INT_EN_REG			0X38	//�ж�ʹ�ܼĴ���
#define MPU_INT_STA_REG			0X3A	//�ж�״̬�Ĵ���

#define MPU_ACCEL_XOUTH_REG		0X3B	//���ٶ�ֵ,X���8λ�Ĵ���
#define MPU_ACCEL_XOUTL_REG		0X3C	//���ٶ�ֵ,X���8λ�Ĵ���
#define MPU_ACCEL_YOUTH_REG		0X3D	//���ٶ�ֵ,Y���8λ�Ĵ���
#define MPU_ACCEL_YOUTL_REG		0X3E	//���ٶ�ֵ,Y���8λ�Ĵ���
#define MPU_ACCEL_ZOUTH_REG		0X3F	//���ٶ�ֵ,Z���8λ�Ĵ���
#define MPU_ACCEL_ZOUTL_REG		0X40	//���ٶ�ֵ,Z���8λ�Ĵ���

#define MPU_TEMP_OUTH_REG		0X41	//�¶�ֵ�߰�λ�Ĵ���
#define MPU_TEMP_OUTL_REG		0X42	//�¶�ֵ��8λ�Ĵ���

#define MPU_GYRO_XOUTH_REG		0X43	//������ֵ,X���8λ�Ĵ���
#define MPU_GYRO_XOUTL_REG		0X44	//������ֵ,X���8λ�Ĵ���
#define MPU_GYRO_YOUTH_REG		0X45	//������ֵ,Y���8λ�Ĵ���
#define MPU_GYRO_YOUTL_REG		0X46	//������ֵ,Y���8λ�Ĵ���
#define MPU_GYRO_ZOUTH_REG		0X47	//������ֵ,Z���8λ�Ĵ���
#define MPU_GYRO_ZOUTL_REG		0X48	//������ֵ,Z���8λ�Ĵ���

#define MPU_I2CSLV0_DO_REG		0X63	//IIC�ӻ�0���ݼĴ���
#define MPU_I2CSLV1_DO_REG		0X64	//IIC�ӻ�1���ݼĴ���
#define MPU_I2CSLV2_DO_REG		0X65	//IIC�ӻ�2���ݼĴ���
#define MPU_I2CSLV3_DO_REG		0X66	//IIC�ӻ�3���ݼĴ���

#define MPU_I2CMST_DELAY_REG	0X67	//IIC������ʱ����Ĵ���
#define MPU_SIGPATH_RST_REG		0X68	//�ź�ͨ����λ�Ĵ���
#define MPU_MDETECT_CTRL_REG	0X69	//�˶������ƼĴ���
#define MPU_USER_CTRL_REG		0X6A	//�û����ƼĴ���
#define MPU_PWR_MGMT1_REG		0X6B	//��Դ����Ĵ���1
#define MPU_PWR_MGMT2_REG		0X6C	//��Դ����Ĵ���2 
#define MPU_FIFO_CNTH_REG		0X72	//FIFO�����Ĵ����߰�λ
#define MPU_FIFO_CNTL_REG		0X73	//FIFO�����Ĵ����Ͱ�λ
#define MPU_FIFO_RW_REG			0X74	//FIFO��д�Ĵ���
#define MPU_DEVICE_ID_REG		0X75	//����ID�Ĵ���

#define AKM_ADDR			(0x49)

#define AKM_REG_WHOAMI      (0x00)

#define AKM_REG_ST1         (0x02)
#define AKM_REG_HXL         (0x03)
#define AKM_REG_ST2         (0x09)

#define AKM_REG_CNTL        (0x0A)
#define AKM_REG_ASTC        (0x0C)
#define AKM_REG_ASAX        (0x10)
#define AKM_REG_ASAY        (0x11)
#define AKM_REG_ASAZ        (0x12)

#define AKM_DATA_READY      (0x01)
#define AKM_DATA_OVERRUN    (0x02)
#define AKM_OVERFLOW        (0x80)
#define AKM_DATA_ERROR      (0x40)

#define AKM_BIT_SELF_TEST   (0x40)

#define AKM_POWER_DOWN          (0x00 | SUPPORTS_AK89xx_HIGH_SENS)
#define AKM_SINGLE_MEASUREMENT  (0x01 | SUPPORTS_AK89xx_HIGH_SENS)
#define AKM_FUSE_ROM_ACCESS     (0x0F | SUPPORTS_AK89xx_HIGH_SENS)
#define AKM_MODE_SELF_TEST      (0x08 | SUPPORTS_AK89xx_HIGH_SENS)

#define AKM_WHOAMI      (0x48)


#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)



unsigned char *mpl_key = (unsigned char*)"eMPL 5.1";		//ʹ��empl����붨��

static int16_t GyroOffset[3] = {7, 16, 23};		//���ٶ�ƫ��
float EulerOffset[3] = {1.11, 0.89, 0};

void mpu_accel_lpfset(void);


uint8_t MPU6500_SetEulerOffset(float offset[3])
{
	EulerOffset[0] = offset[0];
	EulerOffset[1] = offset[1];
	EulerOffset[2] = offset[2];
}

uint8_t MPU6500_GetEulerOffset(float offset[3])
{
	offset[0] = EulerOffset[0];
	offset[1] = EulerOffset[1];
	offset[2] = EulerOffset[2];
}


uint8_t MPU6500_InitConfig(void)
{
	//���MPU6500�Ƿ����
	if(MPU6500_check())
	{
		return 1;
	}
	
	
	struct platform_data_s {
		signed char orientation[9];
	};
	
	inv_error_t result;
    unsigned char accel_fsr;
    unsigned short gyro_rate, gyro_fsr;
    struct int_param_s int_param;
	static struct platform_data_s gyro_pdata = {
		.orientation = { 1, 0, 0,
						 0, 1, 0,
						 0, 0, 1}
	};
	
	result = mpu_init(&int_param);
	if(result)
	{
		return 1;
	}
	result = inv_init_mpl();
	if(result)
	{
		return 1;
	}
    inv_enable_quaternion();
    inv_enable_9x_sensor_fusion();
    inv_enable_fast_nomot();
    inv_enable_gyro_tc();
    inv_enable_eMPL_outputs();
	result = inv_start_mpl();
	if(result)
	{
		return 1;
	}
	
	mpu_set_lpf(30);

    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    mpu_set_sample_rate(200);
    mpu_get_sample_rate(&gyro_rate);
    mpu_get_gyro_fsr(&gyro_fsr);
    mpu_get_accel_fsr(&accel_fsr);
    inv_set_gyro_sample_rate(1000000L / gyro_rate);
    inv_set_accel_sample_rate(1000000L / gyro_rate);
    inv_set_gyro_orientation_and_scale(
            inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
            (long)gyro_fsr<<15);
    inv_set_accel_orientation_and_scale(
            inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
            (long)accel_fsr<<15);
	
     mpu_accel_lpfset();
	
	return 0;
}


uint8_t MPU6500_check(void)
{
	uint8_t temp = 0;
	IIC_SingleRead(MPU_ADDR, 0x75, &temp);
	
	if(temp == 0x70)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}


/**
  * @brief  ��ȡ���ٶ�����ٶ�����(g,��/s)
  * @param  x���ٶȴ�ŵ�ַ
  * @param  y���ٶȴ�ŵ�ַ
  * @param  z���ٶȴ�ŵ�ַ
  * @param  x���ٶȴ�ŵ�ַ
  * @param  y���ٶȴ�ŵ�ַ
  * @param  z���ٶȴ�ŵ�ַ
  * @retval 1��ȡ�ɹ�		0��ȡʧ��
  */
uint8_t MPU6500_GetIMUMotion(Acce_Type *A, Gyro_Type *G)
{
	uint8_t buffer[14];
	
	if(IIC_ReadBuffer(MPU_ADDR, MPU_ACCEL_XOUTH_REG, 14, buffer))
	{
		return 0;
	}
	
	A->ox = ((((int16_t)buffer[0]) << 8) | buffer[1]);
	A->oy = ((((int16_t)buffer[2]) << 8) | buffer[3]);
	A->oz = ((((int16_t)buffer[4]) << 8) | buffer[5]);
	G->ox = ((((int16_t)buffer[8]) << 8) | buffer[9]);
	G->oy = ((((int16_t)buffer[10]) << 8) | buffer[11]);
	G->oz = ((((int16_t)buffer[12]) << 8) | buffer[13]);
	
	G->ox -= GyroOffset[0];
	G->oy -= GyroOffset[1];
	G->oz -= GyroOffset[2];
	
	A->x = A->ox / 65535.0f * 4.0f;
	A->y = A->oy / 65535.0f * 4.0f;
	A->z = A->oz / 65535.0f * 4.0f;
	G->x = G->ox / 65535.0f * 4000.0f;
	G->y = G->oy / 65535.0f * 4000.0f;
	G->z = G->oz / 65535.0f * 4000.0f;
	
	return 1;
}


uint8_t MPU6500_eMPLEular(Euler_Type *E, Acce_Type *A, Gyro_Type *G, inv_time_t T)
{
	long temp[3];
	long temperature;
	long data[3];
	int8_t accuracy;
	int8_t result;
	
	static float lastYaw = 0, currentYaw;
	
	unsigned long timestamp;
	
	result = mpu_get_accel_reg(&(A->ox), NULL);
	if(result)
	{
		return 1;
	}
	temp[0] = A->ox;
	temp[1] = A->oy;
	temp[2] = A->oz;
	result = inv_build_accel(temp, 0, ((unsigned long)T));
	if(result)
	{
		return 1;
	}
	
	result = mpu_get_gyro_reg(&(G->ox), NULL);
	if(result)
	{
		return 1;
	}
	result = inv_build_gyro(&(G->ox), ((unsigned long)T));
	if(result)
	{
		return 1;
	}
	
	result = mpu_get_temperature(&temperature, NULL);
	if(result)
	{
		return 1;
	}
	result = inv_build_temp(temperature, ((unsigned long)T));
	if(result)
	{
		return 1;
	}
	
	inv_execute_on_data();
	
	inv_get_sensor_type_euler(data, &accuracy, &timestamp);
	
	A->x = A->ox / 65535.0f * 4.0f;
	A->y = A->oy / 65535.0f * 4.0f;
	A->z = A->oz / 65535.0f * 4.0f;
	
	G->ox -= GyroOffset[0];
	G->oy -= GyroOffset[1];
	G->oz -= GyroOffset[2];
	
	G->x = G->ox / 65535.0f * 4000.0f;
	G->y = G->oy / 65535.0f * 4000.0f;
	G->z = G->oz / 65535.0f * 4000.0f;
	
	lastYaw = currentYaw;
	
	E->Roll = data[0] / 65536.0f - EulerOffset[1];
	E->Pitch = data[1] / 65536.0f - EulerOffset[0];
	currentYaw = data[2] / 65536.0f - EulerOffset[2];
	
	if((currentYaw > 90) && (lastYaw < -90))
	{
		E->Yaw -= (180 - currentYaw) + (180 + lastYaw);
	}
	else if((currentYaw < -90) && (lastYaw > 90))
	{
		E->Yaw += (180 - lastYaw) + (180 + currentYaw);
	}
	else
	{
		E->Yaw += currentYaw - lastYaw;
	}
	
	return 0;
}



uint8_t MPU6500_GetEular(Euler_Type *E, inv_time_t T)
{
	Acce_Type CurrentAcc;
	Gyro_Type CurrentGyro;
	if(!MPU6500_GetIMUMotion(&CurrentAcc, &CurrentGyro))
	{
		return 0;
	}
	if(MPU6500_eMPLEular(E, &CurrentAcc, &CurrentGyro, T))
	{
		return 0;
	}
	return 1;
}


/**
  * @brief  ���ٶȼƵ�ͨ�˲�����
  * @param  void
  * @retval void
  */
void mpu_accel_lpfset(void)
{
    uint8_t data = 2;
    IIC_SendBuffer(MPU_ADDR, 0x1A, 1, &data);
}

