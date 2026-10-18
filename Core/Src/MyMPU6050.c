#include "main.h"
#include "i2c.h"
#include "MyMPU6050_Reg.h"
#include "MyMPU6050.h"

#define OUCY_MPU6050 0xD0
static int16_t s_gyro_bias[3] = {0, 0, 0};  //调整零偏�?

#define FILTER_ALPHA        0.2f//滤波系数，越小越平滑

static float   s_filt_acc[3]  = {0.0f, 0.0f, 0.0f};
static float   s_filt_gyro[3] = {0.0f, 0.0f, 0.0f};
static uint8_t s_filt_first   = 1;//为了防止第一次导致结果偏�?

static void MPU6050_Filter(float *acc, float *gyro);
/*=================主要用的函数========================*/

static HAL_StatusTypeDef MPU6050_WriteReg(uint8_t reg, uint8_t content)
{
	return HAL_I2C_Mem_Write(
	&hi2c1,
	OUCY_MPU6050,
	reg,
	I2C_MEMADD_SIZE_8BIT,
	&content,
	1,
	100);
}

static HAL_StatusTypeDef MPU6050_ReadRegs(uint8_t reg, uint8_t *buf, uint16_t len)
{
	return HAL_I2C_Mem_Read(&hi2c1, 
	OUCY_MPU6050, 
	reg,
  I2C_MEMADD_SIZE_8BIT, 
	buf, 
	len, 
	100);
}

static uint8_t MPU6050_ReadReg(uint8_t reg)
{
	uint8_t content = 0;
	
	HAL_I2C_Mem_Read(&hi2c1,
  OUCY_MPU6050,
  reg,
  I2C_MEMADD_SIZE_8BIT,
  &content,
  1,
  100);
	
	return content;
}

/*=====================初始�?===========================*/

uint8_t MyMPU6050_Init(void)
{
	uint8_t MyID;
	MyID =MPU6050_ReadReg(MPU6050_WHO_AM_I);
	
	if ((MyID == 0x00) || (MyID == 0xFF))
    {
        return 1;
    }
		
	MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x00);//唤醒
	HAL_Delay(50);  
		
	MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);//选择时钟
  HAL_Delay(10);
		
	MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 9);        /* 1000/(1+9) = 100Hz */

  MPU6050_WriteReg(MPU6050_CONFIG, 0x03);

  MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x00);

  MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x00);
		
	if (MPU6050_ReadReg(MPU6050_PWR_MGMT_1) != 0x01)
  {
     return 2;
  }
		
	return 0;
}

uint8_t MyMPU6050_GetID(void)
{
    return MPU6050_ReadReg(MPU6050_WHO_AM_I);
}

/*====================显示数据======================*/

void MyMPU6050_ReadRaw(int16_t *acc, int16_t *gyro, int16_t *temp)
{
   uint8_t buf[14];

    // 一次读�?14 个字节（加速度 6 + 温度 2 + 陀�?6�?
   MPU6050_ReadRegs(MPU6050_ACCEL_XOUT_H, buf, 14);

    // 拼接：高字节在前，低字节在后�?
   acc[0] = (int16_t)((buf[0]  << 8) | buf[1]);    /* 加速度 X */
   acc[1] = (int16_t)((buf[2]  << 8) | buf[3]);    /* 加速度 Y */
   acc[2] = (int16_t)((buf[4]  << 8) | buf[5]);    /* 加速度 Z */

  if (temp != NULL)
  {
     *temp = (int16_t)((buf[6] << 8) | buf[7]);  /* 温度 */
  }

  gyro[0] = (int16_t)((buf[8]  << 8) | buf[9]);   /* 陀�?X */
  gyro[1] = (int16_t)((buf[10] << 8) | buf[11]);  /* 陀�?Y */
  gyro[2] = (int16_t)((buf[12] << 8) | buf[13]);  /* 陀�?Z */
		
	gyro[0] -= s_gyro_bias[0];
	gyro[1] -= s_gyro_bias[1];
	gyro[2] -= s_gyro_bias[2];
}

void MyMPU6050_ReadPhysical(float *acc, float *gyro, float *temp)
{
  int16_t acc_raw[3];
  int16_t gyro_raw[3];
  int16_t temp_raw;

  MyMPU6050_ReadRaw(acc_raw, gyro_raw, &temp_raw);

  acc[0] = (float)acc_raw[0] / ACCEL_SCALE;
  acc[1] = (float)acc_raw[1] / ACCEL_SCALE;
  acc[2] = (float)acc_raw[2] / ACCEL_SCALE;

  gyro[0] = (float)gyro_raw[0] / GYRO_SCALE;
  gyro[1] = (float)gyro_raw[1] / GYRO_SCALE;
  gyro[2] = (float)gyro_raw[2] / GYRO_SCALE;
	
	MPU6050_Filter(acc, gyro);

  if (temp != NULL)
  {
      *temp = (float)temp_raw / TEMP_SCALE + TEMP_OFFSET;
  }
}

/*======================解决偏移函数========================*/

//原理特简单，将板子静止，多次读取此时陀螺仪的值，再除以数据的个数�?
//就能算出每次误差的大概值，然后减去就行�?
uint8_t MyMPU6050_CalibrateGyro(uint16_t samples)
{
	int32_t sum[3] = {0,0,0};
	int16_t  acc[3];
  int16_t  gyro[3];
  int16_t  temp;
	
	s_gyro_bias[0] = 0;
  s_gyro_bias[1] = 0;
  s_gyro_bias[2] = 0;
	
  uint16_t i;
	for (i = 0; i< samples; i++)
	{
		MyMPU6050_ReadRaw(acc,gyro,&temp);
		
		sum[0] += gyro[0];
    sum[1] += gyro[1];
    sum[2] += gyro[2];
	}
	
	s_gyro_bias[0] = (int16_t)(sum[0] / (int32_t)samples);
  s_gyro_bias[1] = (int16_t)(sum[1] / (int32_t)samples);
  s_gyro_bias[2] = (int16_t)(sum[2] / (int32_t)samples);

  return 0;
	
}


void MyMPU6050_GetGyroBias(int16_t *bias)
{
  bias[0] = s_gyro_bias[0];
  bias[1] = s_gyro_bias[1];
  bias[2] = s_gyro_bias[2];
}

static void MPU6050_Filter(float *acc, float *gyro)
{
  uint8_t i;

  if (s_filt_first != 0)
  {
		for (i = 0; i < 3; i++)
    {
			s_filt_acc[i]  = acc[i];
			s_filt_gyro[i] = gyro[i];
    }
      s_filt_first = 0;
    }
   else
   {
			for (i = 0; i < 3; i++)//原理很简�?.2的新�?0.8的旧�?
      {
				s_filt_acc[i]  = FILTER_ALPHA * acc[i]  + (1.0f - FILTER_ALPHA) * s_filt_acc[i];
				s_filt_gyro[i] = FILTER_ALPHA * gyro[i] + (1.0f - FILTER_ALPHA) * s_filt_gyro[i];
      }
		}

    for (i = 0; i < 3; i++)
    {
        acc[i]  = s_filt_acc[i];
        gyro[i] = s_filt_gyro[i];
    }
}

void MyMPU6050_SetFilterAlpha(float alpha)
{
    if (alpha < 0.0f) { alpha = 0.0f; }
    if (alpha > 1.0f) { alpha = 1.0f; }
}
//中断
void MyMPU6050_EnableDataReadyInt(void)
{
    MPU6050_WriteReg(MPU6050_REG_INT_PIN_CFG, 0x00);//手册里说是每次数据准备就绪发出推挽输�?0微秒脉冲
    MPU6050_WriteReg(MPU6050_REG_INT_ENABLE, 0x01);//数据准备就绪
}
