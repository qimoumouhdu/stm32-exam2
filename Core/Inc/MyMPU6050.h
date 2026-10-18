#ifndef __MYMPU6050_H
#define __MYMPU6050_H

#include <stdint.h>

#define ACCEL_SCALE         16384.0f    //±2g
#define GYRO_SCALE          131.0f      //±250°/s
#define TEMP_SCALE          340.0f      //温度系数，固定值
#define TEMP_OFFSET         36.53f      //温度偏置，固定值

uint8_t MPU6050_ReadReg(uint8_t reg);
HAL_StatusTypeDef MPU6050_ReadRegs(uint8_t reg, uint8_t *buf, uint16_t len);
HAL_StatusTypeDef MPU6050_WriteReg(uint8_t reg, uint8_t content);


uint8_t MyMPU6050_Init(void);
uint8_t MyMPU6050_GetID(void);

void MyMPU6050_ReadRaw(int16_t *acc, int16_t *gyro, int16_t *temp);
void MyMPU6050_ReadPhysical(float *acc, float *gyro, float *temp);

uint8_t MyMPU6050_CalibrateGyro(uint16_t samples);
void    MyMPU6050_GetGyroBias(int16_t *bias);
void MPU6050_Filter(float *acc, float *gyro);
void MyMPU6050_SetFilterAlpha(float alpha);
void MyMPU6050_EnableDataReadyInt(void);




#endif
