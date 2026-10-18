#ifndef __MYMPU6050_REG_H
#define __MYMPU6050_REG_H

#define	MPU6050_SMPLRT_DIV		0x19//采样率分频  1000/ （MPU6050_SMPLRT_DIV +1）
#define	MPU6050_CONFIG			  0x1A//滤波器 输入越大越平滑但延迟更高
#define	MPU6050_GYRO_CONFIG		0x1B//陀螺仪量程 系数=32768/满量程0x00 = ±250°/s，0x08 = ±500°/s
#define	MPU6050_ACCEL_CONFIG	0x1C//加速度计量程 越大系数越小 bit4:3

//因为数据是2字节所以要分为前半部分和后半部分，也就是高字节和低字节
#define	MPU6050_ACCEL_XOUT_H	0x3B//X 轴高字节
#define	MPU6050_ACCEL_XOUT_L	0x3C//X 轴低字节
#define	MPU6050_ACCEL_YOUT_H	0x3D//Y 轴高字节
#define	MPU6050_ACCEL_YOUT_L	0x3E//Y 轴低字节
#define	MPU6050_ACCEL_ZOUT_H	0x3F//Z 轴高字节
#define	MPU6050_ACCEL_ZOUT_L	0x40//Z 轴低字节
#define	MPU6050_TEMP_OUT_H		0x41//温度高字节
#define	MPU6050_TEMP_OUT_L		0x42//温度低字节
#define	MPU6050_GYRO_XOUT_H		0x43//X 轴高字节
#define	MPU6050_GYRO_XOUT_L		0x44//X 轴低字节
#define	MPU6050_GYRO_YOUT_H		0x45//Y 轴高字节
#define	MPU6050_GYRO_YOUT_L		0x46//Y 轴低字节
#define	MPU6050_GYRO_ZOUT_H		0x47//Z 轴高字节
#define	MPU6050_GYRO_ZOUT_L		0x48//Z 轴低字节

#define	MPU6050_PWR_MGMT_1		0x6B//睡眠模式，读写之前唤醒一下，写 0x01 唤醒+配置时钟
#define	MPU6050_PWR_MGMT_2		0x6C//控制各个轴是否进入待机
#define	MPU6050_WHO_AM_I		  0x75//设备ID，但我的应该不是正版，输出不了0x68

#define MPU6050_REG_INT_PIN_CFG     0x37
#define MPU6050_REG_INT_ENABLE      0x38

#endif
