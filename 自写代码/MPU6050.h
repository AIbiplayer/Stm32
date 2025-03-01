/**
 *@file MPU6050.h
 *@author 申飞麟
 *@brief MPU6050操作函数（软件）, 默认没有IIC_Init()
 *@date 2025/2/27
 */

#ifndef MPU6050_H
#define MPU6050_H

#include "stdint.h"


typedef struct SensorData ///< 传感器数据
{
    int16_t GYRO_X;  ///< 陀螺仪X轴
    int16_t GYRO_Y;  ///< 陀螺仪Y轴
    int16_t GYRO_Z;  ///< 陀螺仪Z轴
    int16_t ACCEL_X; ///< 加速度X轴
    int16_t ACCEL_Y; ///< 加速度Y轴
    int16_t ACCEL_Z; ///< 加速度Z轴
} SensorData;

void MPU6050_Init(void);
void MPU6050_W(uint8_t Address, uint8_t Data);
uint8_t MPU6050_R(uint8_t Address);
void MPU6050_GetData(SensorData *Sensor);

#endif
