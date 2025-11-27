/**
 * @file Mpu6050.h
 * @brief 陀螺仪读取代码
 * @date 2025/10/8
 */

#ifndef __MPU6050_H
#define __MPU6050_H

#include "main.h"

void MPU6050_WriteReg(const uint8_t RegAddress, uint8_t Data);
uint8_t MPU6050_ReadReg(const uint8_t RegAddress);

void MPU6050_Init(void);
uint8_t MPU6050_GetID(void);
void MPU6050_GetData(int16_t* AccX, int16_t* AccY, int16_t* AccZ,
                     int16_t* GyroX, int16_t* GyroY, int16_t* GyroZ);

#endif
