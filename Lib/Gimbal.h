/**
 * @file Gimbal.h
 * @brief 云台控制代码
 * @date 2025/9/30
 */

#ifndef __GIMBAL_H__
#define __GIMBAL_H__

#include "main.h"

void Gimbal_Init(void);
void calibrate_GZ(void);
void kalmanFilter(void);

#endif // __GIMBAL_H__
