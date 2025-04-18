#ifndef PID_H
#define PID_H

#include "stdint.h"
#include "stm32f1xx.h"

int32_t Read_Speed(TIM_HandleTypeDef *htim);
float PD_Control(float Ex_Angle, float Angle, float Gyro);
float PI_Control(int32_t EX_Vercity, int32_t Encoder_L, int32_t Encoder_R);
float T_Control(int32_t Ex_Angle, float Gyro);
void Control(void);
void Load_Motor(int32_t M1, int32_t M2);

#endif
