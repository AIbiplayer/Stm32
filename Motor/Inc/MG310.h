/**
* @file MG310.h
 * @brief 电机驱动模块
 * @date 2025/11/26
 */

#ifndef MG310_H
#define MG310_H

#include "tim.h"
#include "PID.h"

#define ENCODERA_TIM htim5
#define ENCODERB_TIM htim8
#define ENCODERC_TIM htim4
#define ENCODERD_TIM htim3
#define GAP_TIM     htim14//电机速度计算定时器,1ms

typedef enum
{
    MOTOR_NORMAL = 0,
    MOTOR_REVERSE
} Reverse_e;

typedef struct
{
    float Speed; //目标速度
    PID_Typedef PID; //电机速度PID结构体
    float Pulse_Output; // 电机脉冲输出
    TIM_HandleTypeDef* TIMx; //电机对应的定时器
    float (*Motor_GetSpeed)(TIM_HandleTypeDef*); //电机速度
} Motor_Instance_s;

void MG310_Init(void);
void MG310_Drive(void);
void MG310_ChangePID(float Kp, float Ki, float Kd);
float GetSpeed(TIM_HandleTypeDef* htim);

#endif //MG310_H
