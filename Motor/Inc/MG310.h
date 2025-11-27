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

typedef struct
{
    int16_t Speed : 15; //电机速度
    int16_t Direction : 1; //电机方向
    PID_Typedef PID; //电机速度PID结构体
} Motor;

void Motor_Init(void);

#endif //MG310_H
