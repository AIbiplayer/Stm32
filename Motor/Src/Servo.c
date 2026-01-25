/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2025-12-07 19:51:53
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2025-12-23 15:26:28
 * @FilePath: \MDK-ARMe:\keilproject\gimbal_v2.0\Motor\Servo.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "../Inc/Servo.h"
#include "tim.h"

/**
 * @brief 初始化舵机
 */
void Servo_Init(void)
{
    HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);

    Servo_SetAngle(YAW, 0.0f);//初始化yaw轴为0度
    Servo_SetAngle(PITCH, 0.0f);//初始化pitch轴为0度
}

/**
 * @brief 设置舵机角度
 * @param channel 选择是yaw轴还是pitch轴
 * @param angle 舵机角度
 */
void Servo_SetAngle(uint8_t channel, float angle)
{
    switch (channel)
    {
        case YAW:
        /* code */
            if(angle > 0)
                __HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_1, angle/135.0f*500.0f+750.0f);
            else if(angle < 0)
                __HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_1, 750.0f+angle/135.0f*500.0f);
            else
                __HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_1, 750.0f);
        break;
        /* code */
        case PITCH:
            if(angle > 0)
                __HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_2, angle/90.0f*500.0f+750.0f);
            else if(angle < 0)
                __HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_2, 750.0f+angle/90.0f*500.0f);
            else
                __HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_2, 750.0f);
        break;
    }
}

/**
 * @brief 获取舵机角度
 * @param channel 选择是yaw轴还是pitch轴
 */
float Servo_GetAngle(uint8_t channel)
{
    float angle;
    uint32_t dutyCycle;
    switch (channel)
    {
        case YAW:
            dutyCycle = __HAL_TIM_GetCompare(&htim9, TIM_CHANNEL_1);
            angle = (dutyCycle - 750.0f) * 135.0f / 500.0f;
            break;
        case PITCH:
            dutyCycle = __HAL_TIM_GetCompare(&htim9, TIM_CHANNEL_2);
            angle = (dutyCycle - 750.0f) * 90.0f / 500.0f;
            break;
        default:
            angle = 0.0f; // 默认返回0度
            break;
    }
    return angle;
}
