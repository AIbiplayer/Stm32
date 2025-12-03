/**
* @file MG310.c
 * @brief 电机驱动模块
 * @date 2025/11/26
 */

#include "Chassis_Task.h"
#include "MG310.h"
#include "main.h"
#include "tim.h"

//电机实例，从0到3依次为ABCD电机
Motor_Instance_s MG310[4] = {0};
PID_Typedef Motor_PID;

/**
 * @brief 电机驱动初始化
 * @note 通道一个一个开，防止有BUG
 * @todo 增加电机控制回调
 */
void MG310_Init(void)
{
    //PWM通道输出使能
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
    HAL_TIM_Encoder_Start(&ENCODERA_TIM, TIM_CHANNEL_1);
    HAL_TIM_Encoder_Start(&ENCODERA_TIM, TIM_CHANNEL_2);
    HAL_TIM_Encoder_Start(&ENCODERB_TIM, TIM_CHANNEL_1);
    HAL_TIM_Encoder_Start(&ENCODERB_TIM, TIM_CHANNEL_2);
    HAL_TIM_Encoder_Start(&ENCODERC_TIM, TIM_CHANNEL_1);
    HAL_TIM_Encoder_Start(&ENCODERC_TIM, TIM_CHANNEL_2);
    HAL_TIM_Encoder_Start(&ENCODERD_TIM, TIM_CHANNEL_1);
    HAL_TIM_Encoder_Start(&ENCODERD_TIM, TIM_CHANNEL_2);
    HAL_TIM_Base_Start_IT(&GAP_TIM);

    PID_Param(&Motor_PID, 10, 0, 1,
              Derivative_On_Measurement | Integral_Limit,
              1, 25, 20, 6750);

    MG310[0].TIMx = &ENCODERA_TIM;
    MG310[1].TIMx = &ENCODERB_TIM;
    MG310[2].TIMx = &ENCODERC_TIM;
    MG310[3].TIMx = &ENCODERD_TIM;
    for (uint8_t i = 0; i < 4; i++)
    {
        MG310[i].PID = Motor_PID;
        MG310[i].Motor_GetSpeed = GetSpeed;
    }
}

/**
 * @brief 获取电机速度函数
 * @param htim 电机对应的定时器句柄
 * @return 电机速度值
 */
int16_t GetSpeed(TIM_HandleTypeDef* htim)
{
    const int16_t speed = __HAL_TIM_GET_COUNTER(htim);
    __HAL_TIM_SET_COUNTER(htim, 0);
    return speed;
}

/**
 * @brief 电机驱动函数
 * @note 此函数循环调用，进行电机速度控制
 */
void MG310_Drive(void)
{
    // 电机A输出
    MG310[0].PID_Output > 0
            ? __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, MG310[0].PID_Output),
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0)
            : __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, -MG310[0].PID_Output),
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
    // 电机B输出
    MG310[1].PID_Output > 0
            ? __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, MG310[1].PID_Output),
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0)
            : __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, -MG310[1].PID_Output),
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
    // 电机C输出
    MG310[2].PID_Output > 0
            ? __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, MG310[2].PID_Output),
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0)
            : __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, -MG310[2].PID_Output),
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
    // 电机D输出
    MG310[3].PID_Output > 0
            ? __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, MG310[3].PID_Output),
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0)
            : __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, -MG310[3].PID_Output),
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
}
