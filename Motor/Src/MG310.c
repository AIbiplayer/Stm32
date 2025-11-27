/**
* @file MG310.c
 * @brief 电机驱动模块
 * @date 2025/11/26
 *
 * @details
 */

#include "MG310.h"
#include "main.h"
#include "tim.h"

void Motor_Init(void)
{
    //PWM通道输出使能
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
    HAL_TIM_Encoder_Start(&ENCODERA_TIM, TIM_CHANNEL_1);
    HAL_TIM_Encoder_Start(&ENCODERA_TIM, TIM_CHANNEL_2);
    HAL_TIM_Encoder_Start(&ENCODERB_TIM, TIM_CHANNEL_1);
    HAL_TIM_Encoder_Start(&ENCODERB_TIM, TIM_CHANNEL_2);
    HAL_TIM_Encoder_Start(&ENCODERC_TIM, TIM_CHANNEL_1);
    HAL_TIM_Encoder_Start(&ENCODERC_TIM, TIM_CHANNEL_2);
    HAL_TIM_Encoder_Start(&ENCODERD_TIM, TIM_CHANNEL_1);
    HAL_TIM_Encoder_Start(&ENCODERD_TIM, TIM_CHANNEL_2);

    HAL_TIM_Base_Start_IT(&htim2);
    HAL_TIM_Base_Start_IT(&htim3);
    HAL_TIM_Base_Start_IT(&htim4);
    HAL_TIM_Base_Start_IT(&htim5);
    HAL_TIM_Base_Start_IT(&GAP_TIM);

    //清空上次上机装载CCR
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 0);

    __HAL_TIM_SET_COUNTER(&ENCODERA_TIM, 0);
    __HAL_TIM_SET_COUNTER(&ENCODERB_TIM, 0);
    __HAL_TIM_SET_COUNTER(&ENCODERC_TIM, 0);
    __HAL_TIM_SET_COUNTER(&ENCODERD_TIM, 0);

}
