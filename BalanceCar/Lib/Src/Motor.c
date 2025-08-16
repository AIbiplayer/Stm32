/**
 *@file Motor.c
 *@brief 电机控制模块
 *@author 申飞麟
 *@date 2025/8/13
 **/

#include "main.h"
#include "Motor.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

/**
 * @brief 电机停转
 */
void Motor_Stop(void)
{
    HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET); //AIN
    HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET); //BIN
    HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
}

/**
 * @brief 输出值加载至电机
 * @param OutPut_Left 左电机输出值
 * @param OutPut_Right 右电机输出值
 */
void Motor_Load(int16_t OutPut_Left, int16_t OutPut_Right)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET); // STBY启动
    if (OutPut_Left > 0)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // 向后方向控制
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, OutPut_Left);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); // 向前方向控制
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
        OutPut_Left = -OutPut_Left;
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, OutPut_Left);
    }
    if (OutPut_Right > 0)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); // 向后方向控制
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, OutPut_Right);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET); // 向前方向控制
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
        OutPut_Right = -OutPut_Right;
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, OutPut_Right);
    }
}
