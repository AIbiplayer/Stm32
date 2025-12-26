/**
* @file HC_SR04.c
 * @brief 超声波控制
 * @author Shen FeiLin
 * @date 2025/12/12
 */

#include "HC_SR04.h"
#include "main.h"
#include "tim.h"
#include "user_lib.h"
#include "bsp_dwt.h"
#include "DJI_Motor.h"

static uint8_t Measure_Status = 0; ///< 测量状态，已经测量为1，正在测量为0
static uint32_t Pulse_Width = 0; ///< 计算的脉冲宽度
static float Distance = 0; ///< 计算的距离
static float Last_Distance = 0; ///< 上次计算的距离

/**
 * @brief 超声波模块初始化
 */
void HC_Init(void)
{
    HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1); // 启动超声波输入捕获
}

/**
 * @brief 向模块发送信号，使用PE6接口发送
 */
void HC_Send_Trig(void)
{
    uint32_t delay_us = DWT_GetTimeline_us();
    HAL_GPIO_WritePin(HC_SR04_TRIG_GPIO_Port, HC_SR04_TRIG_Pin, GPIO_PIN_SET);
    while (delay_us - DWT_GetTimeline_us() < 75);
    HAL_GPIO_WritePin(HC_SR04_TRIG_GPIO_Port, HC_SR04_TRIG_Pin, GPIO_PIN_RESET);
    Measure_Status = 0;
}

/**
 * @brief 获得测量距离
 * @return 距离，单位cm
 */
float HC_Get_Measure(void)
{
    if (Measure_Status == 1)
    {
        Last_Distance = Distance;
        Distance = (float)Pulse_Width * 0.033145f / 2.0f; // 声速为340m/s，时间单位为us，距离单位为cm
        Distance = Distance >= 80.0f ? 80.0f : Distance; // 最大测量距离
        Distance = fabsf(Distance - Last_Distance) > 20.0f ? Last_Distance : Distance; // 突变过滤
        Distance = 0.6f * Distance + 0.4f * Last_Distance; // 简单滤波
        return Distance;
    }
    return 0;
}

/**
 * @brief 上下沿捕捉
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim)
{
    static uint32_t Up_Time = 0; ///< 检测的上升时间
    static uint32_t Down_Time = 0; ///< 检测的下降时间
    if (HAL_GPIO_ReadPin(HC_SR04_ECHO_GPIO_Port, HC_SR04_ECHO_Pin) == GPIO_PIN_SET) // 检测上升沿数字
    {
        Up_Time = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // 记录上升沿数字
        __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING); // 改为下降沿捕获
    }
    else
    {
        Measure_Status = 1; // 测量完成
        Down_Time = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // 记录下降沿数字
        Pulse_Width = Down_Time - Up_Time;
        __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING); // 改为上升沿捕获
    }
}
