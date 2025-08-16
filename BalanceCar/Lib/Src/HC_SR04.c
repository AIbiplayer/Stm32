/**
*@file HC_SR04.c
 *@brief 超声波测距
 *@author 申飞麟
 *@date 2025/8/13
 **/

#include "HC_SR04.h"
#include "main.h"
#include "stdbool.h"

float Distance = 0; // 距离
bool GetDistance_Status = 0; // 获取距离状态

extern TIM_HandleTypeDef htim1;

/**
 * @brief 启动DWT计数器以实现精准延时
 */
void Delay_Us_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/**
 * @brief 延迟函数，采用DWT计数器
 * @param 延时的时间
 */
void Delay_Us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t cycles = us * (SystemCoreClock / 1000000);
    while ((DWT->CYCCNT - start) < cycles);
}

/**
 * @brief 捕捉中断，用于计算距离
 */
void EXTI15_10_IRQHandler(void)
{
    float High_Level_Time = 0.0f; // 高电平持续时间
    __HAL_TIM_SetCounter(&htim1, 0);
    HAL_TIM_Base_Start(&htim1);

    while (HAL_GPIO_ReadPin(HC_SR04_Echo_GPIO_Port,HC_SR04_Echo_Pin) == GPIO_PIN_SET) // 检测到高电平
    {
        High_Level_Time = __HAL_TIM_GetCounter(&htim1);
        if (High_Level_Time > 5000) // 距离过大，结束
            break;
    }
    HAL_TIM_Base_Stop(&htim1);

    GetDistance_Status = 1;
    Distance = High_Level_Time / 1000000.0f * 340.0f / 2.0f * 100.0f; // 距离计算

    HAL_GPIO_EXTI_IRQHandler(HC_SR04_Echo_Pin);
}

/**
 * @brief 超声波发送指令
 */
void HC_SR04_Send_Trig(void)
{
    GetDistance_Status = 0;
    HAL_GPIO_WritePin(HC_SR04_Trig_GPIO_Port,HC_SR04_Trig_Pin, GPIO_PIN_SET);
    Delay_Us(20);
    HAL_GPIO_WritePin(HC_SR04_Trig_GPIO_Port,HC_SR04_Trig_Pin, GPIO_PIN_RESET);
}
