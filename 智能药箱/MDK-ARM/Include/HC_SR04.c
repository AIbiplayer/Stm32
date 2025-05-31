/**
 * @brief 超声波控制(TIM3)
 * @date 2025/5/25
 */

#include "main.h"
#include "DHT11.h"

extern TIM_HandleTypeDef htim3;

uint8_t Measure_Status = 0; ///< 测量状态，已经测量为1，正在测量为0
uint32_t Up_Time = 0;       ///< 检测的上升时间
uint32_t Down_Time = 0;     ///< 检测的下降时间
uint32_t Pulse_Width = 0;   ///< 计算的脉冲宽度

/**
 * @brief 向模块发送信号，使用PE6接口发送
 */
void HC_Send_Trig(void)
{
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_SET);
    DHT11_Delay_us(75);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_RESET);
    Measure_Status = 0;
}

/**
 * @brief 获得测量距离，捕捉口为PE3
 * @param Temperature 温度
 */
float HC_Get_Measure(uint8_t Temperature)
{
    if (Measure_Status == 1)
    {
        float Distance = (Pulse_Width * (0.033145f + 0.000061 * Temperature)) / 2.0f; // 声速为340m/s，时间单位为us，距离单位为cm
        return Distance * 1.01f;
    }
    else
        return 0;
}

/**
 * @brief 上下沿捕捉
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{

    if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3) == GPIO_PIN_SET) // 检测上升沿数字
    {
        Up_Time = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);                             // 记录上升沿数字
        __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING); // 改为下降沿捕获
    }
    else
    {
        Measure_Status = 1;                                         // 测量完成
        Down_Time = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // 记录下降沿数字
        Pulse_Width = Down_Time - Up_Time;
        __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING); // 改为上升沿捕获
    }
}
