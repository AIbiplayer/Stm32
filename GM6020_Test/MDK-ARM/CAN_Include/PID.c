/**
 * @file PID.c
 * @brief 简易PID程序,使用TIM中断
 * @date 2025/8/5
 */

#include "main.h"
#include "CAN_Func.h"

#define OUT_LIMIT 2000.0   // 输出限幅
#define ERROR_LIMIT 2000.0 // 误差积分限幅

float Target = 160.0; // 目标值
float Actual = 0.0;   // 实际值
float Out = 0.0;      // 输出值

float Kp = 38.5 * 0.87; // 比例环
float Kd = 1000.0;     // 微分环
float Ki = 0.21 * 0.83; // 积分环

float Error_Now = 0.0;  // 本次误差
float Error_Last = 0.0; // 上次误差
float Error_Sum = 0.0;  // 误差积分

extern TIM_HandleTypeDef htim2; // 句柄
extern CAN_HandleTypeDef hcan;

extern float Angle; // 机械角度，范围0~8192
extern float Temp;  // 温度
extern float Speed; // 转速，范围-320~320rpm
extern float I;     // 电流，范围-16384~16384，对应-3A~3A

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        Actual = Angle;
        Error_Last = Error_Now;
        Error_Now = Target - Actual;
        if (Error_Now > 180.0)
        {
            Error_Now -= 360.0;
        }
        else if (Error_Now <= -180.0)
        {
            Error_Now += 360.0;
        }
        Error_Sum += Error_Now;

        if (Error_Sum > ERROR_LIMIT) // 积分限幅
        {
            Error_Sum = ERROR_LIMIT;
        }
        else if (Error_Sum < -ERROR_LIMIT)
        {
            Error_Sum = -ERROR_LIMIT;
        }

        Out = Kp * Error_Now + Ki * Error_Sum + Kd * (Error_Now - Error_Last);

        if (Out > OUT_LIMIT)
        {
            Out = OUT_LIMIT; // 限制输出最大值
        }
        else if (Out < -OUT_LIMIT)
        {
            Out = -OUT_LIMIT; // 限制输出最小值
        }
        if (Out < 0.0)
        {
            Out += 0xFFFF;
        }
    }
    uint8_t SendData[8] = {0};
    SendData[6] = ((uint16_t)Out >> 8);
    SendData[7] = ((uint16_t)Out & 0xFF);

    Can_SendData(&hcan, 0x1FE, SendData, 8);
}
