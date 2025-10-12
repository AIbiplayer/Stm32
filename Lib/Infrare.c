/**
*  @file Infrare.c
 * @brief 红外接收驱动代码及按键扫描
 * @date 2025/9/30
 */

#include <string.h>
#include "main.h"
#include "stdbool.h"
#include  "Chassis.h"
#include  "ShowShape.h"
#include "Key.h"

uint8_t Inf_Interval = 0; // 红外间隔时间
uint8_t Inf_Data[4] = {0}; // 红外数据
uint8_t Inf_Byte[33] = {0}; // 红外字节原始数据
uint8_t Inf_Index = 0; // 红外数据索引
bool Inf_StartFlag = false; // 红外开始标志
bool Inf_ReceiveFlag = false; // 红外接收完成标志

extern Control_Mode Chassis_Mode;
extern Chassis Chassis_Control;
extern uint8_t Chassis_Index;

/**
 * @brief 外部中断3服务函数
 * @note 红外接收中断，接收时闪烁LED2
 */
void EXTI3_IRQHandler(void)
{
    __HAL_GPIO_EXTI_CLEAR_IT(Infrare_Pin);
    HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);

    if (Inf_StartFlag)
    {
        if (Inf_Interval < 150 && Inf_Interval > 50) // 0.56ms
            Inf_Index = 0;

        Inf_Byte[Inf_Index] = Inf_Interval;
        Inf_Interval = 0;
        Inf_Index++;

        if (Inf_Index == 33) // 接收完成
        {
            Inf_Index = 0;
            Inf_Interval = 0;
            Inf_ReceiveFlag = true;
        }
    }
    else
    {
        Inf_Index = 0;
        Inf_Interval = 0;
        Inf_StartFlag = true;
    }

    HAL_GPIO_EXTI_IRQHandler(Infrare_Pin);
}

/**
 * @brief 红外接收解析函数
 */
void Inf_Server(void)
{
    uint8_t i, j, idx = 1; //idx 从1 开始表示对同步头的时间不处理
    uint8_t temp;
    for (i = 0; i < 4; i++)
    {
        for (j = 0; j < 8; j++)
        {
            if (Inf_Byte[idx] >= 8 && Inf_Byte[idx] < 15) //表示 0
            {
                temp = 0;
            }
            else if (Inf_Byte[idx] >= 18 && Inf_Byte[idx] < 25) //表示 1
            {
                temp = 1;
            }
            Inf_Data[i] <<= 1;
            Inf_Data[i] |= temp;
            idx++;
        }
    }

    if (Chassis_Mode != INFRARE_MODE) // 非红外模式不处理
        return;
    switch (Inf_Data[2]) // 按键值
    {
    case 162: // 1,速度倍率1
        Chassis_Control.Speed_Multiple = 1;
        break;
    case 98: // 2，速度倍率2
        Chassis_Control.Speed_Multiple = 2;
        break;
    case 226: // 3，速度倍率3
        Chassis_Control.Speed_Multiple = 3;
        break;
    case 34: // 4，云台画sin
        ShowSin();
        break;
    case 2: // 5,云台画圆
        ShowCircal();
        break;
    case 194: // 6，云台画三角
        ShowTriangle();
        break;
    case 224: // 7，云台停止
        Yaw_SetAngle(90.0f); // 设置偏航舵机初始位置
        Pitch_SetAngle(90.0f); // 设置俯仰舵机初始位置
        break;
    case 168: // 8，底盘为四轮模式
        Chassis_Index = 1;
        break;
    case 144: // 9，底盘为三轮模式
        Chassis_Index = 0;
        break;
    case 104: // *，底盘旋转
        Chassis_Control.Status = CHASSIS_SPIN;
        break;
    case 176: // #，底盘正常运动
        break;
    case 24: // 上，前进
        Chassis_Control.Status = CHASSIS_NORAML;
        Chassis_Control.Move.x = 30;
        Chassis_Control.Move.y = 0;
        Chassis_Control.Move.w = 0;
        break;
    case 74: // 下，后退
        Chassis_Control.Status = CHASSIS_NORAML;
        Chassis_Control.Move.x = -30;
        Chassis_Control.Move.y = 0;
        Chassis_Control.Move.w = 0;
        break;
    case 16: // 左，左移
        Chassis_Control.Status = CHASSIS_NORAML;
        Chassis_Control.Move.x = 0;
        Chassis_Control.Move.y = 30;
        Chassis_Control.Move.w = 0;
        break;
    case 90: // 右，右移
        Chassis_Control.Status = CHASSIS_NORAML;
        Chassis_Control.Move.x = 0;
        Chassis_Control.Move.y = -30;
        Chassis_Control.Move.w = 0;
        break;
    case 56: // 中间，停止
        Chassis_Control.Status = CHASSIS_STOP;
        break;
    default:
        Chassis_Control.Status = CHASSIS_STOP;
        break;
    }
}

/**
 * @brief 红外接收停止函数
 */
void Inf_Stop(void)
{
    HAL_NVIC_DisableIRQ(EXTI3_IRQn);
    __HAL_GPIO_EXTI_CLEAR_IT(Infrare_Pin);
    HAL_NVIC_ClearPendingIRQ(EXTI3_IRQn);
}

/**
 * @brief 红外接收启动函数
 */
void Inf_Start(void)
{
    HAL_NVIC_EnableIRQ(EXTI3_IRQn);
    __HAL_GPIO_EXTI_CLEAR_IT(Infrare_Pin);
    HAL_NVIC_ClearPendingIRQ(EXTI3_IRQn);
}
