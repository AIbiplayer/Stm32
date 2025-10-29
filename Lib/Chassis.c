/**
* @file Chassis.c
 * @brief 底盘控制模块
 * @date 2025/10/4
 */

#include "Chassis.h"
#include "math.h"
#include "Key.h"
#include "usart.h"
#include "BlueTooth.h"

#define PI 3.14159265

Chassis Chassis_Control = {
    1,
    CHASSIS_STOP,
    {0, 0, 0},
    {0, 0, 0, 0}
}; // 底盘控制结构体

uint8_t Chassis_Index = 1; // 底盘索引,0为三轮,1为四轮
int8_t Chassis_Speed[3] = {0}; // x,y,w

extern int8_t Receive_Target_Speed[3];
extern Control_Mode Chassis_Mode;

/**
 * @brief 底盘行为函数
 * @note 循环调用
 */
void Chassis_Behavior(void)
{
    if (Chassis_Mode == BLUETOOTH_MODE) //蓝牙模式接收速度
    {
        Chassis_Control.Move.x = Receive_Target_Speed[0];
        Chassis_Control.Move.y = Receive_Target_Speed[1];
        Chassis_Control.Move.w = Receive_Target_Speed[2];
    }

    switch (Chassis_Index) //底盘模式
    {
    case 0: // 三轮模式
        if (Chassis_Control.Status == CHASSIS_STOP)
        {
            Chassis_Control.Speed_Set[0] = 0;
            Chassis_Control.Speed_Set[1] = 0;
            Chassis_Control.Speed_Set[2] = 0;
        }
        else if (Chassis_Control.Status == CHASSIS_SPIN)
        {
            Chassis_Control.Move.w = 30;
            Chassis_Control.Speed_Set[0] = Chassis_Control.Speed_Multiple * Chassis_Control.Move.w;
            Chassis_Control.Speed_Set[1] = Chassis_Control.Speed_Multiple * Chassis_Control.Move.w;
            Chassis_Control.Speed_Set[2] = Chassis_Control.Speed_Multiple * Chassis_Control.Move.w;
        }
        else if (Chassis_Control.Status == CHASSIS_NORAML)
        {
            Chassis_Control.Speed_Set[0] = Chassis_Control.Speed_Multiple
                * (Chassis_Control.Move.y
                    + Chassis_Control.Move.w);

            Chassis_Control.Speed_Set[1] = Chassis_Control.Speed_Multiple
                * (sin(60 * PI / 180) * Chassis_Control.Move.x
                    - cos(60 * PI / 180) * Chassis_Control.Move.y
                    + Chassis_Control.Move.w);

            Chassis_Control.Speed_Set[2] = Chassis_Control.Speed_Multiple
                * (-sin(60 * PI / 180) * Chassis_Control.Move.x
                    - cos(60 * PI / 180) * Chassis_Control.Move.y
                    + Chassis_Control.Move.w);
        }
        Uart_printf(&huart2, "%d,%d,%d,3\n",
                    Chassis_Control.Speed_Set[0],
                    Chassis_Control.Speed_Set[1],
                    Chassis_Control.Speed_Set[2]);
        break;
    case 1: //四轮模式
        if (Chassis_Control.Status == CHASSIS_STOP)
        {
            Chassis_Control.Speed_Set[0] = 0;
            Chassis_Control.Speed_Set[1] = 0;
            Chassis_Control.Speed_Set[2] = 0;
            Chassis_Control.Speed_Set[3] = 0;
        }
        else if (Chassis_Control.Status == CHASSIS_SPIN)
        {
            Chassis_Control.Move.w = 30;
            Chassis_Control.Speed_Set[0] = Chassis_Control.Speed_Multiple * Chassis_Control.Move.w;
            Chassis_Control.Speed_Set[1] = Chassis_Control.Speed_Multiple * Chassis_Control.Move.w;
            Chassis_Control.Speed_Set[2] = Chassis_Control.Speed_Multiple * Chassis_Control.Move.w;
            Chassis_Control.Speed_Set[3] = Chassis_Control.Speed_Multiple * Chassis_Control.Move.w;
        }
        else if (Chassis_Control.Status == CHASSIS_NORAML)
        {
            Chassis_Control.Speed_Set[0] = Chassis_Control.Speed_Multiple
                * (Chassis_Control.Move.x
                    - Chassis_Control.Move.y
                    + Chassis_Control.Move.w);

            Chassis_Control.Speed_Set[1] = Chassis_Control.Speed_Multiple
                * (-Chassis_Control.Move.x
                    - Chassis_Control.Move.y
                    + Chassis_Control.Move.w);

            Chassis_Control.Speed_Set[2] = Chassis_Control.Speed_Multiple
                * (Chassis_Control.Move.x
                    + Chassis_Control.Move.y
                    + Chassis_Control.Move.w);

            Chassis_Control.Speed_Set[3] = Chassis_Control.Speed_Multiple
                * (-Chassis_Control.Move.x
                    + Chassis_Control.Move.y
                    + Chassis_Control.Move.w);
        }
        Uart_printf(&huart2, "%d,%d,%d,%d,4\n",
                    Chassis_Control.Speed_Set[0],
                    Chassis_Control.Speed_Set[1],
                    Chassis_Control.Speed_Set[2],
                    Chassis_Control.Speed_Set[3]);
        break;
    default:
        Chassis_Control.Status = CHASSIS_STOP;
        break;
    }
}
