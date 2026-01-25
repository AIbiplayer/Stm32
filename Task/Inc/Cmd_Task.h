/**
* @file Cmd_Task.h
 * @brief 底盘控制任务
 * @date 2025/11/28
 */

#ifndef CMD_TASK_H
#define CMD_TASK_H

#include "main.h"

#define UART_CAMERA huart5// 摄像头串口
#define UART_BLUETOOTH huart2 // 蓝牙串口
#define UART_DEBUG huart1// 调试串口

// 控制模式
typedef enum
{
    BLUETOOTH_MODE = 0, // 蓝牙模式
    PS2_MODE, // 红外按键模式
} Control_Mode_e;

// 视觉模式
typedef enum
{
    NONE_VISION = 0, // 无模式
    FACE_VISION, // 人脸识别模式
    TRAIL_VISION, // 寻迹模式
    LASER_VISION // 激光打靶模式
} Vision_Mode_e;

// 摄像头数据结构体
typedef struct
{
    Vision_Mode_e Mode : 4; // 视觉模式
    uint8_t Target_Found : 4; // 是否找到目标，1-已找到，0-未找到
    int8_t Error_X; // 误差X,在不同模式中代表不同的数据
    int8_t Error_Y; // 误差Y,在不同模式中代表不同的数据
} Camera_Data_s;

void Uart_Init(void);
void Cmd_Task(void);
void OLED_SHOW(void);
void Key_Setting(void);

#endif //CMD_TASK_H
