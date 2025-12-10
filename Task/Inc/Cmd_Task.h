/**
* @file Cmd_Task.h
 * @brief 底盘控制任务
 * @date 2025/11/28
 */

#ifndef CMD_TASK_H
#define CMD_TASK_H

#define UART_VISION huart5// 摄像头串口
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
    TRAIL_VISION, // 寻迹模式
    FACE_VISION, // 人脸识别模式
    LASER_VISION // 激光打靶模式
} Vision_Mode_e;

void Cmd_Task(void);
void OLED_SHOW(void);
void Key_Setting(void);

#endif //CMD_TASK_H
