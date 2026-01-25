/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2025-12-23 15:17:29
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2025-12-23 16:07:29
 * @FilePath: \MDK-ARMf:\Desktop\MotorDrivers_F407\MotorDrivers_F407\Modules\Src\Gimbal_Trace.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "Gimbal_Trace.h"
#include "PID.h"
#include "stm32f4xx_hal.h"
#include <string.h>
#include "../../Motor/Inc/Servo.h"

extern float pid_output;
extern float current_angle;
extern float target_angle;
extern UART_HandleTypeDef huart3;
PID_Typedef yaw_pid;
PID_Typedef pitch_pid;

#define TRACE_BUF_SIZE 70
#define HALF_IMG_WIDTH 320   // 图像宽度的一半
#define HALF_IMG_HEIGHT 240  // 图像高度的一半

trace_t trace;

/**
 * @brief 距离转角度
 * @param trace_ 跟踪数据
 */
void distance_to_angle(trace_t *trace_)
{
    trace_->x_angle = (float)trace_->x_distence / HALF_IMG_WIDTH * 90.0f;// 计算x轴角度
    trace_->y_angle = (float)trace_->y_distence / HALF_IMG_HEIGHT * 135.0f;// 计算y轴角度
}

/** 
 * @brief 自动跟踪
 * @param channel 跟踪通道
 */
void auto_tracing(uint8_t channel)
{
    switch (channel)
    {
        case YAW:
            /* code */
            distance_to_angle(&trace);// 距离转角度
            current_angle = Servo_GetAngle(YAW);
            target_angle = trace.x_angle;
            Servo_SetAngle(YAW, yaw_pid.Output);
            break;
        case PITCH:
            /* code */
            distance_to_angle(&trace);// 距离转角度
            current_angle = Servo_GetAngle(PITCH);
            target_angle = trace.y_angle;
            Servo_SetAngle(PITCH, pitch_pid.Output);
            break;
        default:
            break;
    }
}
