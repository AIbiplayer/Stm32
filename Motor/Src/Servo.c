/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2025-12-07 19:51:53
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2025-12-23 15:26:28
 * @FilePath: \MDK-ARMe:\keilproject\gimbal_v2.0\Motor\Servo.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "Servo.h"
#include "Cmd_Task.h"
#include "tim.h"
#include "Bluetooth.h"
#include "ax_ps2.h"
#include "stdlib.h"

#define K 25
extern Camera_Data_s Cam_Instance;
extern JOYSTICK_TypeDef JoystickStruct;
extern Bluetooth_Data_s BL_Instance;

static int16_t Y = 500, P = 500;

/**
 * @brief 初始化舵机
 */
void Servo_Init(void) {
    HAL_TIM_PWM_Start(&htim9, PITCH_CHANNEL);
    HAL_TIM_PWM_Start(&htim9, YAW_CHANNEL);

    __HAL_TIM_SetCompare(&htim9, PITCH_CHANNEL, 500);
    __HAL_TIM_SetCompare(&htim9, YAW_CHANNEL, 500);
}

static int16_t Angle_Limit(int16_t angle) {
    if (angle > 2500)
        return 2500;
    else if (angle < 500)
        return 500;
    else
        return angle;
}

/**
 * @brief 视觉模式下自动设置舵机占空比
 */
void Servo_Set_Auto(void) {
    static int16_t P = 1, Y = 1;
    P += abs(Cam_Instance.Error_Y) * Cam_Instance.Error_Y / 100;
    P = Angle_Limit(P);
    Y += abs(Cam_Instance.Error_X) * Cam_Instance.Error_X / 100;
    Y = Angle_Limit(Y);
    __HAL_TIM_SetCompare(&htim9, PITCH_CHANNEL, P);
    __HAL_TIM_SetCompare(&htim9, YAW_CHANNEL, Y);
}

void Servo_Set_Ble(void) {
    if (BL_Instance.Rocker_Handle_Data.L1) {
        Y += K;
        Y = Angle_Limit(Y);
        __HAL_TIM_SetCompare(&htim9, YAW_CHANNEL, Y);
    } else if (BL_Instance.Rocker_Handle_Data.L2) {
        Y -= K;
        Y = Angle_Limit(Y);
        __HAL_TIM_SetCompare(&htim9, YAW_CHANNEL, Y);
    }
    if (BL_Instance.Rocker_Handle_Data.R1) {
        P += K;
        P = Angle_Limit(P);
        __HAL_TIM_SetCompare(&htim9, PITCH_CHANNEL, P);
    } else if (BL_Instance.Rocker_Handle_Data.R2) {
        P -= K;
        P = Angle_Limit(P);
        __HAL_TIM_SetCompare(&htim9, PITCH_CHANNEL, P);
    }
}

void Servo_Set_PS2(void) {
    if (JoystickStruct.L1) {
        Y += K;
        __HAL_TIM_SetCompare(&htim9, YAW_CHANNEL, Y);
    } else if (JoystickStruct.L2) {
        Y -= K;
        __HAL_TIM_SetCompare(&htim9, YAW_CHANNEL, Y);
    }
    if (JoystickStruct.R1) {
        P += K;
        P = Angle_Limit(P);
        __HAL_TIM_SetCompare(&htim9, PITCH_CHANNEL, P);
    } else if (JoystickStruct.R2) {
        P -= K;
        P = Angle_Limit(P);
        __HAL_TIM_SetCompare(&htim9, PITCH_CHANNEL, P);
    }
}
