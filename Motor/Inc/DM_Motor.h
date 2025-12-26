/**
* @file DM_Motor.h
 * @brief 达妙电机驱动模块
 * @author Shen FeiLin
 * @date 2025/12/13
 */

#ifndef DM_MOTOR_H
#define DM_MOTOR_H

#define DM_MOTOR_CNT 10 ///< 达妙电机总数

#include "main.h"
#include "Motor_Def.h"

typedef struct
{
    uint8_t id : 4; ///< 电机ID
    uint8_t state : 4; ///< 电机状态
    float pos; ///< 位置，单位rad
    float vel; ///< 速度，单位rad/s
    float tor; ///< 力矩，单位N/m
    uint8_t Block_CNT; ///< 堵转计数
    uint8_t Block_Flag; ///< 堵转标志
} DM_Motor_Measure_s; ///< 达妙电机测量值

typedef struct
{
    Motor_Working_Type_e Work_Type; ///< 是否使能
    DM_Motor_Measure_s Measure; ///< 达妙电机测量值
    DM_Control_Setting_s Control_Setting;
    CANInstance* Motor_Can_Instance;
} DM_Motor_Instance; ///< 达妙电机实例

DM_Motor_Instance* DM_Motor_Init(DM_Motor_Init_s* Motor_Init);
void DM_Motor_Control(void);
void DM_MotorEnable(DM_Motor_Instance* motor);
void DM_MotorStop(DM_Motor_Instance* motor);
void DM_MotorChangeReverse(DM_Motor_Instance* motor, const Motor_Reverse_Flag_e motor_reverse_flag);
void DM_MotorSet(DM_Motor_Instance* motor, float aTarget_, float vTarget_);
void DM_MotorSaveZero(DM_Motor_Instance* motor);
void DM_MotorClearError(DM_Motor_Instance* motor);

#endif //DM_MOTOR_H
