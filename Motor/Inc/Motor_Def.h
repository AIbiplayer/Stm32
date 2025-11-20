/**
* @file Motor_Def.h
 * @brief 通用电机配置
 * @author Shen FeiLin
 * @date 2025/10/24
 */

#ifndef MOTOR_DEF_H
#define MOTOR_DEF_H

#include "../../Algorithm/Inc/PID.h"
#include "../../Bsp/Inc/bsp_can.h"

typedef enum
{
    M3508 = 0, ///< M3508电机
    M2006, ///< M2006电机
    GM6020 ///< GM6020电机
} Motor_Type_e; ///< 电机类型

/* 反馈来源设定,若设为OTHER_FEED则需要指定数据来源指针*/
typedef enum
{
    MOTOR_FEEDBACK = 0,
    OTHER_FEEDBACK
} Feedback_Source_e;

// 前馈类型
typedef enum
{
    FEEDFORWARD_NONE = 0,
    SPEED_FEEDFORWARD
} Feedforward_Type_e;

/* 反馈量正反标志 */
typedef enum
{
    FEEDBACK_NORMAL = 0,
    FEEDBACK_REVERSE
} Feedback_Reverse_Flag_e;

//翻转类型
typedef enum
{
    MOTOR_REVERSE = 0, ///< 电机翻转
    MOTOR_NORMAL, ///< 电机正转
} Motor_Reverse_Flag_e; ///< 翻转类型

//控制类型
typedef enum
{
    SPEED_CONTROL = 0, ///<速度环控制
    ANGLE_CONTROL, ///<角度环控制
    SPEED_ANGLE_CONTROL, ///< 速度外、角度内环控制
    ANGLE_SPEED_CONTROL ///<速度内，角度外环控制
} Motor_Loop_Control_Type_e;

typedef enum
{
    MOTOR_ENABLE = 0, ///< 电机启动
    MOTOR_STOP ///< 电机停止
} Motor_Working_Type_e; ///< 电机运动状态

typedef struct
{
    Motor_Loop_Control_Type_e Loop_Control;
    Motor_Reverse_Flag_e Reverse_Flag;

    Feedback_Source_e Angle_Feedback_Source; // 角度反馈类型
    Feedback_Source_e Speed_Feedback_Source; // 速度反馈类型
    float* Other_Angle_Feedback_Ptr; // 角度反馈指针
    float* Other_Speed_Feedback_Ptr; // 速度反馈指针

    Feedforward_Type_e Feedforward_Flag; // 前馈标志
    float* Speed_Feedforward_Ptr; // 速度前馈指针


    PID_Typedef Speed_PID;
    PID_Typedef Angle_PID;

    float Target;
} Motor_Control_Setting_s; ///< 电机控制设置

/**
 *@note 仅为电机基本设置，需要注册不同电机类型
 */
typedef struct
{
    Motor_Control_Setting_s Control_Setting;
    Motor_Working_Type_e Working_Type;
    Motor_Type_e Motor_Type;
    CAN_Init_Config_s Can_Init_Config; ///< CAN模式设置
} Motor_Init_s; ///< 电机初始化

#endif //MOTOR_DEF_H
