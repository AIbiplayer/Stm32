/**
 * @file DJI_Motor.h
 * @brief 大疆电机控制
 * @author Shen FeiLin
 * @date 2025/10/24
 */

#ifndef DJI_MOTOR_H
#define DJI_MOTOR_H

#include "Motor_Def.h"

#define ECD_ANGLE_COEF_DJI 0.043945f // (360/8192),将编码器值转化为角度制
#define DJI_MOTOR_CNT 10 // 大疆电机个数

/*滤波系数*/
#define SPEED_SMOOTH_COEF 0.85f      // 最好大于0.85
#define CURRENT_SMOOTH_COEF 0.9f     // 必须大于0.9

#define Gimbal_Pitch_gravity -450  //重力补偿的重心位置
#define Torque_Constant_6020  0.741  //6020转力矩转换为电流的系数
#define Inertia_Moment_6020   133  //6020转动惯量kg.cm2

typedef struct {
    uint16_t Last_Ecd; ///< 上一次角度值，范围0~8192
    uint16_t Ecd; ///< 角度值，范围0~8192
    int16_t Speed; ///< 速度，单位rpm
    float Angle; ///< 角度，范围0~360
    float gravity_compensate; //重力补偿
    float Current; ///< 电流值
    uint8_t Temp; ///< 温度

    int32_t Total_Round; ///< 总圈数
    float Total_Angle; ///< 总角度值

    uint8_t Block_CNT; ///< 堵转计数
    uint8_t Block_Flag; ///< 堵转标志
} DJI_Motor_Measure_s; ///< 大疆电机测量值

/**
 *@note 这里Measure、Motor_App是变量，但Daemon是指针
 *      因为Measure、Motor_App是核心数据，必须以实例保存
 *      而Daemon是保护程序，可以通过指针引用同一个实例，灵活的同时节省内存
 *      这种灵活的结构体通常用于需要频繁更新数据且对内存要求较高的场景，内部通常有void*指针
 */
typedef struct {
    DJI_Motor_Measure_s Measure; ///< 大疆电机测量值
    Motor_Control_Setting_s Control_Setting;
    Motor_Type_e Motor_Type;

    CANInstance *Motor_Can_Instance;

    uint8_t Send_Group;
    uint8_t Message_Num;
} DJI_Motor_Instance; ///< 大疆电机实例

DJI_Motor_Instance *DJI_Motor_Init(Motor_Init_s *Motor_Init);

void DJI_MotorSetTarget(DJI_Motor_Instance *motor, float Target_);

void DJI_MotorEnable(DJI_Motor_Instance *motor);

void DJI_MotorChangeLoop(DJI_Motor_Instance *motor, Motor_Loop_Control_Type_e Loop);

void DJI_MotorChangeReverse(DJI_Motor_Instance *motor, Motor_Reverse_Flag_e motor_reverse_flag);

void DJI_Motor_Control(void);

void DJI_motor_offline(void *owner_id);

float Angle_limit(float angle, float max, float min);

void DJI_MotorStop(DJI_Motor_Instance *motor);

#endif //DJI_MOTOR_H
