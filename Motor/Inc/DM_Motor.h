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

typedef enum {
    DM_DISABLE = 0, ///< 电机失能
    DM_ENABLE, ///< 电机使能
    DM_OVERPRESS = 8, ///< 过压
    DM_UNDERVOLTAGE, ///< 欠压
    DM_OVERCURRENT, ///< 过流
    DM_MOS, ///< MOS管过热
    DM_OVERHEAT, ///< 线圈过热
    DM_LOSE, ///< 失控
    DM_OVERBURDEN ///< 过载
} DM_error_e; ///< 达妙电机错误标志

typedef struct {
    float Gyro[3]; ///< 角速度，单位°/s
    float Roll; ///< 滚转角，单位°
    float Pitch; ///< 俯仰角，单位°
    float Yaw; ///< 偏航角，单位°
} DM_IMU_Measure_s; ///< 达妙IMU

typedef struct {
    DM_error_e error; ///< 电机错误标志
    uint16_t ecd; ///< 角度值，范围0~8192
    float angle; ///< 角度，单位°
    float speed; ///< 速度，单位rpm
    float current; ///< 电流值，单位mA
    float total_angle; ///< 累计角度，单位°
} DM_Motor_Measure_s; ///< 达妙电机测量值

typedef struct {
    DM_Motor_Measure_s Measure; ///< 达妙电机测量值
    Motor_Control_Setting_s Control_Setting;
    CANInstance *Motor_Can_Instance;
    uint8_t id; ///< 电机ID
} DM_Motor_Instance; ///< 达妙电机实例

typedef struct {
    DM_IMU_Measure_s Measure;
    CAN_Init_Config_s Can_Init_Config;
    CANInstance *IMU_Can_Instance;
} DM_IMU_Instance_s; // 达妙IMU实例

DM_Motor_Instance *DM_Motor_Init(DM_Motor_Init_s *Motor_Init);

void DM_Motor_Control(void);

void Decode_dm_imu(CANInstance *Instance);

void DM_motor_offline(void *owner_id);

void DM_MotorEnable(void);

void DM_MotorStop(void);

void DM_MotorChangeReverse(DM_Motor_Instance *motor, const Motor_Reverse_Flag_e motor_reverse_flag);

void DM_MotorSet(DM_Motor_Instance *motor, float Target_);

void DM_MotorSaveZero(DM_Motor_Instance *motor);

void DM_MotorClearError(DM_Motor_Instance *motor);

#endif //DM_MOTOR_H
