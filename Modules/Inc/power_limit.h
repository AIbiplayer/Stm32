//
// Created by lenovo on 25-11-6.
//

#ifndef POWER_LIMIT_H
#define POWER_LIMIT_H

#include "DJI_Motor.h"

/**
 * @brief 根据裁判功率上限与缓冲能量限制已注册3508电机的输出
 */
void power_limit(void);

/**
 * @brief 计算3508电机功率
 * @param output_current 经PID计算后即将发送的电机输出值（DJI电流指令）
 * @param speed_rpm 电机当前转速，单位rpm
 * @return 电机输出功率估计值
 */
float power_calculate(float output_current, float speed_rpm);

/**
 * @brief 设置最大电功率
 * @param _power_max 底盘最大功率
 */
void SetPowerMax(float power_max);

/**
 * @brief 设置功率限制里面的电机实例
 * @param motor_instance 电机结构体的指针数组（成员需要按照ID顺序存入）
 */
// void SetMotorInstance(DJIMotorInstance *motor_instance[]);

/**
 * @brief 将需要功率控制的电机注册到功率控制模块
 * @param motor 需要功率控制的电机实例
 */
void PLMotor_Register(DJI_Motor_Instance *motor);
#endif //POWER_LIMIT_H
