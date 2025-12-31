//
// Created by lenovo on 25-11-6.
//

#ifndef POWER_LIMIT_H
#define POWER_LIMIT_H

#include "DJI_Motor.h"

/**
 * @brief 限制功率（未完全完成，还需要从裁判系统读取当前等级）
 */
void power_limit();

/**
 * @brief 计算3508电机功率
 * @param I 经PID计算后即将发送的电机电流值
 * @param speed_rads 电机的当前转速
 * @return 电机输出功率估计值
 */
float power_calculate(float I, float speed_rads);

/**
 * @brief 设置最大电功率
 * @param _power_max 底盘最大功率
 */
void SetPowerMax(float _power_max);

/**
 * @brief 设置功率限制里面的电机实例
 * @param motor_instance 电机结构体的指针数组（成员需要按照ID顺序存入）
 */
// void SetMotorInstance(DJIMotorInstance *motor_instance[]);

/**
 * @brief 将需要功率控制的电机注册到功率控制模块
 * @param motor 需要功率控制的电机实例
 */
void PLMotor_Register(DJI_Motor_Instance* motor);
#endif //POWER_LIMIT_H
