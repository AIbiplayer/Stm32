/**
 * @file Chassis.h
 * @brief 底盘控制头文件
 * @date 2025/10/4
 */

#ifndef INC_CHASSIS_H_
#define INC_CHASSIS_H_

#include "main.h"

typedef enum // 底盘模式
{
    CHASSIS_NORAML = 0,
    CHASSIS_STOP,
    CHASSIS_SPIN
} Chassis_Status;

typedef struct // 移动结构体
{
    int8_t x;
    int8_t y;
    int8_t w;
} Chassis_MoveData;

typedef struct // 底盘结构体
{
    uint8_t Speed_Multiple; // 速度倍率
    Chassis_Status Status;  // 底盘模式
    Chassis_MoveData Move;  // 移动结构体
    int8_t Speed_Set[4];    // 四轮或三轮速度
} Chassis;

void Chassis_Behavior(void);

#endif /* INC_CHASSIS_H_ */
