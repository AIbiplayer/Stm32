/**
* @file Chassis_Task.h
 * @brief 电机驱动任务
 * @date 2025/11/28
 */

#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "main.h"
#include "Cmd_Task.h"

#define PI 3.14159265

typedef enum // 底盘模式
{
    CHASSIS_MEC = 0, //四轮麦克纳姆轮模式
    CHASSIS_OMNI_TRI, //三轮全向轮模式
    CHASSIS_OMNI_SQU // //四轮全向轮模式
} Chassis_Status_e;

typedef struct // 移动结构体
{
    int8_t x; //Vx
    int8_t y; //Vy
    int8_t w; //Vw
} Chassis_Move_s;

typedef struct // 底盘结构体，这里都是输入
{
    Chassis_Status_e Status : 4; // 底盘模式
    Control_Mode_e Control_Mode : 4; // 控制模式
    Chassis_Move_s Move; // 移动结构体
    int8_t Speed_Set[4]; // 四轮或三轮速度
} Chassis_Instance_s;

void Chassis_Task(void);
void Chassis_Behavior(void);

#endif //CHASSIS_TASK_H
