/**
* @file TMC.h
 * @brief 双板驱动具体控制接口
 * @author Shen FeiLin
 * @date 2025/12/30
 */

#ifndef TMC_H
#define TMC_H

#include "robot_def.h"

#define MCU_GIMBAL   // 分别定义云台板和底盘板
// #define MCU_CHASSIS   // 分别定义云台板和底盘板

#define TMC_CHASSIS_CAN_ID 0x114 // 底盘板TMC的CAN ID
#define TMC_GIMBAL_CAN_ID 0x115 // 云台板TMC的CAN ID

#ifdef MCU_CHASSIS // 如果是底盘板

#endif

#ifdef MCU_GIMBAL // 如果是云台板

#endif

#pragma pack(1)
typedef struct {
    Chassis_Ctrl_Cmd_s Chassis_Cmd; ///< 底盘控制命令
    uint8_t gimbal_mode: 2; ///< 云台模式,供底盘判断是否需要进行跟随等控制
    uint8_t UI_reset: 1; ///< UI重置标志,供底盘判断是否需要进行UI重置等控制
    uint8_t friction_mode: 1; ///< 摩擦轮模式,供底盘判断是否需要进行跟随等控制
    uint8_t loader_mode: 2; ///< 拨盘模式,供底盘判断是否需要进行跟随等控制
    uint8_t distance_status: 1; ///< 距离状态,供底盘判断是否允许追击
    uint8_t isfind_enemy: 1; ///< 视觉是否找到目标
    uint8_t superpower_flag: 1; ///< 是否开启超级电容
    uint8_t reset_flag: 1; ///< 是否复位，1为复位，0为不复位
    uint8_t shoot_speed: 5; ///< 发射速度,供底盘判断是否需要进行跟随等控制,单位为m/s
} TMC_To_Chassis_s; ///< 从云台发送到底盘的控制数据结构体

typedef struct {
    Shoot_Upload_Data_s Shoot_Upload_Data; ///< 发射反馈数据
} TMC_To_Gimbal_s; ///< 从底盘发送到云台的控制数据结构体
#pragma pack()

#endif //TMC_H
