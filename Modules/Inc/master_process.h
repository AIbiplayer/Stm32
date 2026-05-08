#ifndef MASTER_PROCESS_H
#define MASTER_PROCESS_H

#include "bsp_usart.h"
#include "INS.h"
#include "seasky_protocol.h"

#define VISION_RECV_SIZE 18u // 当前为固定值,36字节
#define VISION_SEND_SIZE 64u // 发送缓冲区最大64字节，可更改,足够大即可

#pragma pack(1)
#pragma pack()

/**
 * @brief 调用此函数初始化和视觉的串口通信
 */
Vision_Recv_s *VisionInit(void);

/**
 * @brief 发送视觉数据
 */
void VisionSend(void);
/**
 * @brief 设置视觉发送标志位
 * @param enemy_color
 * @param work_mode
 * @param bullet_speed
 */
// void VisionSetFlag(Enemy_Color_e enemy_color, Work_Mode_e work_mode, Bullet_Speed_e bullet_speed);

/**
 * @brief 设置发送数据的姿态部分
 */
void VisionSetAltitude(float yaw, float pitch);

/**
 * @brief 设置视觉控制状态
 * @param auto_find 右键进入自瞄时为1,松开为0
 * @param mode 0-普通自瞄,1-小能量机关,2-大能量机关
 */
void VisionSetControlState(uint8_t auto_find, uint8_t mode);

/**
 * @brief 获取视觉通信在线状态
 * @return 1 在线，0 离线
 */
uint8_t VisionIsOnline(void);

/**
 * @brief 根据云台IMU和pitch电机角度更新实时发送数据
 * @param imu 云台IMU数据
 * @param pitch_up_total_angle PITCH_UP电机total_angle,单位deg
 * @param pitch_down_total_angle PITCH_DOWN电机total_angle,单位deg
 */
void VisionUpdateRealtimeData(const INS_t *imu, float pitch_up_total_angle, float pitch_down_total_angle);

/**
 * @brief 根据发射反馈更新非实时发送数据
 * @param heat 当前热量
 * @param shooter_heat_limit 当前热量上限
 * @param robot_color 当前机器人颜色,0红1蓝
 * @param bullet_speed 实时弹速,单位m/s
 */
void VisionUpdateNRTData(uint16_t heat, uint16_t shooter_heat_limit, uint8_t robot_color, float bullet_speed);

#endif // !MASTER_PROCESS_H
