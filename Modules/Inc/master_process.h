#ifndef MASTER_PROCESS_H
#define MASTER_PROCESS_H

#include "bsp_usart.h"
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

#endif // !MASTER_PROCESS_H