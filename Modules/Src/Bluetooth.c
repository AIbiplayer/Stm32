/**
 * @file Bluetooth.c
 * @brief 蓝牙模块驱动
 * @date 2025/10/8
 */

#include "Bluetooth.h"
#include "Cmd_Task.h"
#include "usart.h"
#include "string.h"
#include "Chassis_Task.h"
#include "Debug_Tool.h"

#define FRAME_ROCKER  0x31  // 摇杆模式
#define FRAME_HANDLE  0x32  // 手柄模式
#define FRAME_GRAVITY 0x33  // 重力模式

char Bluetooth_Receive_Buffer[2][RX_BUFF_SIZE] = {0}; // 蓝牙双缓冲区
char* Buffer_Ptr = Bluetooth_Receive_Buffer[0]; // 指向当前处理的缓冲区
CCMRAM_DATA Bluetooth_Data_s BL_Instance = {0}; // 蓝牙数据实例

extern Chassis_Instance_s CH_Instance;
extern Camera_Data_s Cam_Instance;

/**
 * @brief 解析蓝牙接收的数据
 * @note 对三种模式分别进行解析
 * @todo 返回的状态或许可以用上
 */
void Bluetooth_Parse(const uint8_t len)
{
    static uint8_t buf[RX_BUFF_SIZE] = {0}; // 临时存储接收数据
    memcpy(buf, Buffer_Ptr, RX_BUFF_SIZE);

    if (buf[0] != 0xAA || buf[1] != 0x55) // 数据包头错误
        return;

    uint8_t checksum = 0;
    for (uint8_t i = 0; i < len - 1; i++) // 计算校验和，最后一个字节为校验和
        checksum += buf[i];
    if (checksum != buf[len - 1])
        return;

    BL_Instance.X_L = buf[5]; //@note 重力模式会重新覆盖
    BL_Instance.Y_L = -buf[4];

    switch (buf[3]) //不同模式分开接收
    {
    case FRAME_ROCKER:
        {
            if (len != 10) break; // 摇杆模式固定长度10字节
            BL_Instance.Mode = MODE_ROCKER;
            BL_Instance.Rocker_Handle_Data.X_R = buf[7];
            BL_Instance.Rocker_Handle_Data.Y_R = -buf[6];
            // 位运算映射按键（低字节到高字节）
            BL_Instance.Rocker_Handle_Data.L1 = (buf[8] >> 7) & 1;
            BL_Instance.Rocker_Handle_Data.L2 = (buf[8] >> 6) & 1;
            BL_Instance.Rocker_Handle_Data.R1 = (buf[8] >> 5) & 1;
            BL_Instance.Rocker_Handle_Data.R2 = (buf[8] >> 4) & 1;
            BL_Instance.Rocker_Handle_Data.K1 = (buf[8] >> 3) & 1;
            BL_Instance.Rocker_Handle_Data.K2 = (buf[8] >> 2) & 1;
            BL_Instance.Rocker_Handle_Data.K3 = (buf[8] >> 1) & 1;
            BL_Instance.Rocker_Handle_Data.K4 = (buf[8] >> 0) & 1;
            break;
        }
    case FRAME_HANDLE:
        {
            if (len != 11) break; // 手柄模式固定长度11字节
            BL_Instance.Mode = MODE_HANDLE;
            BL_Instance.Rocker_Handle_Data.X_R = buf[7];
            BL_Instance.Rocker_Handle_Data.Y_R = -buf[6];
            BL_Instance.Dif_Data.Handle_Data.Y = (buf[8] >> 0) & 1;
            Cam_Instance.Mode = BL_Instance.Dif_Data.Handle_Data.Y && !BL_Instance.Dif_Data.Handle_Data.Y_Last
                                    ? (Cam_Instance.Mode + 1) % 4
                                    : Cam_Instance.Mode;
            BL_Instance.Dif_Data.Handle_Data.Y_Last = BL_Instance.Dif_Data.Handle_Data.Y;
            //按键1
            BL_Instance.Dif_Data.Handle_Data.Up = (buf[8] >> 7) & 1;
            BL_Instance.Dif_Data.Handle_Data.Back = (buf[8] >> 6) & 1;
            BL_Instance.Dif_Data.Handle_Data.Left = (buf[8] >> 5) & 1;
            BL_Instance.Dif_Data.Handle_Data.Right = (buf[8] >> 4) & 1;
            BL_Instance.Dif_Data.Handle_Data.A = (buf[8] >> 3) & 1;
            BL_Instance.Dif_Data.Handle_Data.B = (buf[8] >> 2) & 1;
            BL_Instance.Dif_Data.Handle_Data.X = (buf[8] >> 1) & 1;
            //按键2
            BL_Instance.Rocker_Handle_Data.L1 = (buf[9] >> 7) & 1;
            BL_Instance.Rocker_Handle_Data.L2 = (buf[9] >> 6) & 1;
            BL_Instance.Rocker_Handle_Data.R1 = (buf[9] >> 5) & 1;
            BL_Instance.Rocker_Handle_Data.R2 = (buf[9] >> 4) & 1;
            BL_Instance.Rocker_Handle_Data.K1 = (buf[9] >> 3) & 1;
            BL_Instance.Rocker_Handle_Data.K2 = (buf[9] >> 2) & 1;
            BL_Instance.Rocker_Handle_Data.K3 = (buf[9] >> 1) & 1;
            BL_Instance.Rocker_Handle_Data.K4 = (buf[9] >> 0) & 1;
            break;
        }
    case FRAME_GRAVITY:
        {
            if (len != 8) break; // 重力模式固定长度8字节
            BL_Instance.Mode = MODE_GRAVITY;
            BL_Instance.X_L = -buf[6];
            BL_Instance.Y_L = -buf[5];
            BL_Instance.Dif_Data.Gravity_Data.G_Yaw_Speed = BL_Instance.Dif_Data.Gravity_Data.G_Yaw
                - BL_Instance.Dif_Data.Gravity_Data.Last_G_Yaw;
            BL_Instance.Dif_Data.Gravity_Data.Last_G_Yaw = BL_Instance.Dif_Data.Gravity_Data.G_Yaw;
            BL_Instance.Dif_Data.Gravity_Data.G_Yaw = buf[4];
            break;
        }
    default: break;
    }
    memset(Buffer_Ptr, 0, sizeof(Bluetooth_Receive_Buffer[0]));
}



