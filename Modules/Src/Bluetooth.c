/**
 * @file Bluetooth.c
 * @brief 蓝牙模块驱动
 * @date 2025/10/8
 */

#include "Bluetooth.h"
#include "usart.h"
#include "string.h"
#include "stdbool.h"
#include "Debug_Tool.h"

#define FRAME_ROCKER  0x31  // 摇杆模式
#define FRAME_HANDLE  0x32  // 手柄模式
#define FRAME_GRAVITY 0x33  // 重力模式

char Bluetooth_Receive_Buffer[2][RX_BUFF_SIZE] = {0}; // 蓝牙双缓冲区
char* Buffer_Ptr = Bluetooth_Receive_Buffer[0]; // 指向当前处理的缓冲区
CCMRAM_DATA Bluetooth_Data_s Instance = {0}; // 蓝牙数据实例

static bool Toggle = false; // 切换双缓冲区标志

/**
 * @brief 解析蓝牙接收的数据
 * @note 对三种模式分别进行解析
 */
bool Bluetooth_Parse(const uint8_t len)
{
    static uint8_t buf[RX_BUFF_SIZE] = {0}; // 临时存储接收数据
    memcpy(buf, Buffer_Ptr, RX_BUFF_SIZE);

    if (buf[0] != 0xAA || buf[1] != 0x55) // 数据包头错误
        return false;

    uint8_t checksum = 0;
    for (uint8_t i = 0; i < len - 1; i++) // 计算校验和，最后一个字节为校验和
        checksum += buf[i];
    if (checksum != buf[len - 1])
        return false;

    Instance.X_L = buf[5]; //@note 重力模式会重新覆盖
    Instance.Y_L = -buf[4];

    switch (buf[3]) //不同模式分开接收
    {
    case FRAME_ROCKER:
        {
            if (len != 10) break; // 摇杆模式固定长度10字节
            Instance.Mode = MODE_ROCKER;
            Instance.Rocker_Handle_Data.X_R = buf[7];
            Instance.Rocker_Handle_Data.Y_R = -buf[6];
            // 位运算映射按键（低字节到高字节）
            Instance.Rocker_Handle_Data.L1 = (buf[8] >> 7) & 1;
            Instance.Rocker_Handle_Data.L2 = (buf[8] >> 6) & 1;
            Instance.Rocker_Handle_Data.R1 = (buf[8] >> 5) & 1;
            Instance.Rocker_Handle_Data.R2 = (buf[8] >> 4) & 1;
            Instance.Rocker_Handle_Data.K1 = (buf[8] >> 3) & 1;
            Instance.Rocker_Handle_Data.K2 = (buf[8] >> 2) & 1;
            Instance.Rocker_Handle_Data.K3 = (buf[8] >> 1) & 1;
            Instance.Rocker_Handle_Data.K4 = (buf[8] >> 0) & 1;
            return true;
        }
    case FRAME_HANDLE:
        {
            if (len != 11) break; // 手柄模式固定长度11字节
            Instance.Mode = MODE_HANDLE;
            Instance.Rocker_Handle_Data.X_R = buf[7];
            Instance.Rocker_Handle_Data.Y_R = -buf[6];
            //按键1
            Instance.Dif_Data.Handle_Data.Up = (buf[8] >> 7) & 1;
            Instance.Dif_Data.Handle_Data.Back = (buf[8] >> 6) & 1;
            Instance.Dif_Data.Handle_Data.Left = (buf[8] >> 5) & 1;
            Instance.Dif_Data.Handle_Data.Right = (buf[8] >> 4) & 1;
            Instance.Dif_Data.Handle_Data.A = (buf[8] >> 3) & 1;
            Instance.Dif_Data.Handle_Data.B = (buf[8] >> 2) & 1;
            Instance.Dif_Data.Handle_Data.X = (buf[8] >> 1) & 1;
            Instance.Dif_Data.Handle_Data.Y = (buf[8] >> 0) & 1;
            //按键2
            Instance.Rocker_Handle_Data.L1 = (buf[9] >> 7) & 1;
            Instance.Rocker_Handle_Data.L2 = (buf[9] >> 6) & 1;
            Instance.Rocker_Handle_Data.R1 = (buf[9] >> 5) & 1;
            Instance.Rocker_Handle_Data.R2 = (buf[9] >> 4) & 1;
            Instance.Rocker_Handle_Data.K1 = (buf[9] >> 3) & 1;
            Instance.Rocker_Handle_Data.K2 = (buf[9] >> 2) & 1;
            Instance.Rocker_Handle_Data.K3 = (buf[9] >> 1) & 1;
            Instance.Rocker_Handle_Data.K4 = (buf[9] >> 0) & 1;
            return true;
        }
    case FRAME_GRAVITY:
        {
            if (len != 8) break; // 重力模式固定长度8字节
            Instance.Mode = MODE_GRAVITY;
            Instance.X_L = -buf[6];
            Instance.Y_L = -buf[5];
            Instance.Dif_Data.Gravity_Data.Last_G_Yaw = Instance.Dif_Data.Gravity_Data.G_Yaw;
            Instance.Dif_Data.Gravity_Data.G_Yaw = buf[4];
            return true;
        }
    default: break;
    }
    return false; // 帧号不匹配或长度错误
}

/**
 * @brief UART的DMA接收
 * @param huart UART句柄
 * @param Size 接收数据长度
 * @note 使用了双缓冲区技术，可能会出现错误
 * @todo 此函数应放在任务模块中
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size) // 串口接收回调函数
{
    if (huart->Instance == USART2)
    {
        HAL_UART_DMAStop(&huart2);

        Bluetooth_Parse(Size);
        memset(Buffer_Ptr, 0, sizeof(Bluetooth_Receive_Buffer[0])); // 清空接收缓冲区

        Buffer_Ptr = Toggle ? Bluetooth_Receive_Buffer[0] : Bluetooth_Receive_Buffer[1];
        Toggle = !Toggle; // 切换缓冲区

        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t*)Buffer_Ptr, sizeof(Bluetooth_Receive_Buffer[0]));
        __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT); // 禁用DMA半传输中断，避免进入两次回调
    }
}

/**
 * @brief 串口初始化
 * @todo 蓝牙启动时底盘速度归零
 */
void Bluetooth_Start(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t*)Buffer_Ptr, sizeof(Bluetooth_Receive_Buffer[0]));
    __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
}

/**
 * @brief 停止蓝牙模块
 * @note 该函数停止UART的DMA接收和中断
 * @todo 蓝牙停止时自动切换至PS2模式，同时底盘速度归零
 */
void Bluetooth_Stop(void)
{
    HAL_UART_DMAStop(&huart2);
    __HAL_UART_DISABLE_IT(&huart2, UART_IT_IDLE);
}
