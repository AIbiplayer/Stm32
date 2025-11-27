/**
 * @file Bluetooth.h
 * @brief 蓝牙数据处理头文件
 * @date 2025/10/4
 */

#ifndef INC_BLUETOOTH_H_
#define INC_BLUETOOTH_H_

#include "main.h"
#include "stdbool.h"

#define RX_BUFF_SIZE 64 // 定义接收缓冲区大小

typedef enum
{
    MODE_NONE = 0, // 无模式
    MODE_ROCKER, // 摇杆模式
    MODE_HANDLE, // 手柄模式
    MODE_GRAVITY // 重力感应模式
} Bluetooth_Mode_e; // 蓝牙模式

/**
 * @brief 手柄模式数据结构体
 * @note 这是在手柄和摇杆共有的按键基础上，添加手柄特有的数据
 */
typedef struct
{
    uint8_t Up : 1; // 上按键
    uint8_t Back : 1; // 下按键
    uint8_t Left : 1; // 左按键
    uint8_t Right : 1; // 右按键
    uint8_t Y : 1; // Y按键
    uint8_t X : 1; // X按键
    uint8_t A : 1; // A按键
    uint8_t B : 1; // B按键
} Handle_Data_s;

/**
 * @brief 摇杆/手柄共有数据结构体
 * @note 这是手柄和摇杆共有的按键
 */
typedef struct
{
    uint8_t L1 : 1; // L1按键
    uint8_t L2 : 1; // L2按键
    uint8_t R1 : 1; // R1按键
    uint8_t R2 : 1; // R2按键
    uint8_t K1 : 1; // K1按键
    uint8_t K2 : 1; // K2按键
    uint8_t K3 : 1; // K3按键
    uint8_t K4 : 1; // K4按键
    int8_t X_R; //右摇杆X轴数据
    int8_t Y_R; //右摇杆Y轴数据
} Rocker_Handle_Data_s;

/**
 * @brief 重力感应模式数据结构体
 * @note  计算单位时间YAW的差值以求得角速度
 */
typedef struct
{
    int8_t G_Yaw; // 航向角数据
    int8_t Last_G_Yaw; // 上一次航向角数据
} Gravity_Data_s;

/**
 * @brief 不同模式数据联合体
 * @note 这里包含了不同模式特有的数据
 */
typedef union
{
    Handle_Data_s Handle_Data; // 手柄模式特有按键数据
    Gravity_Data_s Gravity_Data; // 重力感应模式特有数据
} Dif_Data_u;

/**
 * @brief 蓝牙数据总结构体
 */
typedef struct
{
    Bluetooth_Mode_e Mode; // 当前模式

    int8_t X_L; // X方向数据（前进为正方向）
    int8_t Y_L; // Y方向数据（左边为正方向）
    Dif_Data_u Dif_Data; // 不同模式特有的数据
    Rocker_Handle_Data_s Rocker_Handle_Data; // 摇杆/手柄共有数据
} Bluetooth_Data_s;

void Bluetooth_Stop(void);
void Bluetooth_Start(void);
bool Bluetooth_Parse(const uint8_t len);

#endif /* INC_Bluetooth_H_ */
