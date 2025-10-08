/**
 * @file Key.h
 * @brief 按键处理头文件
 * @date 2025/10/4
 */

#ifndef __KEY_H__
#define __KEY_H__

#include "main.h"
#include "stdbool.h"

// 主控板按键
typedef struct
{
    bool Key_PutStatus; // 按键状态
    bool Key_OnceFlag;  // 按键单次标志
    uint8_t Key_Time;   // 按键时间
} KEY;

// 控制模式
typedef enum
{
    BLUETOOTH_MODE = 0, // 蓝牙模式
    INFRARE_MODE,       // 红外按键模式
    NONE_MODE           // 无模式
} Control_Mode;

void Key_Server(void);

#endif // __KEY_H__
