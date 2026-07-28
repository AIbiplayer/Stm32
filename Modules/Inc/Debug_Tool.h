/**
* @file Debug_Tool.c
 * @brief 调试工具模块
 * @date 2025/11/25
 */

#ifndef DEBUG_TOOL_H
#define DEBUG_TOOL_H

#include "main.h"

#define CCMRAM_CODE __attribute__((section(".ccmram_code")))
#define CCMRAM_DATA __attribute__((section(".ccmram_data")))

void Uart_printf(UART_HandleTypeDef* huart, char* format, ...);
void Debug_Chassis(void);
void Debug_PID(void);
void Debug_Bluetooth(void);
void Debug_PS2(void);
void Debug_Camera(void);
void Debug_Parse(const uint16_t len);
void BUZZ_ON(void);
void BUZZ_OFF(void);

#endif //DEBUG_TOOL_H
