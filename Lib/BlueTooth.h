/**
 * @file BlueTooth.h
 * @brief 蓝牙及串口数据处理头文件
 * @date 2025/10/4
 */

#ifndef INC_BLUETOOTH_H_
#define INC_BLUETOOTH_H_

#include "main.h"

void Uart_Init(void);
void BlueTooth_Stop(void);
void BlueTooth_Start(void);
void Uart_printf(UART_HandleTypeDef *huart, char *format, ...);

#endif /* INC_BLUETOOTH_H_ */
