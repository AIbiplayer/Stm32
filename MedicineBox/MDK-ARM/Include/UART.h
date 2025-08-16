/**
 * @brief UART通信
 * @date 2025/4/17
 */

#ifndef UART_H
#define UART_H

#include "main.h"

extern uint8_t Cabinet_List;

void All_Connect(void);
void Parse_MQTTData(void);
void Voice_Remind(const char *String);
void Uart_printf(UART_HandleTypeDef *huart, char *format, ...);

#endif
