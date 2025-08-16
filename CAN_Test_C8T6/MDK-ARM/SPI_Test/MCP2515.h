/**
 * @file MCP2515.h
 * @brief MCP2515驱动头文件
 * @author 申飞麟
 * @date 2025/8/5
 */

#ifndef MCP2515_H
#define MCP2515_H

#include "main.h"

void MCP2515_Init(void);
void MCP2515_Read_Register_Data(uint8_t Address, uint8_t *ReceiveData, uint8_t Length);
void MCP2515_Send_Tx_Buffer(uint16_t ID, uint8_t *SendData, uint8_t Length);
void MCP2515_Read_Rx_Buffer(uint8_t *ReceiveData, uint8_t Length);

#endif