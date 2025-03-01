/**
 *@file SW_IIC.h
 *@author 申飞麟
 *@brief 软件IIC配置头文件
 *@date 2025/2/27
 */

#ifndef SW_IIC_H
#define SW_IIC_H

#include "stdint.h"

uint8_t SCL_R(void);
uint8_t SDA_R(void);
void IIC_Init(void);
void IIC_Start(void);
void IIC_End(void);
void IIC_Send(uint8_t Byte);
uint8_t IIC_Receive(void);
void IIC_SendACK(uint8_t ACK);
uint8_t IIC_ReceiveACK(void);

#endif
