/**
 *@file SW_SPI.h
 *@author 申飞麟
 *@brief 软件SPI配置（默认模式0，如需更换模式，更改SCK极性和出现的位置即可）
 *@date 2025/3/1
 */

#ifndef SW_SPI_H
#define SW_SPI_H

#include "stdint.h"

void SWSPI_Init(void);
uint8_t SWSPI_Swap(uint8_t Byte);
void SWSPI_Start(void);
void SWSPI_End(void);

#endif
