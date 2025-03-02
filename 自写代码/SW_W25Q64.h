/**
 *@file SW_W25Q64.h
 *@author 申飞麟
 *@brief 软件W25Q64操作
 *@date 2025/3/1
 */

#ifndef SW_W25Q64_H
#define SW_W25Q64_H

#include "stdint.h"

void SW_W25Q64_Init(void);
uint32_t SW_W25Q64_GetID(void);
void SW_W25Q64_Write(uint8_t *DataArray, uint32_t Address, uint8_t Count);
void SW_W25Q64_Read(uint8_t *DataArray, uint32_t Address, uint32_t Count);
void SW_W25Q64_SectorErase(uint32_t Address);

#endif
