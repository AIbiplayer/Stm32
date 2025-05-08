/**
 * @brief Flash烧写
 * @date 2025/4/17
 */

#ifndef FLASH_H
#define FLASH_H

void Flash_Erase(uint32_t Banks, uint32_t Page, uint32_t NPages);
void Flash_Write(uint32_t Address, uint32_t DataAddress);
void Flash_Read(uint32_t *ReadData, uint32_t Address);

#endif
