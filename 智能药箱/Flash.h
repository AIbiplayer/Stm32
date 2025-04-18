#ifndef FLASH_H
#define FLASH_H

void Flash_Erase(uint32_t Banks, uint32_t Pages, uint32_t NPages, uint32_t ErrorCode);
void Flash_Write(uint32_t Address, uint32_t DataAddress);
void Flash_Read(uint64_t *ReadData, uint32_t Address);

#endif
