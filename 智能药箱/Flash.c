/**
 * @brief Flash烧写
 * @date 2025/4/17
 */

#include "main.h"
#include "stm32u5xx_hal_flash.h"

/**
 * @brief Flash擦除
 * @param Banks 块1或2
 * @param Pages 页码，块1要64起步，不大于127
 * @param ErrorCode 错误返回代码
 */
void Flash_Erase(uint32_t Banks, uint32_t Pages, uint32_t NPages, uint32_t ErrorCode)
{
    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef Flash;
    Flash.TypeErase = FLASH_TYPEERASE_PAGES;
    Flash.Banks = Banks;
    Flash.NbPages = NPages;
    Flash.Page = Pages;

    HAL_FLASHEx_Erase(&Flash, &ErrorCode);
    HAL_FLASH_Lock();
}

/**
 * @brief Flash写入
 * @param Address 读取的地址
 * @param DataAddress 写入的数据的地址
 */
void Flash_Write(uint32_t Address, uint32_t DataAddress)
{
    HAL_FLASH_Unlock();
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD, Address, DataAddress);
    HAL_FLASH_Lock();
}

/**
 * @brief Flash读取
 * @param ReadData 转运的地址
 * @param Address 读取数据的地址
 */
void Flash_Read(uint64_t *ReadData, uint32_t Address)
{
    HAL_FLASH_Unlock();
    *ReadData = *(__IO uint64_t *)Address;
    HAL_FLASH_Lock();
}
