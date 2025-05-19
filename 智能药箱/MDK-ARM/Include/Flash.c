/**
 * @brief Flash烧写
 * @date 2025/4/17
 */

#include "main.h"
#include "stm32u5xx_hal_flash.h"

#define BANK1_ADDRESS 0x08080000        ///< Bank1地址，64页起步
#define BANK2_ADDRESS 0x08100000        ///< Bank2地址，0页起步
#define RTC_Alarm_ADDRESS 0x08082000    ///< Bank1的第65页地址，此页用于存放服药闹钟提醒

/**
 * @brief Flash擦除
 * @param Banks 块1或2
 * @param Page 页码，块1要64起步，不大于127
 * @param NPages 擦除页数
 */
void Flash_Erase(uint32_t Banks, uint32_t Page, uint32_t NPages)
{
    HAL_ICACHE_Disable(); // 禁用指令缓存
    HAL_FLASH_Unlock();
    uint32_t ErrorCode = 0;

    FLASH_EraseInitTypeDef Flash;
    Flash.TypeErase = FLASH_TYPEERASE_PAGES;
    Flash.Banks = Banks;
    Flash.NbPages = NPages;
    Flash.Page = Page;

    HAL_FLASHEx_Erase(&Flash, &ErrorCode);
    FLASH_WaitForLastOperation(1000U);
    HAL_ICACHE_Enable(); // 重新启用

    HAL_FLASH_Lock();
}

/**
 * @brief Flash写入
 * @param Address 写入的地址，必须16位对齐，如0x08080000 和 0x08080010
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
void Flash_Read(uint32_t *ReadData, uint32_t Address)
{
    *ReadData = *(__IO uint32_t *)Address;
}
