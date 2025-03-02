/**
 *@file SW_W25Q64.c
 *@author 申飞麟
 *@brief 软件W25Q64操作
 *@date 2025/3/1
 */

#include "stm32f10x.h"
#include "SW_SPI.h"

/**
 * @brief 软件W25Q64开启写使能
 */
void SW_W25Q64_WriteEnable(void)
{
    SWSPI_Start();
    SWSPI_Swap(0x06);
    SWSPI_End();
}

/**
 * @brief 软件W25Q64忙等待，超时则不等待
 */
void SW_W25Q64_WaitBuzy(void)
{
    uint32_t Time = 100000;
    SWSPI_Start();
    SWSPI_Swap(0x05);
    while ((SWSPI_Swap(0xFF) & 0x01))
    {
        Time--;
        if (!Time)
        {
            break;
        }
    }
    SWSPI_End();
}

/**
 * @brief 软件W25Q64擦除指定扇区,擦除是写入的过程
 * @param Address 指定扇区地址,后三位在同一扇区,如0x123FFF
 */
void SW_W25Q64_SectorErase(uint32_t Address)
{
    SW_W25Q64_WriteEnable();

    SWSPI_Start();
    SWSPI_Swap(0x20);
    SWSPI_Swap(Address >> 16);
    SWSPI_Swap(Address >> 8);
    SWSPI_Swap(Address);
    SWSPI_End();
    SW_W25Q64_WaitBuzy(); // 擦除完也要等待
}

/**
 * @brief 软件W25Q64初始化，等于软件SPI的初始化
 */
void SW_W25Q64_Init(void)
{
    SWSPI_Init();
}

/**
 * @brief 软件W25Q64获取生产ID
 * @return ID号
 */
uint32_t SW_W25Q64_GetID(void)
{
    uint32_t ID = 0;
    SWSPI_Start();
    SWSPI_Swap(0x9F);
    ID |= SWSPI_Swap(0xFF) << 16;
    ID |= SWSPI_Swap(0xFF) << 8;
    ID |= SWSPI_Swap(0xFF);
    SWSPI_End();
    return ID;
}

/**
 * @brief 软件W25Q64向指定地址写数据，写入前判断是否要擦除扇区数据，避免数据覆盖
 * @param DataArray 要发送的数据包含在数组内
 * @param Address 要写入的24位地址，比如0x123456，建议后三位为000，方便擦除
 * @param Count 数组长度，不能超过256
 */
void SW_W25Q64_Write(uint8_t *DataArray, uint32_t Address, uint8_t Count)
{
    SW_W25Q64_WriteEnable(); // 开启写使能

    SWSPI_Start();
    SWSPI_Swap(0x02);
    SWSPI_Swap(Address >> 16);
    SWSPI_Swap(Address >> 8);
    SWSPI_Swap(Address);
    for (uint8_t i = 0; i < Count; i++)
    {
        SWSPI_Swap(DataArray[i]);
    }
    SWSPI_End();
    SW_W25Q64_WaitBuzy();
}

/**
 * @brief 软件W25Q64从指定地址读数据
 * @param DataArray 要读的数据包含在数组内
 * @param Address 要读的24位地址，比如0x123456
 * @param Count 数组长度
 */
void SW_W25Q64_Read(uint8_t *DataArray, uint32_t Address, uint32_t Count)
{
    SWSPI_Start();
    SWSPI_Swap(0x03);
    SWSPI_Swap(Address >> 16);
    SWSPI_Swap(Address >> 8);
    SWSPI_Swap(Address);
    for (uint8_t i = 0; i < Count; i++)
    {
        DataArray[i] = SWSPI_Swap(0xFF);
    }
    SWSPI_End();
}
