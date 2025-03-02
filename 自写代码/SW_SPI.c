/**
 *@file SW_SPI.c
 *@author 申飞麟
 *@brief 软件SPI配置（默认模式0，如需更换模式，更改SCK极性和出现的位置即可）
 *@date 2025/3/1
 */

#include "stm32f10x.h"
#include "GPIOX_Init.h"

#define SCK(x) GPIO_WriteBit(GPIOA, GPIO_Pin_5, x)      ///< SCK操作
#define CS(x) GPIO_WriteBit(GPIOA, GPIO_Pin_4, x)       ///< CS操作
#define MISO() GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6) ///< MISO操作
#define MOSI(x) GPIO_WriteBit(GPIOA, GPIO_Pin_7, x)     ///< MOSI操作

/**
 * @brief 软件SPI开启读写功能
 */
void SWSPI_Start(void)
{
    GPIO_WriteBit(GPIOA, GPIO_Pin_4, 0);
}

/**
 * @brief 软件SPI终止读写功能
 */
void SWSPI_End(void)
{
    GPIO_WriteBit(GPIOA, GPIO_Pin_4, 1);
}

/**
 * @brief 软件SPI初始化，引脚默认为CS(A4),SCK(A5),MISO(A6),MOSI(A7), 软件操作可以改引脚
 */
void SWSPI_Init(void)
{
    GPIOX_Init(GPIOA, RCC_APB2Periph_GPIOA, GPIO_Speed_10MHz, GPIO_Mode_Out_PP, GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_7); // 软件下为复用输出，硬件为推挽复用输出
    GPIOX_Init(GPIOA, RCC_APB2Periph_GPIOA, GPIO_Speed_10MHz, GPIO_Mode_IPU, GPIO_Pin_6);                              // MISO为上拉输入，A6
    CS(1);                                                                                                             // 不选中从机
    SCK(0);                                                                                                            // 默认模式0,SCK低电平开始
}

/**
 * @brief 软件SPI交换数据，默认模式0，发送的数据转换为接收的数据，原数据丢失
 * @param Byte 发送的数据
 */
uint8_t SWSPI_Swap(uint8_t Byte)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        MOSI(Byte & 0x80);
        Byte <<= 1;
        SCK(1);
        if (MISO() == 1)
        {
            Byte |= 0x01;
        }
        SCK(0);
    }
    return Byte;
}
