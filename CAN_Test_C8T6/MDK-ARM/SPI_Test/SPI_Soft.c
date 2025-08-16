/**
 * @file SPI_Soft.c
 * @brief 软件SPI，低电平启动，上升沿读取，空闲时SCK低电平
 * @author 申飞麟
 * @date 2025/8/5
 */

#include "main.h"
#include "SPI_Soft.h"

/**
 * @brief 初始化软件SPI
 */
void SPI_Soft_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0}; // 初始化GPIO引脚

    __HAL_RCC_GPIOA_CLK_ENABLE(); // 使能GPIOA时钟

    // 配置CS引脚
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // 配置SCK引脚
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // 配置MOSI引脚
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // 配置MISO引脚为输入
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    CS(1); // CS引脚拉高
    SCK(0);
}

/**
 * @brief 交换数据，CS不变换
 * @param Data 数据
 * @return 交换的数据
 */
uint8_t SPI_Soft_Swap(uint8_t Data)
{
    uint8_t ReceiveData = 0; // 获得的数据

    for (uint8_t i = 0; i < 8; i++) // 数据交换
    {
        MOSI((Data << i) & 0x80);
        SCK(1);
        ReceiveData |= MISO() << (7 - i);
        SCK(0);
    }

    return ReceiveData;
}
