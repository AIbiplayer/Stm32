/**
 * @file SPI_Soft.h
 * @brief 软件SPI，低电平启动，上升沿读取
 * @author 申飞麟
 * @date 2025/8/5
 */

#ifndef SPI_SOFT_H
#define SPI_SOFT_H

#include "main.h"

#define CS(x) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, x)   // CS引脚拉低
#define SCK(x) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, x)  // SCK引脚拉低
#define MOSI(x) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, x) // MOSI引脚拉低
#define MISO() HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)      // MISO引脚读取

void SPI_Soft_Init(void);
uint8_t SPI_Soft_Swap(uint8_t Data);

#endif
