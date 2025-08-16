/**
 * @brief 读取DHT11（支持多引脚，指针形式传递）
 * @date 2025/5/9
 */

#ifndef __DHT11_H
#define __DHT11_H

#include "main.h"

// 定义DHT11结构体，包含引脚和端口信息
typedef struct
{
    GPIO_TypeDef *GPIOx;
    uint16_t GPIO_Pin;
} DHT11_TypeDef;

extern uint8_t Voice_Status;
extern uint8_t TIM5_Repeat;

// 默认使用PC3口（保持向后兼容）
#define DHT11_DEFAULT {GPIOC, GPIO_PIN_3}

// 宏定义修改为使用结构体指针参数
#define DHT11_Read_Data_Default(temp, humi) DHT11_Read_Data(&dht_default, temp, humi)
#define DHT11_HIGH(dht) HAL_GPIO_WritePin((dht)->GPIOx, (dht)->GPIO_Pin, GPIO_PIN_SET)
#define DHT11_LOW(dht) HAL_GPIO_WritePin((dht)->GPIOx, (dht)->GPIO_Pin, GPIO_PIN_RESET)
#define DHT11_DQ_IN(dht) HAL_GPIO_ReadPin((dht)->GPIOx, (dht)->GPIO_Pin)

// 通用函数声明
void DHT11_Operation(void);
void DHT11_Delay_Init(void);
void DHT11_Delay_us(uint32_t us);
HAL_StatusTypeDef DHT11_Read_Data(DHT11_TypeDef *dht, uint8_t *temp, uint8_t *humi);

// 以下函数通常不需要外部调用，但保持声明以保持兼容性
HAL_StatusTypeDef DHT11_Read_Bit(DHT11_TypeDef *dht);
HAL_StatusTypeDef DHT11_Check(DHT11_TypeDef *dht);
uint8_t DHT11_Read_Byte(DHT11_TypeDef *dht);
void DHT11_Rst(DHT11_TypeDef *dht);
void DHT11_IO_OUT(DHT11_TypeDef *dht);
void DHT11_IO_IN(DHT11_TypeDef *dht);

// 向后兼容的宏定义（使用默认PC3口）
static DHT11_TypeDef dht_default = DHT11_DEFAULT;

#endif
