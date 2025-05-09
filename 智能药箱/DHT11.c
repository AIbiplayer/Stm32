/**
 * @brief 读取DHT11（支持多引脚，指针形式传递）
 * @date 2025/5/9
 */

#include "DHT11.h"
#include "main.h"

static void DHT11_Delay_us(uint16_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t cycles = us * (SystemCoreClock / 1000000);
    while ((DWT->CYCCNT - start) < cycles)
        ;
}

/**
 * @brief 启动DWT计数器以实现精准延时
 */
void DHT11_Delay_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/**
 * @brief DATA引脚设置为输出模式
 */
void DHT11_IO_OUT(DHT11_TypeDef *dht)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = dht->GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(dht->GPIOx, &GPIO_InitStruct);
}

/**
 * @brief DATA引脚设置为输入模式
 */
void DHT11_IO_IN(DHT11_TypeDef *dht)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = dht->GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(dht->GPIOx, &GPIO_InitStruct);
}

/**
 * @brief 复位DHT11
 */
void DHT11_Rst(DHT11_TypeDef *dht)
{
    DHT11_IO_OUT(dht);  // SET OUTPUT
    DHT11_LOW(dht);     // 拉低DQ
    HAL_Delay(20);      // 拉低至少18ms
    DHT11_HIGH(dht);    // DQ=1
    DHT11_Delay_us(30); // 主机拉高20~40us
}

/**
 * @brief DHT11检测函数
 * @return HAL_ERROR为检测不到，HAL_OK为检测到
 */
HAL_StatusTypeDef DHT11_Check(DHT11_TypeDef *dht)
{
    uint8_t retry = 0;
    DHT11_IO_IN(dht);
    while (DHT11_DQ_IN(dht) && retry < 100)
    {
        retry++;
        DHT11_Delay_us(1);
    }
    if (retry >= 100)
        return HAL_ERROR;
    else
        retry = 0;
    while (!DHT11_DQ_IN(dht) && retry < 100) // DHT11拉低后会再次拉高40~80us
    {
        retry++;
        DHT11_Delay_us(1);
    };
    if (retry >= 100)
        return HAL_ERROR;
    return HAL_OK;
}

/**
 * @brief DHT11读取1位
 * @return HAL_ERROR为读取错误，HAL_OK为正常读取
 */
HAL_StatusTypeDef DHT11_Read_Bit(DHT11_TypeDef *dht)
{
    uint8_t retry = 0;
    while (DHT11_DQ_IN(dht) && retry < 100) // 等待变为低电平
    {
        retry++;
        DHT11_Delay_us(1);
    }
    retry = 0;
    while (!DHT11_DQ_IN(dht) && retry < 100) // 等待变高电平
    {
        retry++;
        DHT11_Delay_us(1);
    }
    DHT11_Delay_us(40); // 等待40us
    if (DHT11_DQ_IN(dht))
        return HAL_ERROR;
    else
        return HAL_OK;
}

/**
 * @brief DHT11读取1字节
 * @return 读取值
 */
uint8_t DHT11_Read_Byte(DHT11_TypeDef *dht)
{
    uint8_t i, dat;
    dat = 0;
    for (i = 0; i < 8; i++)
    {
        dat <<= 1;
        dat |= DHT11_Read_Bit(dht);
    }
    return dat;
}

/**
 * @brief DHT11读取一次数据，温度为0~50°，湿度为20%~90%
 * @param dht DHT11结构体指针，包含GPIO端口和引脚信息
 * @param temp 温度值指针
 * @param humi 湿度值指针
 * @return HAL_ERROR为读取错误，HAL_OK为正常读取
 */
HAL_StatusTypeDef DHT11_Read_Data(DHT11_TypeDef *dht, uint8_t *temp, uint8_t *humi)
{
    uint8_t buf[5];
    uint8_t i;
    DHT11_Rst(dht);
    if (DHT11_Check(dht) == HAL_OK)
    {
        for (i = 0; i < 5; i++) // 读取40位数据
        {
            buf[i] = DHT11_Read_Byte(dht);
        }
        if ((buf[0] + buf[1] + buf[2] + buf[3]) == buf[4])
        {
            *humi = buf[0];
            *temp = buf[2];
            return HAL_OK;
        }
    }
    return HAL_ERROR;
}