/**
 * @brief 读取DHT11（支持多引脚，指针形式传递）
 * @date 2025/5/9
 */

#include "DHT11.h"
#include "main.h"
#include "UART.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim5;
extern uint8_t Online_Status;

uint8_t Voice_Status = 0;      ///< 语音状态，为1时不播报，由TIM5控制
uint8_t TIM5_Repeat = 0;       ///< TIM5计时重复次数
uint8_t Temp = 25, Humid = 75; ///< 温湿度

const char Voice_Online_Error[] = {0xCD, 0xF8, 0xC2, 0xE7, 0xC1, 0xAC, 0xBD, 0xD3, 0xD2, 0xEC, 0xB3, 0xA3, 0x00}; ///< 网络连接异常

/**
 * @brief DHT11延迟函数，采用DWT计数器
 */
void DHT11_Delay_us(uint32_t us)
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
    DHT11_IO_OUT(dht);     // SET OUTPUT
    DHT11_LOW(dht);        // 拉低DQ
    DHT11_Delay_us(19000); // 拉低至少18ms
    DHT11_HIGH(dht);       // DQ=1
    DHT11_Delay_us(30);    // 主机拉高20~40us
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

/**
 * @brief DHT11读取数据并操作，默认正常柜为PC3，冷藏柜为PB0
 */
void DHT11_Operation(void)
{
    DHT11_TypeDef DHT11_C3; ///< 正常柜
    DHT11_C3.GPIOx = GPIOC;
    DHT11_C3.GPIO_Pin = GPIO_PIN_3;

    DHT11_TypeDef DHT11_B0; ///< 冷藏柜
    DHT11_B0.GPIOx = GPIOB;
    DHT11_B0.GPIO_Pin = GPIO_PIN_0;

    // 药柜读取失败发送Error，读取成功发送温湿度
    if (DHT11_Read_Data(&DHT11_C3, &Temp, &Humid) != HAL_OK)
    {
        Uart_printf(&huart2, "AT+MQTTPUB=0,\"CabinetTH/set\",\"Error\",0,0\r\n");
    }
    else
    {
        Uart_printf(&huart2, "AT+MQTTPUB=0,\"CabinetTH/set\",\"T%dH%d\",0,0\r\n", Temp, Humid);
    }

    // 冷藏柜读取失败发送Error，读取成功发送温湿度
    if (DHT11_Read_Data(&DHT11_B0, &Temp, &Humid) != HAL_OK)
    {
        Uart_printf(&huart2, "AT+MQTTPUB=0,\"ColdCabinetTH/set\",\"Error\",0,0\r\n");
    }
    else
    {
        Uart_printf(&huart2, "AT+MQTTPUB=0,\"ColdCabinetTH/set\",\"T%dH%d\",0,0\r\n", Temp, Humid);
    }
}

/**
 * @brief 使用TIM5进行中断，每1秒检测，每10秒语音播报一次，每60秒检测网络一次
 */
void TIM5_IRQHandler(void)
{
    static uint8_t Online_Repeat = 0; // 网络检测
    static uint8_t Online_Number = 0; // 网络检测次数
    TIM5_Repeat++;
    if (TIM5_Repeat == 10)
    {
        Online_Repeat++;
        Voice_Status = 0;
        TIM5_Repeat = 0;
    }
    if (Online_Repeat == 6 && Online_Number == 0)
    {
        Online_Status = 0;
        Online_Number = 1;
        Online_Repeat = 0;
    }
    else if (Online_Repeat == 6 && Online_Number != 0 && Online_Status == 0)
    {
        Online_Number = 0;
        Voice_Remind(Voice_Online_Error);
    }
    DHT11_Operation();
    HAL_TIM_IRQHandler(&htim5);
}
