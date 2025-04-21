/**
 * @brief UART通信
 * @date 2025/4/17
 */

#include "string.h"
#include "stm32u5xx_hal_uart.h"
#include "stm32u5xx_hal_usart.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

#define DMASIZE 128 ///< 最大长度128
#define OK 0        ///< 指令正确
#define Error 1     ///< 指令错误

char ReceiveData[DMASIZE] = {0}; ///< 接收的原始数据
char ReceiveCmd[32] = {0};       ///< 处理后的指令
char ReceiveTopic[32] = {0};     ///< 接收的主题

const char WiFiConnect[] = "AT+CWJAP=\"wocao\",\"99999999\"\r\n";                                                   ///< WiFi连接指令
const char KeyConnect[] = "AT+MQTTUSERCFG=0,1,\"b0573a46ef944339acb37bda7913cd12\",\"NULL\",\"NULL\",0,0,\"\"\r\n"; ///< MQTT配置
const char BemfaConnect[] = "AT+MQTTCONN=0,\"bemfa.com\",9501,0\r\n";                                               ///< 连接巴法云平台
const char CipMode[] = "AT+CIPMODE=1\r\n";                                                                          ///< 透传模式

// 订阅主题，可自由添加
const char Topic_LED_Connect[] = "AT+MQTTSUB=0,\"LED\",0\r\n";
const char Topic_LED_SendCmd[] = "AT+MQTTPUB=0,\"LED/set\",\"666\",0,0\r\n";

/**
 * @brief 等待指令函数
 */
uint8_t WaitForACK(char *ACK)
{
    uint32_t Time = 100000;
    while (Time)
    {
        Time--;
        if (strstr(ACK, ReceiveData) != NULL)
        {
            return OK;
        }
    }
    return Error;
}

/**
 * @brief 连接WIFI
 */
uint8_t Connect_WiFi(void)
{
    HAL_UART_Transmit_DMA(&huart1, (uint8_t *)WiFiConnect, strlen(WiFiConnect));
    HAL_Delay(1000);
    if (WaitForACK("OK") == OK)
    {
        return OK;
    }
    return Error;
}

/**
 * @brief 连接MQTT
 */
uint8_t Connect_MQTT(void)
{
    HAL_UART_Transmit_DMA(&huart1, (uint8_t *)KeyConnect, strlen(KeyConnect));
    if (WaitForACK("OK") == OK)
    {
        return OK;
    }
    return Error;
}

/**
 * @brief 连接巴法云
 */
uint8_t Connect_Bemfa(void)
{
    HAL_UART_Transmit_DMA(&huart1, (uint8_t *)BemfaConnect, strlen(BemfaConnect));
    HAL_Delay(3500);
    if (WaitForACK("OK") == OK)
    {
        return OK;
    }
    return Error;
}

/**
 * @brief 设置透传模式
 */
uint8_t Send_Cipmode(void)
{
    HAL_UART_Transmit_DMA(&huart1, (uint8_t *)CipMode, strlen(CipMode));
    if (WaitForACK("OK") == OK)
    {
        return OK;
    }
    return Error;
}

/**
 * @brief 订阅主题，此处可自由添加
 * @param Topic 主题
 */
uint8_t Connect_Topic(void)
{
    HAL_UART_Transmit_DMA(&huart1, (uint8_t *)Topic_LED_Connect, strlen(Topic_LED_Connect));
    if (WaitForACK("OK") == OK)
    {
        return OK;
    }
    return Error;
}

/**
 * @brief 总连接函数，任意中断则失败
 */
uint8_t All_Connect(void)
{
    if (Connect_WiFi() != OK)
    {
        return Error;
    }
    if (Connect_MQTT() != OK)
    {
        return Error;
    }
    if (Connect_Bemfa() != OK)
    {
        return Error;
    }
    if (Send_Cipmode() != OK)
    {
        return Error;
    }
    if (Connect_Topic() != OK)
    {
        return Error;
    }
    return OK;
}

/**
 * @brief 接收函数
 */
void USART1_IRQHandler(void) // 测试用U1，真正用U3
{
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE))
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart1);

        HAL_UART_DMAStop(&huart1);
        HAL_UART_Receive_DMA(&huart1, (uint8_t *)ReceiveData, 128);
        OLED_ShowString(1, 1, ReceiveData);
    }
    HAL_UART_IRQHandler(&huart1);
}
