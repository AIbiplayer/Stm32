/**
 * @brief UART通信
 * @date 2025/4/17
 */

#include "string.h"
#include "main.h"
#include "UART.h"
#include "stm32u5xx_hal_uart.h"
#include "stm32u5xx_hal_usart.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

#define DMASIZE 128 ///< 最大长度128
#define OK 0        ///< 正确
#define Error 1     ///< 错误

char ReceiveData[DMASIZE] = {0}; ///< 接收的原始数据
char ReceiveCmd[32] = {0};       ///< 处理后的指令
char ReceiveTopic[32] = {0};     ///< 接收的主题
char ReceiveStatus[32] = {1};    ///< 接收指令的状态

const char WiFiConnect[] = "AT+CWJAP=\"wocao\",\"99999999\"\r\n";                                                   ///< WiFi连接指令
const char KeyConnect[] = "AT+MQTTUSERCFG=0,1,\"b0573a46ef944339acb37bda7913cd12\",\"NULL\",\"NULL\",0,0,\"\"\r\n"; ///< MQTT配置
const char BemfaConnect[] = "AT+MQTTCONN=0,\"bemfa.com\",9501,0\r\n";                                               ///< 连接巴法云平台
const char CipMode[] = "AT+CIPMODE=1\r\n";                                                                          ///< 透传模式

// 订阅主题，可自由添加
const char Topic_LED_Connect[] = "AT+MQTTSUB=0,\"Ping\",0\r\n";
const char Topic_LED_SendCmd[] = "AT+MQTTPUB=0,\"Ping/set\",\"666\",0,0\r\n";

/**
 * @brief 等待指令函数
 */
uint8_t WaitForACK(void)
{
    char ReceiveBuff[DMASIZE];
    strcpy(ReceiveBuff, ReceiveData);
    uint32_t Time = 100000;
    while (Time)
    {
        Time--;
        if (strstr(ReceiveBuff,"OK") != NULL)
        {
            return OK;
        }
    }
    return Error;
}

/**
 * @brief 连接WIFI
 */
ESP_Status Connect_WiFi(void)
{
    HAL_Delay(1000);
    HAL_UART_Transmit_DMA(&huart2, (uint8_t *)WiFiConnect, strlen(WiFiConnect));
    HAL_Delay(3500);
    if (WaitForACK() == OK)
    {
        return WiFi_OK;
    }
    return ESP_ERROR;
}

/**
 * @brief 连接MQTT
 */
ESP_Status Connect_MQTT(void)
{
    HAL_Delay(200);
    HAL_UART_Transmit_DMA(&huart2, (uint8_t *)KeyConnect, strlen(KeyConnect));
    HAL_Delay(200);
    if (WaitForACK() == OK)
    {
        return MQTT_OK;
    }
    return ESP_ERROR;
}

/**
 * @brief 连接巴法云
 */
ESP_Status Connect_Bemfa(void)
{
    HAL_UART_Transmit_DMA(&huart2, (uint8_t *)BemfaConnect, strlen(BemfaConnect));
    HAL_Delay(3500);
    if (WaitForACK() == OK)
    {
        return Bemfa_OK;
    }
    return ESP_ERROR;
}

/**
 * @brief 设置透传模式
 */
ESP_Status Send_Cipmode(void)
{
    HAL_UART_Transmit_DMA(&huart2, (uint8_t *)CipMode, strlen(CipMode));
    HAL_Delay(200);
    if (WaitForACK() == OK)
    {
        return CipMode_OK;
    }
    return ESP_ERROR;
}

/**
 * @brief 订阅主题，此处可自由添加
 * @param Topic 主题
 */
ESP_Status Connect_Topic(void)
{
    HAL_UART_Transmit_DMA(&huart2, (uint8_t *)Topic_LED_Connect, strlen(Topic_LED_Connect));
    HAL_Delay(200);
    if (WaitForACK() == OK)
    {
        return Topic_OK;
    }
    return ESP_ERROR;
}

/**
 * @brief 总连接函数，任意中断则失败
 */
ESP_Status All_Connect(void)
{
    HAL_UART_Receive_DMA(&huart2, (uint8_t *)ReceiveData, 128);
    if (Connect_WiFi() != WiFi_OK)
    {
        return ESP_ERROR;
    }
    if (Connect_MQTT() != MQTT_OK)
    {
        return ESP_ERROR;
    }
    if (Connect_Bemfa() != Bemfa_OK)
    {
        return ESP_ERROR;
    }
    if (Send_Cipmode() != CipMode_OK)
    {
        return ESP_ERROR;
    }
    if (Connect_Topic() != Topic_OK)
    {
        return ESP_ERROR;
    }
    return Online_OK;
}

/**
 * @brief 数据处理函数
 */
void Parse_MQTTData(void)
{
    char *p = ReceiveData; // 指向原始数据
    char *topicStart, *topicEnd, *dataStart;
    int len = 0;

    // 检查是否以 "+MQTTSUBRECV:" 开头
    if (strncmp(p, "+MQTTSUBRECV:", 12) != 0)
    {
        return; // 不是MQTT订阅消息，直接返回
    }
    p += 12; // 跳过 "+MQTTSUBRECV:"

    // 跳过第一个数字（消息ID）
    while (*p != ',')
        p++;
    p++; // 跳过逗号

    // 提取主题（位于双引号之间）
    if (*p == '"')
    {
        topicStart = ++p; // 跳过第一个引号
        while (*p != '"' && *p != '\0')
            p++;
        topicEnd = p;
        p++; // 跳过第二个引号
    }
    else
    {
        return; // 格式错误
    }

    // 复制主题到 ReceiveTopic
    len = topicEnd - topicStart;
    if (len >= 32)
        len = 31; // 防止溢出
    strncpy(ReceiveTopic, topicStart, len);
    ReceiveTopic[len] = '\0'; // 确保字符串结束

    // 跳过逗号和长度字段（不需要atoi，直接跳到最后一个逗号）
    while (*p != ',' && *p != '\0')
        p++;
    p++; // 跳过逗号
    while (*p != ',' && *p != '\0')
        p++;
    p++; // 跳过逗号

    // 剩余部分就是有效数据
    dataStart = p;
    len = strlen(dataStart);
    if (len >= 32)
        len = 31; // 防止溢出
    strncpy(ReceiveCmd, dataStart, len);
    ReceiveCmd[len] = '\0'; // 确保字符串结束
}

void Operation(void)
{
    if (strcmp(ReceiveTopic, "Ping") == 0 && strcmp(ReceiveCmd, "666"))
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET); // 绿
    }
}

/**
 * @brief 接收函数
 */
void USART2_IRQHandler(void) // 测试用U1，真正用U3
{
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE))
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart2);
        HAL_UART_DMAStop(&huart2);

        Parse_MQTTData();

        HAL_UART_Receive_DMA(&huart2, (uint8_t *)ReceiveData, 128);
    }
    HAL_UART_IRQHandler(&huart2);
}
