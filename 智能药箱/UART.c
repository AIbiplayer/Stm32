/**
 * @brief UART通信
 * @date 2025/4/17
 */

#include "string.h"
#include "main.h"
#include "UART.h"
#include "Motor.h"
#include "stm32u5xx_hal_uart.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

#define DMASIZE 128 ///< 最大长度128
#define OK 0        ///< 正确
#define Error 1     ///< 错误

char ReceiveData[DMASIZE] = {0}; ///< 接收的原始数据
char ReceiveCmd[32] = {0};       ///< 处理后的指令
char ReceiveTopic[32] = {0};     ///< 接收的主题

const char WiFiConnect[] = "AT+CWJAP=\"wocao\",\"99999999\"\r\n";                                                   ///< WiFi连接指令
const char KeyConnect[] = "AT+MQTTUSERCFG=0,1,\"b0573a46ef944339acb37bda7913cd12\",\"NULL\",\"NULL\",0,0,\"\"\r\n"; ///< MQTT配置
const char BemfaConnect[] = "AT+MQTTCONN=0,\"bemfa.com\",9501,0\r\n";                                               ///< 连接巴法云平台
const char CipMode[] = "AT+CIPMODE=1\r\n";                                                                          ///< 透传模式

// 订阅在线状态主题
const char Topic_Ping_Connect[] = "AT+MQTTSUB=0,\"Ping\",0\r\n";
const char Topic_Ping_SendCmd[] = "AT+MQTTPUB=0,\"Ping/set\",\"OnlineOK\",0,0\r\n"; // 发送OnlineOK表示收到
// 订阅药柜温湿度发送主题
const char Topic_CabinetTH_Connect[] = "AT+MQTTSUB=0,\"CabinetTH\",0\r\n";
const char Topic_CabinetTH_SendCmd[] = "AT+MQTTPUB=0,\"CabinetTH/set\",\"T25H75\",0,0\r\n"; // 这里传递温湿度
// 订阅药柜位置接收主题
const char Topic_CabinetLocation_Connect[] = "AT+MQTTSUB=0,\"CabinetLocation\",0\r\n";
const char Topic_CabinetLocation_SendCmd[] = "AT+MQTTPUB=0,\"CabinetLocation/set\",\"OpenOK\",0,0\r\n"; // 发送OpenOK表示柜子打开
// 订阅关闭药柜主题
const char Topic_CloseCabinet_Connect[] = "AT+MQTTSUB=0,\"CloseCabinet\",0\r\n";
const char Topic_CloseCabinet_SendCmd[] = "AT+MQTTPUB=0,\"CloseCabinet/set\",\"CloseOK\",0,0\r\n"; // 发送CloseOK表示柜子关闭
// 订阅冷藏柜温湿度发送主题
const char Topic_ColdCabinetTH_Connect[] = "AT+MQTTSUB=0,\"ColdCabinetTH\",0\r\n";
const char Topic_ColdCabinetTH_SendCmd[] = "AT+MQTTPUB=0,\"ColdCabinetTH/set\",\"T10H75\",0,0\r\n"; // 这里传递冷藏柜温湿度
// 订阅服药提醒主题
const char Topic_MedicineReminder_Connect[] = "AT+MQTTSUB=0,\"MedicineReminder\",0\r\n";
// 订阅RTC时钟获取主题
const char Topic_RTC_Connect[] = "AT+MQTTSUB=0,\"RTC\",0\r\n";

/**
 * @brief 总连接函数，连接完亮蓝灯
 */
void All_Connect(void)
{
    HAL_UART_Receive_DMA(&huart2, (uint8_t *)ReceiveData, 128);
    HAL_Delay(2000);
    HAL_UART_Transmit(&huart2, (uint8_t *)KeyConnect, strlen(KeyConnect), HAL_MAX_DELAY); // 连接MQTT
    HAL_Delay(10);
    HAL_UART_Transmit(&huart2, (uint8_t *)BemfaConnect, strlen(BemfaConnect), HAL_MAX_DELAY); // 连接巴法云
    HAL_Delay(3500);
    HAL_UART_Transmit(&huart2, (uint8_t *)CipMode, strlen(CipMode), HAL_MAX_DELAY); // 设置透传
    HAL_Delay(10);
    for (int i = 0; i < 1; i++) // 重复订阅主题
    {
        HAL_UART_Transmit(&huart2, (uint8_t *)Topic_Ping_Connect, strlen(Topic_Ping_Connect), HAL_MAX_DELAY);
        HAL_Delay(1000);
        HAL_UART_Transmit(&huart2, (uint8_t *)Topic_CabinetLocation_Connect, strlen(Topic_CabinetLocation_Connect), HAL_MAX_DELAY);
        HAL_Delay(1000);
        HAL_UART_Transmit(&huart2, (uint8_t *)Topic_CabinetTH_Connect, strlen(Topic_CabinetTH_Connect), HAL_MAX_DELAY);
        HAL_Delay(1000);
        HAL_UART_Transmit(&huart2, (uint8_t *)Topic_CloseCabinet_Connect, strlen(Topic_CloseCabinet_Connect), HAL_MAX_DELAY);
        HAL_Delay(1000);
        HAL_UART_Transmit(&huart2, (uint8_t *)Topic_ColdCabinetTH_Connect, strlen(Topic_ColdCabinetTH_Connect), HAL_MAX_DELAY);
        HAL_Delay(1000);
        HAL_UART_Transmit(&huart2, (uint8_t *)Topic_MedicineReminder_Connect, strlen(Topic_MedicineReminder_Connect), HAL_MAX_DELAY);
        HAL_Delay(1000);
        HAL_UART_Transmit(&huart2, (uint8_t *)Topic_RTC_Connect, strlen(Topic_RTC_Connect), HAL_MAX_DELAY);
        HAL_Delay(1000);
    }
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
    while (*p != ',' && *p != '\0')
        p++;
    if (*p == '\0')
        return; // 格式错误
    p++;        // 跳过逗号

    // 提取主题（位于双引号之间）
    if (*p == '"')
    {
        topicStart = ++p; // 跳过第一个引号
        while (*p != '"' && *p != '\0')
            p++;
        if (*p == '\0')
            return; // 格式错误
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

    // 跳过逗号和长度字段（直接跳到最后一个逗号）
    while (*p != ',' && *p != '\0')
        p++;
    if (*p == '\0')
        return; // 格式错误
    p++;        // 跳过逗号
    while (*p != ',' && *p != '\0')
        p++;
    if (*p == '\0')
        return; // 格式错误
    p++;        // 跳过逗号

    // 提取消息（去除末尾的\r\n或其他非可见字符）
    dataStart = p;
    while (*p != '\r' && *p != '\n' && *p != '\0')
        p++;
    len = p - dataStart; // 计算消息实际长度（到\r或\n前）
    if (len >= 32)
        len = 31; // 防止溢出
    strncpy(ReceiveCmd, dataStart, len);
    ReceiveCmd[len] = '\0'; // 确保字符串结束
}

/**
 * @brief 接收数据函数，主要是DMA重启
 */
void USART2_IRQHandler(void)
{
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE))
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart2);

        Parse_MQTTData(); // 解析数据

        if (strcmp(ReceiveTopic, "") != 0 && strcmp(ReceiveCmd, "") != 0)
        {
            if (strcmp(ReceiveTopic, "Ping") == 0 && strcmp(ReceiveCmd, "Ping") == 0) // Ping检测
            {
                HAL_UART_Transmit(&huart2, (uint8_t *)Topic_Ping_SendCmd, strlen(Topic_Ping_SendCmd), HAL_MAX_DELAY);
            }
            else if (strcmp(ReceiveTopic, "CabinetLocation") == 0) // 开柜门识别
            {
                if (strcmp(ReceiveCmd, "1") == 0) // 开柜门1
                {
                    Rotate_Motor_Move(1);
                }
                else if (strcmp(ReceiveCmd, "2") == 0) // 开柜门2
                {
                    Rotate_Motor_Move(2);
                }
                else if (strcmp(ReceiveCmd, "3") == 0) // 开柜门3
                {
                    Rotate_Motor_Move(3);
                }
                else if (strcmp(ReceiveCmd, "4") == 0) // 开柜门4
                {
                    Rotate_Motor_Move(4);
                }
                else if (strcmp(ReceiveCmd, "5") == 0) // 开柜门5
                {
                    Rotate_Motor_Move(5);
                }
                HAL_UART_Transmit(&huart2, (uint8_t *)Topic_CabinetLocation_SendCmd, strlen(Topic_CabinetLocation_SendCmd), HAL_MAX_DELAY);
            }
            strcpy(ReceiveCmd, "");
            strcpy(ReceiveTopic, "");
        }
        HAL_UART_DMAStop(&huart2);
        HAL_UART_Receive_DMA(&huart2, (uint8_t *)ReceiveData, 128);
        HAL_UART_IRQHandler(&huart2);
    }
}
