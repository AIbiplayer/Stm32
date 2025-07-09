/**
 * @brief UART通信
 * @date 2025/4/17
 */

#include "stdarg.h"
#include "stdlib.h"
#include "stdio.h"
#include "string.h"
#include "main.h"
#include "DHT11.h"
#include "UART.h"
#include "Motor.h"
#include "Flash.h"
#include "RTC_Medicine.h"
#include "stm32u5xx_hal_uart.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2; // 传输和接收WiFi数据
extern UART_HandleTypeDef huart3; // 语音播报
extern RTC_HandleTypeDef hrtc;
extern RTC_DateTypeDef sDate;
extern RTC_TimeTypeDef sTime;
extern uint8_t Medicine_Remind_Status;

#define DMASIZE 1024 ///< 最大长度1024

char ReceiveData[DMASIZE] = {0}; ///< 接收的原始数据
char ReceiveCmd[512] = {0};      ///< 处理后的指令
char ReceiveTopic[512] = {0};    ///< 接收的主题
uint8_t Cabinet_List = 0;        ///< 柜子标志位，中断结束后执行旋转
uint8_t Online_Status = 0;       ///< 网络连接情况，1为已连接
uint8_t Fan_Status = 0;          ///< 制冷状态，1为制冷

// 服药提醒语音
extern const char Voice_Reminder[];
const char Voice_Connect[] = {0xCD, 0xF8, 0xC2, 0xE7, 0xC1, 0xAC, 0xBD, 0xD3, 0xB3, 0xC9, 0xB9, 0xA6, 0x00};        ///< 网络连接成功语音
const char Voice_MedicineOK[] = {0xB7, 0xFE, 0xD2, 0xA9, 0xB3, 0xC9, 0xB9, 0xA6, 0x00};                             ///< 网络连接成功语音
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
const char Topic_MedicineReminder_SendCmd[] = "AT+MQTTPUB=0,\"MedicineReminder/set\",\"CleanOK\",0,0\r\n"; // 这里传递成功清除服药数据
// 订阅RTC时钟获取主题
const char Topic_RTC_Connect[] = "AT+MQTTSUB=0,\"RTC\",0\r\n";
// 订阅制冷控制主题
const char Topic_FanControl_Connect[] = "AT+MQTTSUB=0,\"FanControl\",0\r\n";

/**
 * @brief printf打印数据到指定串口
 */
void Uart_printf(UART_HandleTypeDef *huart, char *format, ...)
{
    static char buf[1024]; // 定义临时数组，根据实际发送大小微调
    va_list args;
    va_start(args, format);
    uint32_t len = vsnprintf((char *)buf, sizeof(buf), (char *)format, args);
    va_end(args);
    HAL_UART_Transmit(huart, (uint8_t *)buf, len, 100);
}

/**
 * @brief 总连接函数，连接完亮绿灯（PC7）
 */
void All_Connect(void)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET); // 灭绿灯

    HAL_UART_Receive_DMA(&huart2, (uint8_t *)ReceiveData, DMASIZE);
    HAL_Delay(2000);
    HAL_UART_Transmit(&huart2, (uint8_t *)KeyConnect, strlen(KeyConnect), HAL_MAX_DELAY); // 连接MQTT
    HAL_Delay(10);
    HAL_UART_Transmit(&huart2, (uint8_t *)BemfaConnect, strlen(BemfaConnect), HAL_MAX_DELAY); // 连接巴法云
    HAL_Delay(3500);
    HAL_UART_Transmit(&huart2, (uint8_t *)CipMode, strlen(CipMode), HAL_MAX_DELAY); // 设置透传
    HAL_Delay(10);
    for (int i = 0; i < 5; i++) // 重复订阅主题
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
        HAL_UART_Transmit(&huart2, (uint8_t *)Topic_FanControl_Connect, strlen(Topic_FanControl_Connect), HAL_MAX_DELAY);
        HAL_Delay(1000);
    }
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET); // 亮绿灯
}

/**
 * @brief 数据处理函数
 */
void Parse_MQTTData(void)
{
    char *p = ReceiveData;
    char *topicStart, *dataStart;
    int len = 0;
    int isFirstTopic = 1;
    int isFirstCmd = 1;

    ReceiveTopic[0] = '\0';
    ReceiveCmd[0] = '\0';

    while (*p != '\0')
    {
        // 查找"+MQTTSUBRECV:"
        if (strncmp(p, "+MQTTSUBRECV:", 13) != 0)
        {
            p++;
            continue;
        }
        p += 13; // 跳过"+MQTTSUBRECV:"

        // 跳过消息ID
        while (*p != ',' && *p != '\0')
            p++;
        if (*p == '\0')
            break;
        p++; // 跳过逗号

        // 提取主题
        if (*p != '"')
            break;
        topicStart = ++p;
        while (*p != '"' && *p != '\0')
            p++;
        if (*p == '\0')
            break;
        *p++ = '\0'; // 临时终止字符串

        // 检查主题是否已存在
        if (strstr(ReceiveTopic, topicStart) == NULL)
        {
            if (!isFirstTopic)
                strncat(ReceiveTopic, "|", 511 - strlen(ReceiveTopic));
            strncat(ReceiveTopic, topicStart, 511 - strlen(ReceiveTopic));
            isFirstTopic = 0;
        }
        *(p - 1) = '"'; // 恢复引号

        // 跳过逗号和长度字段
        while (*p != ',' && *p != '\0')
            p++;
        if (*p == '\0')
            break;
        p++;
        while (*p != ',' && *p != '\0')
            p++;
        if (*p == '\0')
            break;
        p++;

        // 提取数据
        dataStart = p;
        while (*p != '\r' && *p != '\n' && *p != '\0')
            p++;
        len = p - dataStart;

        if (!isFirstCmd)
            strncat(ReceiveCmd, "|", 511 - strlen(ReceiveCmd));
        isFirstCmd = 0;

        if (len > 0)
        {
            int remain = 511 - strlen(ReceiveCmd);
            strncat(ReceiveCmd, dataStart, len > remain ? remain : len);
        }

        // 跳过换行符
        while ((*p == '\r' || *p == '\n') && *p != '\0')
            p++;
    }
}

/**
 * @brief 接收数据函数
 */
void USART2_IRQHandler(void)
{
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE))
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart2);

        Parse_MQTTData(); // 解析数据

        if (strcmp(ReceiveTopic, "") != 0 && strcmp(ReceiveCmd, "") != 0)
        {
            // Ping检测
            if (strstr(ReceiveTopic, "Ping") != 0 && strstr(ReceiveCmd, "Ping") != 0)
            {
                HAL_UART_Transmit(&huart2, (uint8_t *)Topic_Ping_SendCmd, strlen(Topic_Ping_SendCmd), HAL_MAX_DELAY);
                Online_Status = 1;
            }
            if (strstr(ReceiveTopic, "CabinetLocation") != 0) // 开柜门识别
            {
                if (strstr(ReceiveCmd, "1") != 0) // 开柜门1
                {
                    Cabinet_List = 1;
                }
                else if (strstr(ReceiveCmd, "2") != 0) // 开柜门2
                {
                    Cabinet_List = 2;
                }
                else if (strstr(ReceiveCmd, "3") != 0) // 开柜门3
                {
                    Cabinet_List = 3;
                }
                else if (strstr(ReceiveCmd, "4") != 0) // 开柜门4
                {
                    Cabinet_List = 4;
                }
                else if (strstr(ReceiveCmd, "5") != 0) // 开柜门5
                {
                    Cabinet_List = 5;
                }
                HAL_UART_Transmit(&huart2, (uint8_t *)Topic_CabinetLocation_SendCmd, strlen(Topic_CabinetLocation_SendCmd), HAL_MAX_DELAY);
            }

            // RTC修改
            if (strstr(ReceiveTopic, "RTC") != 0)
            {
                if (ParseAndSetRTC(ReceiveCmd) == HAL_OK)
                {
                    Voice_Remind(Voice_Connect);
                }
            }

            // 设置闹钟
            if (strstr(ReceiveTopic, "MedicineReminder") != 0)
            {
                if (strstr(ReceiveCmd, "Clean") != 0)
                {
                    uint8_t WriteData = 0;
                    Flash_Erase(FLASH_BANK_1, 65, 1);
                    Flash_Write(0x08082000, (uint32_t)&WriteData);
                    Flash_Erase(FLASH_BANK_1, 66, 1);
                    HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A);
                }
                else if (strstr(ReceiveCmd, "MedicineOK") != 0)
                {
                    Medicine_Remind_Status = 2;
                    Voice_Remind(Voice_MedicineOK);
                }
                else
                {
                    Alarm_Operation();
                }
            }

            // 关闭柜子
            if (strstr(ReceiveTopic, "CloseCabinet") != 0)
            {
                Cabinet_List = 6;
                HAL_UART_Transmit(&huart2, (uint8_t *)Topic_CloseCabinet_SendCmd, strlen(Topic_CloseCabinet_SendCmd), HAL_MAX_DELAY);
            }

            // 制冷开关，E0为外部，B11为内部
            if (strstr(ReceiveTopic, "FanControl") != 0)
            {
                if (strstr(ReceiveCmd, "OutOn") != 0)
                {
                    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET);
                    Fan_Status = 1;
                }
                else if (strstr(ReceiveCmd, "OutOff") != 0)
                {
                    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_SET);
                    Fan_Status = 0;
                }
                else if (strstr(ReceiveCmd, "InOn") != 0)
                {
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
                    Fan_Status = 1;
                }
                else if (strstr(ReceiveCmd, "InOff") != 0)
                {
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
                    Fan_Status = 0;
                }
            }
            strcpy(ReceiveCmd, "");
            strcpy(ReceiveTopic, "");
        }
        memset(ReceiveData, 0, DMASIZE);
        HAL_UART_DMAStop(&huart2);
        HAL_UART_Receive_DMA(&huart2, (uint8_t *)ReceiveData, DMASIZE);
        HAL_UART_IRQHandler(&huart2);
    }
}

/**
 * @brief 语音提醒，使用USART3，每10秒播报一次
 * @param String 要播报的字符
 */
void Voice_Remind(const char *String)
{
    static uint8_t Medicine_Remind_Repeat = 0; ///< 服药重复提醒
    if (Voice_Status == 0)
    {
        if (Medicine_Remind_Status == 1 && Medicine_Remind_Repeat != 3)
        {
            Medicine_Remind_Repeat++;
            Uart_printf(&huart3, "%s", Voice_Reminder);
        }
        else if (Medicine_Remind_Status == 2)
        {
            Medicine_Remind_Repeat = 0;
            Medicine_Remind_Status = 0;
            Uart_printf(&huart3, "%s", Voice_MedicineOK);
        }
        else
        {
            Uart_printf(&huart3, "%s", String);
        }
        Voice_Status = 1;
        TIM5_Repeat = 0;
    }
}
