#ifndef UART_H
#define UART_H

typedef enum ESP_Status
{
    WiFi_OK = 0,    ///< WiFi连接成功
    MQTT_OK = 1,    ///< MQTT连接成功
    Bemfa_OK = 2,   ///< 巴法云连接成功
    CipMode_OK = 3, ///< 透传设置成功
    Topic_OK = 4,   ///< 主题订阅成功
    ESP_ERROR = 5,  ///< 失败
    Online_OK = 6   ///< 联网成功
} ESP_Status;

uint8_t All_Connect(void);
uint8_t WaitForACK(void);
uint8_t Connect_WiFi(void);
uint8_t Connect_MQTT(void);
uint8_t Connect_Bemfa(void);
uint8_t Send_Cipmode(void);
uint8_t Connect_Topic(void);
void Parse_MQTTData(void);
void Operation(void);

#endif