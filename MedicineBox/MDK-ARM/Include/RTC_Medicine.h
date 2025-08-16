/**
 * @brief RTC操作
 * @date 2025/5/8
 */

#ifndef RTC_MEDICINE_H
#define RTC_MEDICINE_H

typedef struct RTCAlarm_Remind // 定义RTC闹钟结构体
{
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
} RTCAlarm_Remind;

void Alarm_Operation(void);
HAL_StatusTypeDef ParseAndSetRTC(char *ReceiveCmd);
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc);

#endif
