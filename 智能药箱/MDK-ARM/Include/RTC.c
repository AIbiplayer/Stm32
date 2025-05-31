/**
 * @brief RTC操作
 * @date 2025/5/8
 */

#include "UART.h"
#include "stdlib.h"
#include "string.h"
#include "stdio.h"
#include "main.h"
#include "RTC.h"
#include "Flash.h"

#define RTC_Alarm_ADDRESS 0x08084000       ///< Bank1的第66页地址，此页用于存放服药闹钟提醒
#define RTC_AlarmNumber_ADDRESS 0x08082000 ///< Bank1的第65页地址，此地址用于存放闹钟的个数

extern RTC_HandleTypeDef hrtc;

// 服药提醒语音
const char Voice_Reminder[] = {0xB7, 0xFE, 0xD2, 0xA9, 0xCA, 0xB1, 0xBC, 0xE4, 0xB5, 0xBD, 0xA3, 0xAC, 0xC7, 0xEB, 0xC1, 0xA2, 0xBC, 0xB4, 0xB7, 0xFE, 0xD2, 0xA9, 0x00};
static uint8_t FlashAlarm_Number = 0; ///< 从Flash中找到闹钟的个数

/**
 * @brief 从ReceiveCmd解析日期时间并设置RTC
 * @param ReceiveCmd 输入字符串，格式为"yyyy/MM/dd HH:mm:ss"
 * @return HAL_OK成功，HAL_ERROR失败
 */
HAL_StatusTypeDef ParseAndSetRTC(char *ReceiveCmd)
{
    uint8_t month, day, hour, minute, second;
    uint16_t year;
    RTC_DateTypeDef sDate = {0};
    RTC_TimeTypeDef sTime = {0};

    if (sscanf(ReceiveCmd, "%hu/%hhu/%hhu %hhu:%hhu:%hhu", &year, &month, &day, &hour, &minute, &second) != 6)
    {
        return HAL_ERROR; // 解析失败
    }

    // 设置RTC日期
    sDate.Year = year - 2000; // STM32 RTC年份从2000开始计数
    sDate.Month = month;
    sDate.Date = day;

    if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
    {
        return HAL_ERROR;
    }

    // 设置RTC时间
    sTime.Hours = hour;
    sTime.Minutes = minute;
    sTime.Seconds = second;
    sTime.SubSeconds = 0;
    sTime.TimeFormat = RTC_HOURFORMAT_24;
    sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    sTime.StoreOperation = RTC_STOREOPERATION_RESET;

    if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
    {
        return HAL_ERROR;
    }
    return HAL_OK;
}

extern char ReceiveCmd[512];

/**
 * @brief 闹钟解析函数
 * @param Remind 闹钟数组
 * @return 返回闹钟个数
 */
uint8_t ParseRemindTimes(RTCAlarm_Remind *Remind)
{
    uint8_t count = 0;
    char *token = strtok(ReceiveCmd, "|");

    while (token != NULL && count < 20)
    {
        // 尝试解析格式为"yyyy-MM-dd/H:mm"的时间
        RTCAlarm_Remind newAlarm;
        if (sscanf(token, "%hu-%hhu-%hhu/%hhu:%hhu",
                   &newAlarm.year, &newAlarm.month, &newAlarm.day,
                   &newAlarm.hour, &newAlarm.minute) == 5)
        {
            // 验证时间有效性
            if (newAlarm.month >= 1 && newAlarm.month <= 12 &&
                newAlarm.day >= 1 && newAlarm.day <= 31 &&
                newAlarm.hour <= 23 && newAlarm.minute <= 59)
            {
                Remind[count++] = newAlarm;
            }
        }
        token = strtok(NULL, "|");
    }
    return count;
}

/**
 * @brief 闹钟操作主函数
 */
void Alarm_Operation(void)
{
    static RTCAlarm_Remind Remind[20]; ///< 每次最多增加20个闹钟
    uint8_t NewAlarm_Number = 0;       ///< 新闹钟数量

    Flash_Read((uint64_t *)&FlashAlarm_Number, RTC_AlarmNumber_ADDRESS); // 读取Flash中的闹钟个数

    NewAlarm_Number = ParseRemindTimes(Remind);

    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};
    HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

    // 把每一个大于当前时间的闹钟保存到闹钟数组里
    while (NewAlarm_Number)
    {
        if (Remind[NewAlarm_Number - 1].year > (sDate.Year + 2000) ||
            (Remind[NewAlarm_Number - 1].year == (sDate.Year + 2000) && Remind[NewAlarm_Number - 1].month > sDate.Month) ||
            (Remind[NewAlarm_Number - 1].year == (sDate.Year + 2000) && Remind[NewAlarm_Number - 1].month == sDate.Month && Remind[NewAlarm_Number - 1].day > sDate.Date) ||
            (Remind[NewAlarm_Number - 1].year == (sDate.Year + 2000) && Remind[NewAlarm_Number - 1].month == sDate.Month && Remind[NewAlarm_Number - 1].day == sDate.Date &&
             Remind[NewAlarm_Number - 1].hour > sTime.Hours) ||
            (Remind[NewAlarm_Number - 1].year == (sDate.Year + 2000) && Remind[NewAlarm_Number - 1].month == sDate.Month && Remind[NewAlarm_Number - 1].day == sDate.Date &&
             Remind[NewAlarm_Number - 1].hour == sTime.Hours && Remind[NewAlarm_Number - 1].minute > sTime.Minutes))
        {
            Flash_Write(RTC_Alarm_ADDRESS + (FlashAlarm_Number * 0x10), (uint32_t)&Remind[NewAlarm_Number - 1]);
            FlashAlarm_Number++;
        }
        NewAlarm_Number--;
    }
    Flash_Erase(FLASH_BANK_1, 65, 1);
    Flash_Write(RTC_AlarmNumber_ADDRESS, (uint32_t)&FlashAlarm_Number); // 更新闹钟数量

    // 以下为更新闹钟
    RTCAlarm_Remind AlarmFind = {0};
    RTCAlarm_Remind AlarmFindLast = {0};
    AlarmFindLast.year = 2100;
    AlarmFindLast.month = 12;
    AlarmFindLast.day = 31;
    AlarmFindLast.hour = 23;
    AlarmFindLast.minute = 59;

    for (uint8_t FindList = 0; FindList < FlashAlarm_Number; FindList++) // 轮询Flash中的闹钟，找到最接近的闹钟并设置
    {
        Flash_Read((uint64_t *)&AlarmFind, RTC_Alarm_ADDRESS + (FindList * 0x10));
        // 先判断大于当前时间，再判断最近闹钟
        if ((AlarmFind.year > (sDate.Year + 2000) ||
             (AlarmFind.year == (sDate.Year + 2000) && AlarmFind.month > sDate.Month) ||
             (AlarmFind.year == (sDate.Year + 2000) && AlarmFind.month == sDate.Month && AlarmFind.day > sDate.Date) ||
             (AlarmFind.year == (sDate.Year + 2000) && AlarmFind.month == sDate.Month && AlarmFind.day == sDate.Date &&
              AlarmFind.hour > sTime.Hours) ||
             (AlarmFind.year == (sDate.Year + 2000) && AlarmFind.month == sDate.Month && AlarmFind.day == sDate.Date &&
              AlarmFind.hour == sTime.Hours && AlarmFind.minute > sTime.Minutes)))
        {
            if ((AlarmFind.year < AlarmFindLast.year) ||
                (AlarmFind.year == AlarmFindLast.year && AlarmFind.month < AlarmFindLast.month) ||
                (AlarmFind.year == AlarmFindLast.year && AlarmFind.month == AlarmFindLast.month && AlarmFind.day < AlarmFindLast.day) ||
                (AlarmFind.year == AlarmFindLast.year && AlarmFind.month == AlarmFindLast.month && AlarmFind.day == AlarmFindLast.day && AlarmFind.hour < AlarmFindLast.hour) ||
                (AlarmFind.year == AlarmFindLast.year && AlarmFind.month == AlarmFindLast.month && AlarmFind.day == AlarmFindLast.day && AlarmFind.hour == AlarmFindLast.hour && AlarmFind.minute < AlarmFindLast.minute))
            {
                AlarmFindLast = AlarmFind;
            }
        }
    }
    RTC_AlarmTypeDef sAlarm = {0};
    sAlarm.Alarm = RTC_ALARM_A;
    sAlarm.AlarmTime.Hours = AlarmFindLast.hour;
    sAlarm.AlarmTime.Minutes = AlarmFindLast.minute;
    sAlarm.FlagAutoClr = ALARM_FLAG_AUTOCLR_ENABLE;
    sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
    sAlarm.AlarmDateWeekDay = AlarmFindLast.day;
    sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
    HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN);
}

/**
 * @brief RTC闹钟响应并语音播报
 */
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc1)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
    Voice_Remind(Voice_Reminder);

    // 以下为更新闹钟
    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};
    HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

    RTCAlarm_Remind AlarmFind = {0};
    RTCAlarm_Remind AlarmFindLast = {0};
    AlarmFindLast.year = 2100;
    AlarmFindLast.month = 12;
    AlarmFindLast.day = 31;
    AlarmFindLast.hour = 23;
    AlarmFindLast.minute = 59;

    for (uint8_t FindList = 0; FindList < FlashAlarm_Number; FindList++) // 轮询Flash中的闹钟，找到最接近的闹钟并设置
    {
        Flash_Read((uint64_t *)&AlarmFind, RTC_Alarm_ADDRESS + (FindList * 0x10));
        // 先判断大于当前时间，再判断最近闹钟
        if ((AlarmFind.year > (sDate.Year + 2000) ||
             (AlarmFind.year == (sDate.Year + 2000) && AlarmFind.month > sDate.Month) ||
             (AlarmFind.year == (sDate.Year + 2000) && AlarmFind.month == sDate.Month && AlarmFind.day > sDate.Date) ||
             (AlarmFind.year == (sDate.Year + 2000) && AlarmFind.month == sDate.Month && AlarmFind.day == sDate.Date &&
              AlarmFind.hour > sTime.Hours) ||
             (AlarmFind.year == (sDate.Year + 2000) && AlarmFind.month == sDate.Month && AlarmFind.day == sDate.Date &&
              AlarmFind.hour == sTime.Hours && AlarmFind.minute > sTime.Minutes)))
        {
            if ((AlarmFind.year < AlarmFindLast.year) ||
                (AlarmFind.year == AlarmFindLast.year && AlarmFind.month < AlarmFindLast.month) ||
                (AlarmFind.year == AlarmFindLast.year && AlarmFind.month == AlarmFindLast.month && AlarmFind.day < AlarmFindLast.day) ||
                (AlarmFind.year == AlarmFindLast.year && AlarmFind.month == AlarmFindLast.month && AlarmFind.day == AlarmFindLast.day && AlarmFind.hour < AlarmFindLast.hour) ||
                (AlarmFind.year == AlarmFindLast.year && AlarmFind.month == AlarmFindLast.month && AlarmFind.day == AlarmFindLast.day && AlarmFind.hour == AlarmFindLast.hour && AlarmFind.minute < AlarmFindLast.minute))
            {
                AlarmFindLast = AlarmFind;
            }
        }
    }
    RTC_AlarmTypeDef sAlarm = {0};
    sAlarm.Alarm = RTC_ALARM_A;
    sAlarm.AlarmTime.Hours = AlarmFindLast.hour;
    sAlarm.AlarmTime.Minutes = AlarmFindLast.minute;
    sAlarm.FlagAutoClr = ALARM_FLAG_AUTOCLR_ENABLE;
    sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
    sAlarm.AlarmDateWeekDay = AlarmFindLast.day;
    sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
    HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN);
}
