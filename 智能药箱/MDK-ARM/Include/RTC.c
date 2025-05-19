/**
 * @brief RTC操作
 * @date 2025/5/8
 */

#include "UART.h"
#include "string.h"
#include "stdio.h"
#include "main.h"

extern RTC_HandleTypeDef hrtc;

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

/**
 * @brief RTC闹钟响应
 */
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_SET); // 亮红灯
}
