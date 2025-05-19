/**
 * @brief RTC操作
 * @date 2025/5/8
 */

#ifndef RTC_H
#define RTC_H

HAL_StatusTypeDef ParseAndSetRTC(char *ReceiveCmd);
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc);

#endif
