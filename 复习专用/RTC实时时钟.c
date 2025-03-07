/** 
 *  默认是LSE时钟，如果LSE不振动，则程序卡死
 *  显示时间是导入时间戳的方式
 */

#include "stm32f10x.h"
#include "Delay.h"
#include "GPIOX_Init.h"
#include "TIMX_Init.h"
#include "OLED.h"
#include "time.h"

uint16_t NowTime[6] = {0}; ///< 存储年月日时分秒
void TimeInit(time_t UNIXtime);
void ReadTime(void);

int main(void)
{
    GPIOX_Init(GPIOB, RCC_APB2Periph_GPIOB, GPIO_Speed_10MHz, GPIO_Mode_AF_OD, GPIO_Pin_8 | GPIO_Pin_9); // OLED输入
    OLED_Init();

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE); // 使能PWR和备份区域BKP时钟
    PWR_BackupAccessCmd(ENABLE);                                             // 设置PWR，使备份区域BKP可以访问，不设置则默认关闭

    if (BKP_ReadBackupRegister(BKP_DR1) != 0x0000) // 判断DR1是否被写入，不是则开始写入
    {
        RCC_LSEConfig(RCC_LSE_ON);                        // 开启LSE时钟
        while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) != SET) // 等待LSE开启，LSE损坏则可能导致卡死
            ;
        RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE); // 用LSE作为时钟
        RCC_RTCCLKCmd(ENABLE);                  // RTC使能

        RTC_WaitForSynchro();  // 等待RTC和APB1时钟同步
        RTC_WaitForLastTask(); // 等待写入完成

        RTC_SetPrescaler(32768 - 1); // 时钟设置为1s
        RTC_WaitForLastTask();       // 等待写入完成

        BKP_WriteBackupRegister(BKP_DR1, 0x0000); // 向DR1中写入0x6666，表示已经操作过RTC，下次不需要修改

        TimeInit(1741233720); // 此处导入时间戳，为2025年3月6日12:02:00
    }
    else
    {
        RTC_WaitForSynchro();  // 等待RTC和APB1时钟同步
        RTC_WaitForLastTask(); // 等待写入完成
    }
    while (1)
    {
        ReadTime(); // 连续读取秒数
        OLED_ShowNum(1, 1, NowTime[0], 4);
        OLED_ShowNum(1, 6, NowTime[1], 2);
        OLED_ShowNum(1, 9, NowTime[2], 2);
        OLED_ShowNum(3, 1, NowTime[3], 2);
        OLED_ShowNum(3, 4, NowTime[4], 2);
        OLED_ShowNum(3, 7, NowTime[5], 2);
    }
}

/**
 * @brief 初始化RTC时钟
 * @param UNIXtime 时间戳
 */
void TimeInit(time_t UNIXtime)
{
    RTC_SetCounter(UNIXtime);
    RTC_WaitForLastTask(); // 等待写入完成
}

/**
 * @brief 读取RTC时钟
 */
void ReadTime(void)
{
    struct tm *AllTime;                        ///< 时间数组
    time_t Timesec = (time_t)RTC_GetCounter(); ///< 从RTC获得的时钟
    AllTime = localtime(&Timesec);

    NowTime[0] = AllTime->tm_year + 1900;
    NowTime[1] = AllTime->tm_mon + 1;
    NowTime[2] = AllTime->tm_mday;
    NowTime[3] = (AllTime->tm_hour + 8) % 24;
    NowTime[4] = AllTime->tm_min;
    NowTime[5] = AllTime->tm_sec;
}
