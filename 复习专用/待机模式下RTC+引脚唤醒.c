/** 
 *  待机模式下，只能通过四种模式唤醒。唤醒后复位，因此时钟频率不变
 *  要把待机模式放到最后，不然后面的程序都执行不了
 *  OLED初始化时自动清屏
 *  
 *  Wake Up 引脚是PA0口，不需要再初始化，默认是低电平。只有高电平才能唤醒
 *  待机前加入结束程序的代码，不然会一直运行，达不到最大省电
 */

#include "stm32f10x.h"
#include "Delay.h"
#include "GPIOX_Init.h"
#include "OLED.h"
#include "NVIC_Init.h"

int main(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE); // 加载PWR和BKP时钟
    PWR_BackupAccessCmd(ENABLE);                                             // 开启BKP区域
    PWR_WakeUpPinCmd(ENABLE);                                                // 开启WakeUp引脚唤醒
    OLED_Init();

    if (BKP_ReadBackupRegister(BKP_DR1) != 0x6666)
    {
        RCC_LSEConfig(RCC_LSE_ON);                        // 开启LSE时钟
        while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) != SET) // 等待完全开启，可能卡死
            ;
        RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
        RCC_RTCCLKCmd(ENABLE); // 开启RTC时钟

        RTC_WaitForSynchro();
        RTC_WaitForLastTask();

        RTC_SetPrescaler(32768 - 1);
        RTC_WaitForLastTask();

        BKP_WriteBackupRegister(BKP_DR1, 0x6666);
        RTC_ExitConfigMode();
    }
    else
    {
        RTC_WaitForSynchro();
        RTC_WaitForLastTask();
    }

    RTC_SetAlarm(RTC_GetCounter() + 5);//每次复位时都重新设置5s的时钟

    while (1)
    {
        OLED_ShowString(1, 1, "Wake Up");
        Delayms(1000);
        OLED_ShowString(1, 1, "          ");
        /**
         * 想要达到最大省电，可以在这里加入终止某程序的代码，比如OLED清屏，或关闭电机
         */
        PWR_EnterSTANDBYMode(); // 进入待机模式，后面的程序都不会执行
    }
}
