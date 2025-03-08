/**
 *  窗口看门狗设置最快时间时要参考手册并计算
 *  注意Delay的位置，位置不对会产生复位
 */

#include "stm32f10x.h"
#include "Delay.h"
#include "GPIOX_Init.h"
#include "OLED.h"

int main(void)
{
    GPIOX_Init(GPIOA, RCC_APB2Periph_GPIOA, GPIO_Speed_10MHz, GPIO_Mode_IPU, GPIO_Pin_1);
    OLED_Init();

    if (RCC_GetFlagStatus(RCC_FLAG_WWDGRST) == SET) // 判断是否是看门狗复位
    {
        OLED_ShowString(1, 1, "WWDG");
        Delayms(50);
        OLED_ShowString(1, 1, "        ");
        RCC_ClearFlag();
    }
    else
    {
        OLED_ShowString(1, 1, "RESET");
        Delayms(50);
        OLED_ShowString(1, 1, "        ");
    }
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG, ENABLE); // 开启看门狗时钟
    WWDG_SetWindowValue(0x40 | 21);                      // 最快值为30ms，小于30ms时喂狗则导致复位
    WWDG_SetPrescaler(WWDG_Prescaler_8);                 // 8分频
    RCC_LSICmd(ENABLE);                                  // 开启LSI时钟并等待
    while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) != SET)
        ;
    WWDG_Enable(0x40 | 54); // 开启看门狗，并设定初始值为50ms

    while (1)
    {
        Delayms(40);
        while (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1) == SET)
            ;
        WWDG_SetCounter(0x40 | 54); // 持续喂狗，喂狗值为50ms
    }
}
