/**
 *  独立看门狗先初始化OLED，再判断复位情况
 *  具体喂狗时间要计算得出
 *  需要循环喂狗
 */

#include "stm32f10x.h"
#include "Delay.h"
#include "GPIOX_Init.h"
#include "OLED.h"

int main(void)
{
    GPIOX_Init(GPIOA, RCC_APB2Periph_GPIOA, GPIO_Speed_10MHz, GPIO_Mode_IPU, GPIO_Pin_1);
    OLED_Init();

    if (RCC_GetFlagStatus(RCC_FLAG_IWDGRST) == SET) // 判断是否是看门狗复位
    {
        OLED_ShowString(1, 1, "WWDG");
        Delayms(500);
        OLED_ShowString(1, 1, "        ");
        RCC_ClearFlag();
    }
    else
    {
        OLED_ShowString(1, 1, "RESET");
        Delayms(500);
        OLED_ShowString(1, 1, "        ");
    }
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG, ENABLE); // 开启看门狗时钟
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);        // 开启对看门狗的写使能
    IWDG_SetPrescaler(IWDG_Prescaler_16);                // 16分频，最大1600ms
    RCC_LSICmd(ENABLE);                                  // 开启LSI时钟并等待
    while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) != SET)
        ;

    IWDG_SetReload(0x0FFF); // 最大值喂狗
    IWDG_Enable();          // 开启看门狗
    while (1)
    {
        IWDG_ReloadCounter();
        while (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1) == SET)
            ;
    }
}
