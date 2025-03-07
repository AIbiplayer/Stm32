/** 停止模式下，引脚功能依然正常。这里为低功耗模式，唤醒时间稍长
 *  只能由外部中断EXTI唤醒，其它中断均无效
 *  先开启PWR时钟，再进入停止模式
 *
 *  停止模式唤醒后不复位，继续运行
 *
 *  注意唤醒后时钟频率由72MHz变为8MHz，需要重新初始化系统
 */

#include "stm32f10x.h"
#include "Delay.h"
#include "GPIOX_Init.h"
#include "OLED.h"
#include "NVIC_Init.h"
#include "EXTI_Init.h"

int main(void)
{
    GPIOX_Init(GPIOB, RCC_APB2Periph_GPIOB, GPIO_Speed_10MHz, GPIO_Mode_AF_OD, GPIO_Pin_8 | GPIO_Pin_9); // OLED显示
    GPIOX_Init(GPIOA, RCC_APB2Periph_GPIOA, GPIO_Speed_10MHz, GPIO_Mode_IPU, GPIO_Pin_11);               // 上拉输入
    NV_Init(EXTI15_10_IRQn, 1, 1, 2);                                                                    // 中断开启
    EXT_Init(EXTI_Line11, EXTI_Mode_Interrupt, EXTI_Trigger_Rising);                                     // 触发外部中断
    OLED_Init();
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE); // 进入PWR以开始停止模式

    PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI); // 进入停止模式，低功耗模式下唤醒时间长。用中断唤醒

    while (1)
    {
        OLED_ShowString(2, 1, "Test");
    }
}

void EXTI15_10_IRQHandler(void)
{
    if (EXTI_GetFlagStatus(EXTI_Line11) == SET)
    {
        SystemInit(); // 重新启动时钟
        OLED_ShowString(2, 1, "OK!");
        Delayms(500);
        OLED_ShowString(2, 1, "  ");
        EXTI_ClearFlag(EXTI_Line11);
    }
    EXTI_ClearITPendingBit(EXTI_Line11);
}
