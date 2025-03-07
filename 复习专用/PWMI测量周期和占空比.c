/**
 *PWMI模式和输入捕获模式不同的地方就是，比如TIM3的通道1配置为输入模式，则通道2自动配置相反的模式
 *如CH1是上升沿计数，CH2就是下降沿计数。从模式为复位模式，意味着CH2是下降沿时停止计数，但直到CH1停止
 *计数时才会被复位。
 *比如CH1计了100次，CH2计了50次，则占空比为50%
 *注意复制粘贴时别忘了改数字
 */

#include "stm32f10x.h"
#include "Delay.h"
#include "GPIOX_Init.h"
#include "TIMX_Init.h"
#include "OLED.h"

int main(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); // 先配置时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    GPIOX_Init(GPIOB, RCC_APB2Periph_GPIOB, GPIO_Speed_50MHz, GPIO_Mode_AF_OD, GPIO_Pin_8 | GPIO_Pin_9); // OLED输出
    OLED_Init();
    GPIOX_Init(GPIOA, RCC_APB2Periph_GPIOA, GPIO_Speed_10MHz, GPIO_Mode_AF_PP, GPIO_Pin_0);       // PA0作为输出比较（TIM2的CH1）
    GPIOX_Init(GPIOA, RCC_APB2Periph_GPIOA, GPIO_Speed_10MHz, GPIO_Mode_IN_FLOATING, GPIO_Pin_6); // PA6作为输入捕获（TIM3的CH1）

    TIMX_Init(RCC_APB1Periph_TIM2, TIM2, 3600 - 1, TIM_CounterMode_Up, 40 - 1); // TIM2频率20kHZ,周期为2ms
    TIMX_Init(RCC_APB1Periph_TIM3, TIM3, 72 - 1, TIM_CounterMode_Up, 65535);    // TIM3频率为1MHz

    TIM_OCInitTypeDef T2; // 配置TIM2为PWM模式
    TIM_OCStructInit(&T2);
    T2.TIM_OCMode = TIM_OCMode_PWM1;             // PWM1模式
    T2.TIM_OutputState = TIM_OutputState_Enable; // PWM使能开启
    T2.TIM_Pulse = 20;                           // 占空比为50
    T2.TIM_OCPolarity = TIM_OCPolarity_High;     // 输出为高电平
    TIM_OC1Init(TIM2, &T2);

    TIM_ICInitTypeDef T3;                          // 配置TIM3为PWM输入模式
    T3.TIM_Channel = TIM_Channel_1;                // 通道1,2输入
    T3.TIM_ICPolarity = TIM_ICPolarity_Rising;     // 上升沿计数
    T3.TIM_ICSelection = TIM_ICSelection_DirectTI; // 直接计数，不交换
    T3.TIM_ICPrescaler = TIM_ICPSC_DIV1;           // 不分频
    T3.TIM_ICFilter = 0xF;                         // 滤波
    TIM_PWMIConfig(TIM3, &T3);

    TIM_SelectInputTrigger(TIM3, TIM_TS_TI1FP1);
    TIM_SelectSlaveMode(TIM3, TIM_SlaveMode_Reset);

    TIM_Cmd(TIM2, ENABLE);
    TIM_Cmd(TIM3, ENABLE);

    while (1)
    {
        OLED_ShowNum(1, 1, 1000000 / TIM_GetCapture1(TIM3), 6);
        OLED_ShowString(1, 8, "Hz");
        OLED_ShowNum(2, 1, 100 * TIM_GetCapture2(TIM3) / TIM_GetCapture1(TIM3), 2);
        OLED_ShowString(2, 3, "%%");
    }
}
