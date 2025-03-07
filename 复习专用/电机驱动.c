/**
 *注意电机驱动模块的接线顺序，VM接5V（不要接单片机的5V），VCC接3.3V，AO1,AO2接电机两边
 *PWMA接信号口，AIN1和AIN2随便接，用高低电平翻转和AO1,AO2一同控制转向
 *STBY是休眠，默认接3.3V
 *频率建议在1K~20KHz之间
 *AIN1和AIN2注意在延时前切换
 */

#include "stm32f10x.h"
#include "Delay.h"
#include "GPIOX_Init.h"

void ChangeSpeed(uint8_t Speed);

int main(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    GPIOX_Init(GPIOA, RCC_APB2Periph_GPIOA, GPIO_Speed_10MHz, GPIO_Mode_Out_PP, GPIO_Pin_2 | GPIO_Pin_3); // PA2,PA3来控制转向，引脚可随意
    GPIOX_Init(GPIOA, RCC_APB2Periph_GPIOA, GPIO_Speed_10MHz, GPIO_Mode_AF_PP, GPIO_Pin_1);               // PA1作为PWM输出

    TIM_TimeBaseInitTypeDef TXInit;              // 初始化TIM2
    TXInit.TIM_Prescaler = 7200 - 1;             // 频率为10000HZ
    TXInit.TIM_CounterMode = TIM_CounterMode_Up; // 高电平
    TXInit.TIM_Period = 100 - 1;                 // 周期为10ms
    TXInit.TIM_ClockDivision = TIM_CKD_DIV1;     // 不分频
    TXInit.TIM_RepetitionCounter = 0;            // 高级定时器专用
    TIM_TimeBaseInit(TIM2, &TXInit);

    TIM_OCInitTypeDef T2P; // 初始化TIM2的输出模式
    TIM_OCStructInit(&T2P);
    T2P.TIM_OCMode = TIM_OCMode_PWM1; // PWM1模式
    T2P.TIM_OCPolarity = TIM_OCPolarity_High;
    T2P.TIM_OutputState = TIM_OutputState_Enable;
    T2P.TIM_Pulse = 0;       // 初始时电机不转，转速范围0到100，由占空比决定
    TIM_OC2Init(TIM2, &T2P); // 开启通道2，也就是PA1

    TIM_Cmd(TIM2, ENABLE);

    Delays(1);

    GPIO_SetBits(GPIOA, GPIO_Pin_2); // 方向为反方向
    GPIO_ResetBits(GPIOA, GPIO_Pin_3);
    ChangeSpeed(30);
    Delays(1);
    ChangeSpeed(50);
    Delays(1);
    ChangeSpeed(0);
    GPIO_SetBits(GPIOA, GPIO_Pin_3); // 方向为正方向，注意在延时前切换
    GPIO_ResetBits(GPIOA, GPIO_Pin_2);
    Delays(1);

    ChangeSpeed(50);
    Delays(1);
    while (1)
    {
        ChangeSpeed(0);
    }
}

void ChangeSpeed(uint8_t Speed)
{
    TIM_SetCompare2(TIM2, Speed);
}
