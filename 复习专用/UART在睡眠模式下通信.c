/**
 * 睡眠模式下，默认是中断唤醒。CPU不运行，但引脚功能正常。
 * 中断唤醒后先执行中断函数，再执行睡眠模式后的代码。
 * 如果想执行类似传感器采集的工作，可以把WFI放在循环里。
 * 
 * 睡眠模式是开启时钟，而其它模式下时钟关闭，意味着内部中断无法唤醒。
 * 由于USART是内部中断（用了RCC使能），所以其它模式下无法使用
 */

#include "stm32f10x.h"
#include "Delay.h"
#include "GPIOX_Init.h"
#include "OLED.h"
#include "NVIC_Init.h"

int main(void)
{
    GPIOX_Init(GPIOB, RCC_APB2Periph_GPIOB, GPIO_Speed_10MHz, GPIO_Mode_AF_OD, GPIO_Pin_8 | GPIO_Pin_9); // OLED显示
    GPIOX_Init(GPIOA, RCC_APB2Periph_GPIOA, GPIO_Speed_10MHz, GPIO_Mode_IN_FLOATING, GPIO_Pin_10);       // UART引脚OLED_Init();
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    NV_Init(USART1_IRQn, 1, 1, 2); // UART1开启中断
    OLED_Init();

    USART_InitTypeDef U1;
    U1.USART_BaudRate = 9600;                                      // 波特率9600
    U1.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 硬件无控制流
    U1.USART_Mode = USART_Mode_Rx;                                 // 接收模式
    U1.USART_Parity = USART_Parity_No;                             // 无校验
    U1.USART_StopBits = USART_StopBits_1;                          // 1位停止位
    U1.USART_WordLength = USART_WordLength_8b;                     // 8字节长度
    USART_Init(USART1, &U1);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART1, ENABLE);
    while (1)
    {
        __WFI(); // 进入睡眠模式，有中断则退出
        OLED_ShowString(2, 1, "OK");
        Delayms(200);
        OLED_ShowString(2, 1, "  ");
    }
}

void USART1_IRQHandler(void)
{
    if (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == SET)
    {
        USART_ClearFlag(USART1, USART_FLAG_RXNE);
    }
    OLED_ShowNum(1, 1, USART_ReceiveData(USART1), 2);

    USART_ClearITPendingBit(USART1, USART_IT_RXNE);
}
