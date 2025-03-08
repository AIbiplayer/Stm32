/**
 * 清除闪存中的页数时，不要清除前面几页，因为前面保存的是程序代码，清除就卡死了。
 * 建议在最后几页写入数据，防止程序卡死。
 * 如果想让写入的数据不因为重新烧录而清除（Keil5默认主闪存全部擦除并烧录），那么需要设置写保护
 *
 * 设置写保护注意事项：
 * 1、不要对主闪存前几页设置写保护，不然烧录时会因为烧录到保护区而终止烧录
 * 2、对不想擦除的几页设置写保护。而且要有解除保护的代码，不然无法烧录。
 * 3、如果设置了写保护，在Keil5中设置烧录擦除为部分擦除，这样程序可以烧录，但保护的部分无法进行任何操作，页擦除也无效
 * 4、想真正解除写保护，用ST-Link Utility 或者 FlyMcu 进行解除
 * 5、FlyMcu 需要配置USART1接口并用USB转TTL烧录，并重新接跳线帽使BootLoader0为1，复位。烧录后重置跳线帽，复位
 *
 * 可以在 Keil5 中设置烧录占用的范围，避免程序和存储冲突
 */

#include "stm32f10x.h"
#include "Delay.h"
#include "GPIOX_Init.h"
#include "OLED.h"

/**
 * 详细操作可以写各种函数，比如读函数，写函数等
 */
int main(void)
{
    OLED_Init();

    FLASH_Unlock();              // 解锁闪存
    FLASH_ErasePage(0x0801FC00); // 擦除 0x0801FC00 的内容，也就是第127页
    while (FLASH_GetFlagStatus(FLASH_FLAG_BSY) == SET)
        ;                                      // 等待写入
    FLASH_ProgramHalfWord(0x0801FC00, 0x1234); // 在 0x0801FC00 中写入半字的内容
    while (FLASH_GetFlagStatus(FLASH_FLAG_BSY) == SET)
        ;         // 等待写入
    FLASH_Lock(); // 锁住闪存

    /*  FlyMcu复位专用：
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
   GPIOX_Init(GPIOA, RCC_APB2Periph_GPIOA, GPIO_Speed_50MHz, GPIO_Mode_AF_PP, GPIO_Pin_9); // TX
   GPIOX_Init(GPIOA, RCC_APB2Periph_GPIOA, GPIO_Speed_50MHz, GPIO_Mode_IPU, GPIO_Pin_10);  // RX

   USART_InitTypeDef UInit1;
   UInit1.USART_BaudRate = 9600;
   UInit1.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
   UInit1.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
   UInit1.USART_Parity = USART_Parity_No;
   UInit1.USART_StopBits = USART_StopBits_1;
   UInit1.USART_WordLength = USART_WordLength_8b;
   USART_Init(USART1, &UInit1);

   USART_Cmd(USART1, ENABLE);*/

    uint16_t ReadPoint = *((__IO uint16_t *)(0x0801FC00)); // 读取0x0801FC00下的数据，并禁止优化寄存器
    uint32_t Capacity = *((__IO uint32_t *)(0x1FFFF7E0));  // 读取容量
    uint32_t ID = *((__IO uint32_t *)(0x1FFFF7E8));        // 读取身份标识的后32位

    while (1)
    {
        OLED_ShowHexNum(1, 1, ReadPoint, 8);
        OLED_ShowHexNum(2, 1, Capacity, 8);
        OLED_ShowHexNum(3, 1, ID, 8);
    }
}
