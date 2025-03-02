#include "stm32f10x.h"
#include "Delay.h"
#include "GPIOX_Init.h"
#include "OLED.h"
#include "SW_W25Q64.h"

int main(void)
{
    GPIOX_Init(GPIOB, RCC_APB2Periph_GPIOB, GPIO_Speed_50MHz, GPIO_Mode_AF_OD, GPIO_Pin_8 | GPIO_Pin_9); // OLED输出
    OLED_Init();

    SW_W25Q64_Init();
    uint8_t WriteArray[4] = {0x11, 0x22, 0x33, 0x44}; ///< 要写入的数据
    uint8_t ReadArray[4] = {0};                       ///< 读取的数据

    SW_W25Q64_SectorErase(0x100FFF); // 写入前判断是否擦除

    SW_W25Q64_Write(WriteArray, 0x100FFF, 4); // 写入
    SW_W25Q64_Read(ReadArray, 0x100F00, 4);   // 读取

    while (1)
    {
        OLED_ShowHexNum(1, 1, ReadArray[0], 2);
        OLED_ShowHexNum(1, 4, ReadArray[1], 2);
        OLED_ShowHexNum(1, 7, ReadArray[2], 2);
        OLED_ShowHexNum(1, 10, ReadArray[3], 2);
    }
}
