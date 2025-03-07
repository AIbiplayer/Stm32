/**
 *硬件控制SPI和软件控制SPI的不同就是数据交换的时候，所以把交换函数移植一下完全可以兼容
 *注意CS引脚的GPIO口是推挽输出而不是复用推挽输出
 *正常情况下，SPI初始化时NSS为软件管理
 */

#include "stm32f10x.h"
#include "Delay.h"
#include "GPIOX_Init.h"
#include "OLED.h"

void SwapInformation(uint8_t *SendArray, uint8_t *ReceiveArray, uint8_t BothCount);
uint32_t GetID(void);

int main(void)
{
    GPIOX_Init(GPIOB, RCC_APB2Periph_GPIOB, GPIO_Speed_50MHz, GPIO_Mode_AF_OD, GPIO_Pin_8 | GPIO_Pin_9); // OLED输出
    OLED_Init();

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
    GPIOX_Init(GPIOA, RCC_APB2Periph_GPIOA, GPIO_Speed_10MHz, GPIO_Mode_Out_PP, GPIO_Pin_4);             // 无论软硬件，CS口为复用输出，否则出错
    GPIOX_Init(GPIOA, RCC_APB2Periph_GPIOA, GPIO_Speed_10MHz, GPIO_Mode_AF_PP, GPIO_Pin_5 | GPIO_Pin_7); // 软件下为复用输出，硬件为推挽复用输出
    GPIOX_Init(GPIOA, RCC_APB2Periph_GPIOA, GPIO_Speed_10MHz, GPIO_Mode_IPU, GPIO_Pin_6);                // MISO为上拉输入，A6

    SPI_InitTypeDef SPI;                                   // 配置SPI，非连续模式
    SPI.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128; // 128分频，分频越低越快，但是非连续读取下速度受限
    SPI.SPI_CPHA = SPI_CPHA_1Edge;                         // 默认模式0
    SPI.SPI_CPOL = SPI_CPOL_Low;                           // 默认模式0
    SPI.SPI_CRCPolynomial = 7;                             // CRC校验
    SPI.SPI_DataSize = SPI_DataSize_8b;                    // 8位数据
    SPI.SPI_Direction = SPI_Direction_2Lines_FullDuplex;   // 全双工模式，两根线都用
    SPI.SPI_FirstBit = SPI_FirstBit_MSB;                   // 高位先行模式
    SPI.SPI_Mode = SPI_Mode_Master;                        // 主机模式
    SPI.SPI_NSS = SPI_NSS_Soft;                            // 软件管理NSS，如果不用NSS，则必须由软件管理
    SPI_Init(SPI1, &SPI);

    SPI_Cmd(SPI1, ENABLE);
    GPIO_SetBits(GPIOA, GPIO_Pin_4); // 初始化时CS置高位，不通信

    while (1)
    {
        OLED_ShowHexNum(1, 1, GetID(), 8);
    }
}

/**
 * @brief 硬件SPI交换数据（发送和接收的数组长度必须相同）
 * @param SendArray 发送的数据数组
 * @param ReceiveArray 接收的数据数组
 * @param Count 发送的数组长度，不得超过256
 */
void SwapInformation(uint8_t *SendArray, uint8_t *ReceiveArray, uint8_t BothCount)
{
    GPIO_ResetBits(GPIOA, GPIO_Pin_4); // CS置低位，通信
    uint16_t DelayTime;                ///< 延迟变量，用于等待
    for (uint8_t i = 0; i < BothCount; i++)
    {
        DelayTime = 50000;
        while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) != SET) // 等待TXE
        {
            DelayTime--;
            if (!DelayTime)
            {
                break;
            }
        }
        SPI_I2S_SendData(SPI1, SendArray[i]); // 发送数据
        DelayTime = 50000;
        while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) != SET) // 等待RXNE
        {

            DelayTime--;
            if (!DelayTime)
            {
                break;
            }
        }
        ReceiveArray[i] = SPI_I2S_ReceiveData(SPI1); // 接收数据
    }
    GPIO_SetBits(GPIOA, GPIO_Pin_4); // CS置高位，不通信
}

/**
 * @brief 硬件SPI获取ID
 * @return ID号
 */
uint32_t GetID(void)
{
    uint8_t IDArray[4] = {0, 1, 2, 3};
    uint8_t SendArray[4] = {0x9F, 0xFF, 0xFF, 0xFF};
    SwapInformation(SendArray, IDArray, 4);
    uint32_t ID = 0;
    ID |= IDArray[3];
    ID |= IDArray[2] << 8;
    ID |= IDArray[1] << 16;
    return ID;
}
