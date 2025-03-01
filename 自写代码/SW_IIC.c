/**
 *@file SW_IIC.c
 *@author 申飞麟
 *@brief 软件IIC配置
 *@date 2025/2/27
 */

#include "stm32f10x.h"
#include "Delay.h"

#define SCL(x)                                \
    do                                        \
    {                                         \
        GPIO_WriteBit(GPIOC, GPIO_Pin_13, x); \
        Delayus(10);                          \
    } while (0) // SCL操作

#define SDA(x)                                \
    do                                        \
    {                                         \
        GPIO_WriteBit(GPIOC, GPIO_Pin_14, x); \
        Delayus(10);                          \
    } while (0) // SDA操作

/**
 * @brief 读取SCL电平
 * @return 返回0或1
 */
uint8_t SCL_R(void)
{
    uint8_t Bit; ///< 读取值
    Bit = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13);
    Delayus(10);
    return Bit;
}

/**
 * @brief 读取SDA电平
 * @return 返回0或1
 */
uint8_t SDA_R(void)
{
    uint8_t Bit; ///< 读取值
    Bit = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_14);
    Delayus(10);
    return Bit;
}

/**
 * @brief IIC引脚初始化
 */
void IIC_Init(void)
{
    GPIO_InitTypeDef GPIOX_Init;
    GPIOX_Init.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14; // 默认PC13为SCL,P14为SDA
    GPIOX_Init.GPIO_Speed = GPIO_Speed_10MHz;
    GPIOX_Init.GPIO_Mode = GPIO_Mode_Out_OD; // 默认开漏输出
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    GPIO_Init(GPIOC, &GPIOX_Init);
    GPIO_SetBits(GPIOC, GPIO_Pin_13 | GPIO_Pin_14); // 默认上拉
}

/**
 * @brief IIC起始条件
 */
void IIC_Start(void)
{
    SDA(1);
    SCL(1);
    SDA(0);
    SCL(0);
}

/**
 * @brief IIC结束条件
 */
void IIC_End(void)
{
    SDA(0);
    SCL(1);
    SDA(1);
}

/**
 * @brief IIC发送字节
 * @param Byte 发送的字节
 */
void IIC_Send(uint8_t Byte)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        SDA(Byte & (0x80 >> i)); // 循环发送比特
        SCL(1);
        SCL(0);
    }
}

/**
 * @brief IIC读取字节
 * @return 返回读取的字节
 */
uint8_t IIC_Receive(void)
{
    uint8_t Byte = 0x00; ///< 读取的字节
    SDA(1);
    for (uint8_t i = 0; i < 8; i++)
    {
        SCL(1);
        if (SDA_R()) // 循环接收比特
        {
            Byte = (0x80 >> i);
        }
        SCL(0);
    }
    return Byte;
}

/**
 * @brief IIC发送请求
 * @param ACK 发送信号，1或0
 */
void IIC_SendACK(uint8_t ACK)
{
    SDA(ACK);
    SCL(1);
    SCL(0);
}

/**
 * @brief IIC发送请求
 * @return 接收信号
 */
uint8_t IIC_ReceiveACK(void)
{
    uint8_t ACK; ///< 接收请求信号
    SDA(1);
    SCL(1);
    ACK = SDA_R();
    SCL(0);
    return ACK;
}
