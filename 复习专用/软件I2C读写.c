/**
 * OELD输出时先初始化引脚再初始化OLED
 * 想要实现陀螺仪准确测量需要看寄存器手册
 * 注意等待函数的写法，容易出错
 */

#include "stm32f10x.h"
#include "Delay.h"
#include "GPIOX_Init.h"
#include "OLED.h"
#include "MPU6050.h"

void MPU_Write(uint8_t Address, uint8_t Data);
int8_t MPU_Read(uint8_t Address);
void MPU6050_WaitEvent(I2C_TypeDef *I2Cx, uint32_t I2C_EVENT);
void MPU6050_GetDatas(SensorData *Sensor);

int main(void)
{
    GPIOX_Init(GPIOB, RCC_APB2Periph_GPIOB, GPIO_Speed_50MHz, GPIO_Mode_AF_OD, GPIO_Pin_8 | GPIO_Pin_9); // OLED输出
    OLED_Init();
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);                                                   // I2C2使能
    GPIOX_Init(GPIOB, RCC_APB2Periph_GPIOB, GPIO_Speed_50MHz, GPIO_Mode_AF_OD, GPIO_Pin_10 | GPIO_Pin_11); // I2C引脚初始化

    I2C_InitTypeDef IIC; // I2C初始化
    IIC.I2C_Ack = I2C_Ack_Enable;
    IIC.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; // 7位地址
    IIC.I2C_ClockSpeed = 50000;                                 // 频率
    IIC.I2C_DutyCycle = I2C_DutyCycle_2;                        // 占空比
    IIC.I2C_Mode = I2C_Mode_I2C;                                // 正常I2C模式
    IIC.I2C_OwnAddress1 = 0x66;                                 // 本机地址，作为从机时有效
    I2C_Init(I2C2, &IIC);

    MPU6050_W(0x6B, 0x01); // 配置电源1并使用陀螺仪时钟
    MPU6050_W(0x6C, 0x00); // 配置电源2并唤醒
    MPU6050_W(0x19, 0x05); // 分频，可随意配
    MPU6050_W(0x1A, 0x09); // 滤波
    MPU6050_W(0x1B, 0x18); // 不自测，最大量程
    MPU6050_W(0x1C, 0x18); // 不自测，最大量程

    I2C_Cmd(I2C2, ENABLE);

    SensorData Sensor;
    while (1)
    {
        MPU6050_GetDatas(&Sensor);
        OLED_ShowSignedNum(1, 1, Sensor.ACCEL_X, 4);
        OLED_ShowSignedNum(2, 1, Sensor.ACCEL_Y, 4);
        OLED_ShowSignedNum(3, 1, Sensor.ACCEL_Z, 4);
        OLED_ShowSignedNum(1, 8, Sensor.GYRO_X, 4);
        OLED_ShowSignedNum(2, 8, Sensor.GYRO_Y, 4);
        OLED_ShowSignedNum(3, 8, Sensor.GYRO_Z, 4);
        Delayms(200);
    }
}

/**
 * @brief 硬件下MPU6050写入数据
 * @param Address 从机地址
 */
void MPU_Write(uint8_t Address, uint8_t Data)
{
    I2C_GenerateSTART(I2C2, ENABLE);
    if (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT))
    {
        Delayms(1);
    }
    I2C_Send7bitAddress(I2C2, 0xD0, I2C_Direction_Transmitter);
    if (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
    {
        Delayms(1);
    }
    I2C_SendData(I2C2, Address);
    if (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTING))
    {
        Delayms(1);
    }
    I2C_SendData(I2C2, Data);
    if (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    {
        Delayms(1);
    }
    I2C_GenerateSTOP(I2C2, ENABLE);
}

/**
 * @brief 硬件下MPU6050读取字节（注意等待事件顺序，若一次读取多个字节则等待事件不同）
 * @param Address 读取的地址
 * @return 读取的数据
 */
int8_t MPU_Read(uint8_t Address)
{
    int8_t Data;
    I2C_GenerateSTART(I2C2, ENABLE);
    MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT);

    I2C_Send7bitAddress(I2C2, 0xD0, I2C_Direction_Transmitter);
    MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);

    I2C_SendData(I2C2, Address);
    MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED);

    I2C_GenerateSTART(I2C2, ENABLE);
    MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT);

    I2C_Send7bitAddress(I2C2, 0xD0, I2C_Direction_Receiver);
    MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);

    I2C_AcknowledgeConfig(I2C2, DISABLE);
    I2C_GenerateSTOP(I2C2, ENABLE);

    MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED);
    Data = I2C_ReceiveData(I2C2);
    I2C_AcknowledgeConfig(I2C2, ENABLE);

    return Data;
}

/**
 * @brief 硬件下MPU6050等待事件（不是自己写的）
 * @param I2Cx 要等待的端口
 * @param I2C_EVENT 等待的时间
 */
void MPU6050_WaitEvent(I2C_TypeDef *I2Cx, uint32_t I2C_EVENT)
{
    uint32_t Timeout;
    Timeout = 10000;                                   // 给定超时计数时间
    while (I2C_CheckEvent(I2Cx, I2C_EVENT) != SUCCESS) // 循环等待指定事件
    {
        Timeout--;        // 等待时，计数值自减
        if (Timeout == 0) // 自减到0后，等待超时
        {
            /*超时的错误处理代码，可以添加到此处*/
            break; // 跳出等待，不等了
        }
    }
}

/**
 * @brief MPU6050获取数据，借用软件I2C，但是用的硬件读写
 * @param Sensor 传感器结构体
 */
void MPU6050_GetDatas(SensorData *Sensor)
{
    int16_t High, Low; // 分别接收高位和低位
    High = MPU_Read(0x3B) << 8;
    Low = MPU_Read(0x3C);
    Sensor->ACCEL_X = High | Low;

    High = MPU_Read(0x3D) << 8;
    Low = MPU_Read(0x3E);
    Sensor->ACCEL_Y = High | Low;

    High = MPU_Read(0x3F) << 8;
    Low = MPU_Read(0x40);
    Sensor->ACCEL_Z = High | Low;

    High = MPU_Read(0x43) << 8;
    Low = MPU_Read(0x44);
    Sensor->GYRO_X = High | Low;

    High = MPU_Read(0x45) << 8;
    Low = MPU_Read(0x46);
    Sensor->GYRO_Y = High | Low;

    High = MPU_Read(0x47) << 8;
    Low = MPU_Read(0x48);
    Sensor->GYRO_Z = High | Low;
}
