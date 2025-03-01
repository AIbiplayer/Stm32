/**
 *@file MPU6050.c
 *@author 申飞麟
 *@brief MPU6050操作函数（软件）, 默认没有IIC_Init()
 *@date 2025/2/27
 */

#include "SW_IIC.h"
#include "MPU6050.h"

// 软件I2C下要查看手册以配置寄存器
#define MPU6050_SMPLRT_DIV 0x19   // 数据输出的快慢，值越小越快
#define MPU6050_CONFIG 0x1A       // 配置寄存器设置
#define MPU6050_GYRO_CONFIG 0x1B  // 陀螺仪设置
#define MPU6050_ACCEL_CONFIG 0x1C // 加速度器设置
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_ACCEL_XOUT_L 0x3C
#define MPU6050_ACCEL_YOUT_H 0x3D
#define MPU6050_ACCEL_YOUT_L 0x3E
#define MPU6050_ACCEL_ZOUT_H 0x3F
#define MPU6050_ACCEL_ZOUT_L 0x40
#define MPU6050_TEMP_OUT_H 0x41 // 温度测量
#define MPU6050_TEMP_OUT_L 0x42
#define MPU6050_GYRO_XOUT_H 0x43
#define MPU6050_GYRO_XOUT_L 0x44
#define MPU6050_GYRO_YOUT_H 0x45
#define MPU6050_GYRO_YOUT_L 0x46
#define MPU6050_GYRO_ZOUT_H 0x47
#define MPU6050_GYRO_ZOUT_L 0x48
#define MPU6050_PWR_MGMT_1 0x6B // 电源设置
#define MPU6050_PWR_MGMT_2 0x6C
#define MPU6050_WHO_AM_I 0x75 // MPU地址

/**
 * @brief 向MPU6050写入字节
 * @param Address 从机寄存器地址
 * @param Data 写入的数据
 */
void MPU6050_W(uint8_t Address, uint8_t Data)
{
    IIC_Start();
    IIC_Send(0xD0);   // 初始地址0xD0
    IIC_ReceiveACK(); // 这里省略了判断是否收到ACK
    IIC_Send(Address);
    IIC_ReceiveACK(); // 这里省略了判断是否收到ACK
    IIC_Send(Data);   // 默认写一个字节，多写则重复这两段
    IIC_ReceiveACK(); // 这里省略了判断是否收到ACK
    IIC_End();
}

/**
 * @brief 向MPU6050读取字节，如果想连续读则导入数组指针
 * @param Address 从机寄存器地址
 */
uint8_t MPU6050_R(uint8_t Address)
{
    uint8_t Data; ///< 接收字节
    IIC_Start();
    IIC_Send(0xD0);   // 初始地址0xD0
    IIC_ReceiveACK(); // 这里省略了判断是否收到ACK
    IIC_Send(Address);
    IIC_ReceiveACK(); // 这里省略了判断是否收到ACK

    IIC_Start();
    IIC_Send(0xD0 | 0x01); // 转为读模式
    IIC_ReceiveACK();
    Data = IIC_Receive(); // 默认写一个字节，多写则重复这两段，并且SendACK(0)
    IIC_SendACK(1);       // 停止接收
    IIC_End();
    return Data;
}

/**
 * @brief MPU6050初始化
 */
void MPU6050_Init(void)
{
    IIC_Init();
    MPU6050_W(MPU6050_PWR_MGMT_1, 0x01);   // 配置电源1并使用陀螺仪时钟
    MPU6050_W(MPU6050_PWR_MGMT_2, 0x00);   // 配置电源2并唤醒
    MPU6050_W(MPU6050_SMPLRT_DIV, 0x05);   // 分频，可随意配
    MPU6050_W(MPU6050_CONFIG, 0x06);       // 滤波
    MPU6050_W(MPU6050_GYRO_CONFIG, 0x18);  // 不自测，最大量程
    MPU6050_W(MPU6050_ACCEL_CONFIG, 0x18); // 不自测，最大量程
    MPU6050_W(MPU6050_CONFIG, 0x06);
}

/**
 * @brief MPU6050获取数据
 * @param Sensor 传感器结构体
 */
void MPU6050_GetData(SensorData *Sensor)
{
    int16_t High, Low; // 分别接收高位和低位
    High = MPU6050_R(MPU6050_ACCEL_XOUT_H) << 8;
    Low = MPU6050_R(MPU6050_ACCEL_XOUT_L);
    Sensor->ACCEL_X = High | Low;

    High = MPU6050_R(MPU6050_ACCEL_YOUT_H) << 8;
    Low = MPU6050_R(MPU6050_ACCEL_YOUT_L);
    Sensor->ACCEL_Y = High | Low;

    High = MPU6050_R(MPU6050_ACCEL_ZOUT_H) << 8;
    Low = MPU6050_R(MPU6050_ACCEL_ZOUT_L);
    Sensor->ACCEL_Z = High | Low;

    High = MPU6050_R(MPU6050_GYRO_XOUT_H) << 8;
    Low = MPU6050_R(MPU6050_GYRO_XOUT_L);
    Sensor->GYRO_X = High | Low;

    High = MPU6050_R(MPU6050_GYRO_YOUT_H) << 8;
    Low = MPU6050_R(MPU6050_GYRO_YOUT_L);
    Sensor->GYRO_Y = High | Low;

    High = MPU6050_R(MPU6050_GYRO_ZOUT_H) << 8;
    Low = MPU6050_R(MPU6050_GYRO_ZOUT_L);
    Sensor->GYRO_Z = High | Low;
}
