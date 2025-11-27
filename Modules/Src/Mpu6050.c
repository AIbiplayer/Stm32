/**
 * @file mpu6050.c
 * @brief 陀螺仪读取代码
 * @date 2025/10/8
 */

#include "Mpu6050.h"
#include "i2c.h"
#include "Mpu6050_Reg.h"
#include "stdbool.h"
#include "main.h"

#define MPU6050_ADDRESS		0xD0		//MPU6050的I2C从机地址

bool mpu6050_Error_Flag = false; //MPU6050错误标志

/**
 * @brief 读取MPU6050寄存器数据
 * @param RegAddress 寄存器地址
 * @return 读取到的数据
 */
uint8_t MPU6050_ReadReg(const uint8_t RegAddress)
{
    uint8_t Data;

    mpu6050_Error_Flag
        ? 0
        : HAL_I2C_Mem_Read(&hi2c1,MPU6050_ADDRESS, RegAddress,I2C_MEMADD_SIZE_8BIT, &Data, 1, 10) ==
        HAL_OK
        ? (mpu6050_Error_Flag = false)
        : (mpu6050_Error_Flag = true);

    return Data;
}

/**
 * @brief 向MPU6050寄存器写入数据
 * @param RegAddress 寄存器地址
 * @param Data 要写入的数据
 */
void MPU6050_WriteReg(const uint8_t RegAddress, uint8_t Data)
{
    mpu6050_Error_Flag
        ? 0
        : HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDRESS, RegAddress, I2C_MEMADD_SIZE_8BIT, &Data, 1, 10) ==
        HAL_OK
        ? (mpu6050_Error_Flag = false)
        : (mpu6050_Error_Flag = true);
}

/**
 * @brief 初始化MPU6050
 */
void MPU6050_Init(void)
{
    /*MPU6050寄存器初始化，需要对照MPU6050手册的寄存器描述配置，此处仅配置了部分重要的寄存器*/
    MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01); //电源管理寄存器1，取消睡眠模式，选择时钟源为X轴陀螺仪
    MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00); //电源管理寄存器2，保持默认值0，所有轴均不待机
    MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x09); //采样率分频寄存器，配置采样率
    MPU6050_WriteReg(MPU6050_CONFIG, 0x06); //配置寄存器，配置DLPF
    MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18); //陀螺仪配置寄存器，选择满量程为±2000°/s
    MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18); //加速度计配置寄存器，选择满量程为±16g
}

/**
 * @brief 读取MPU6050的加速度和陀螺仪数据
 * @param AccX 指向存储加速度X轴数据的变量的指针
 * @param AccY 指向存储加速度Y轴数据的变量的指针
 * @param AccZ 指向存储加速度Z轴数据的变量的指针
 * @param GyroX 指向存储陀螺仪X轴数据的变量的指针
 * @param GyroY 指向存储陀螺仪Y轴数据的变量的指针
 * @param GyroZ 指向存储陀螺仪Z轴数据的变量的指针
 */
void MPU6050_GetData(int16_t* AccX, int16_t* AccY, int16_t* AccZ,
                     int16_t* GyroX, int16_t* GyroY, int16_t* GyroZ)
{
    uint8_t DataH, DataL; //定义数据高8位和低8位的变量

    DataH = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H); //读取加速度计X轴的高8位数据
    DataL = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_L); //读取加速度计X轴的低8位数据
    *AccX = (DataH << 8) | DataL; //数据拼接，通过输出参数返回

    DataH = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_H); //读取加速度计Y轴的高8位数据
    DataL = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_L); //读取加速度计Y轴的低8位数据
    *AccY = (DataH << 8) | DataL; //数据拼接，通过输出参数返回

    DataH = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_H); //读取加速度计Z轴的高8位数据
    DataL = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_L); //读取加速度计Z轴的低8位数据
    *AccZ = (DataH << 8) | DataL; //数据拼接，通过输出参数返回

    DataH = MPU6050_ReadReg(MPU6050_GYRO_XOUT_H); //读取陀螺仪X轴的高8位数据
    DataL = MPU6050_ReadReg(MPU6050_GYRO_XOUT_L); //读取陀螺仪X轴的低8位数据
    *GyroX = (DataH << 8) | DataL; //数据拼接，通过输出参数返回

    DataH = MPU6050_ReadReg(MPU6050_GYRO_YOUT_H); //读取陀螺仪Y轴的高8位数据
    DataL = MPU6050_ReadReg(MPU6050_GYRO_YOUT_L); //读取陀螺仪Y轴的低8位数据
    *GyroY = (DataH << 8) | DataL; //数据拼接，通过输出参数返回

    DataH = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H); //读取陀螺仪Z轴的高8位数据
    DataL = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L); //读取陀螺仪Z轴的低8位数据
    *GyroZ = (DataH << 8) | DataL; //数据拼接，通过输出参数返回
}
