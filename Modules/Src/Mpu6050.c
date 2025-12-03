/**
 * @file mpu6050.c
 * @brief 陀螺仪读取代码
 * @date 2025/10/8
 */

#include "Mpu6050.h"
#include "i2c.h"
#include "Mpu6050_Reg.h"
#include "stdbool.h"
#include "math.h"
#include "main.h"

#define MPU6050_ADDRESS		0xD0		//MPU6050的I2C从机地址

bool mpu6050_Error_Flag = false; //MPU6050错误标志

static float Q[2][2] = {{0.0025, 0}, {0, 0.0025}}; //误差协方差
static float R[2][2] = {{0.3, 0}, {0, 0.3}}; //测量误差协方差
static float Z[2]; //加速度计测量值
static float X_hat_p[2]; //X_hat_p = {roll,pitch}
static float X_hat[2] = {0, 0};
static float P_e[2][2] = {{1, 0}, {0, 1}};
static float P_e_p[2][2];
static float K[2][2];
static float B[2][2] = {{0.005, 0}, {0, 0.005}}; //采样时间设为∆t = 5ms
static float U[2];

double GZ_bias = 0; // 陀螺仪零偏
float dt = 0.01; // 采样时间
double roll, yaw, pitch;
double pitch_, roll_; // 角度滤波后数据
int16_t AX, AY, AZ, GX, GY, GZ; // MPU6050原始数据
double yawlast = 0;


/**
 * @brief 卡尔曼滤波函数
 * @note 该函数从MPU6050获取数据并进行卡尔曼滤波计算
 */
void KalmanFilter(void)
{
    MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
    double GZ_corrected = GZ - GZ_bias;
    roll = atan2(AY * 6.1035 * 0.00001, AZ * 6.1035 * 0.00001) * 3.1415927f * 180.0f;
    pitch = atan2(AX * 6.1035 * 0.00001, AZ * 6.1035 * 0.00001) * 3.1415927f * 180.0f;
    pitch_ = pitch / 800 * 90; //线性映射，角度：-90-90
    roll_ = roll / 800 * 90;
    /*********************卡尔曼滤波*******************************/
    U[0] = GY * 6.1035 * 0.01; //pitch
    U[1] = GX * 6.1035 * 0.01; //roll
    yaw = yaw + GZ_corrected * 6.1035 * 0.005 * 0.01;
    Z[0] = roll;
    Z[1] = pitch;
    //1.计算先验估计
    X_hat_p[0] = X_hat[0] + U[0] * B[0][0];
    X_hat_p[1] = X_hat[1] + U[1] * B[1][1];
    //2.计算先验误差协方差
    P_e_p[0][0] = P_e[0][0] + Q[0][0];
    P_e_p[0][1] = 0;
    P_e_p[1][0] = 0;
    P_e_p[1][1] = P_e[1][1] + Q[1][1];
    //3.计算卡尔曼增益
    K[0][0] = P_e_p[0][0] / (P_e_p[0][0] + R[0][0]);
    K[0][1] = 0;
    K[1][0] = 0;
    K[1][1] = P_e_p[1][1] / (P_e_p[1][1] + R[1][1]);
    //4.计算后验估计
    X_hat[0] = X_hat_p[0] + K[0][0] * (Z[0] - X_hat_p[0]);
    X_hat[1] = X_hat_p[1] + K[1][1] * (Z[1] - X_hat_p[1]);
    //5.更新误差协方差
    P_e[0][0] = (1 - K[0][0]) * P_e_p[0][0];
    P_e[1][1] = (1 - K[1][1]) * P_e_p[1][1];
    HAL_Delay(5);
}

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
