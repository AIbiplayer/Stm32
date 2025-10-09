/**
 * @file Gimbal.c
 * @brief 云台控制模块
 * @date 2025/10/8
*/

#include "Gimbal.h"
#include "tim.h"
#include "math.h"
#include "mpu6050.h"
#include "ShowShape.h"

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
 * @brief 云台初始化
 * @note 启动PWM并设置舵机初始位置
 */
void Gimbal_Init(void)
{
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // 启动偏航舵机PWM
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); // 启动
    Yaw_SetAngle(90.0f); // 设置偏航舵机初始位置
    Pitch_SetAngle(90.0f); // 设置俯仰舵机初始位置
}

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
