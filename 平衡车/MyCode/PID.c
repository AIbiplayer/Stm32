#include "PID.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "stdint.h"
#include "string.h"
#include "stm32f1xx_hal_tim.h"
#include "stm32f1xx.h"

char RxData[100] = {0}; ///< 接收的数据

float Balance_Angle = 0.5f; ///< 平衡时小车偏移角度
float Pitch, Yaw, Roll;     ///< 当前角度
short ACCX, ACCY, ACCZ;     ///< 加速度
short GRX, GRY, GRZ;        ///< 角速度

float KS_P = 2.1f;      ///< 直立P环，0~10
float KS_D = 0.012f;    ///< 直立D环，0~1
float KV_P = -0.00025f; ///< 速度P环，0~1

float KV_I = 0.0f; ///< 速度I环

float KT_P = 0.0f;             ///< 转向P环
float KT_D = 0.0f;             ///< 转向D环
int32_t Enco_L = 0;            ///< 左编码器数值
int32_t Enco_R = 0;            ///< 右编码器数值
int32_t EX_Speed = 0;          ///< 期望速度
int32_t EX_Turn = 0;           ///< 预期转向角度
int32_t PD_Out, PI_Out, T_Out; ///< 输出的控制器值
int32_t M_1, M_2;              ///< 电机PWM值

extern TIM_HandleTypeDef htim2, htim3, htim4;
extern UART_HandleTypeDef huart3;

/**
 * @brief 驱动电机
 * @param M1 左电机
 * @param M2 右电机
 */
void Load_Motor(int32_t M1, int32_t M2)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET); // STBY启动

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

    if (M1 < 0)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); // 向前方向控制
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
        M1 = -M1;
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, M1);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // 向后方向控制
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, M1);
    }

    if (M2 < 0)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET); // 向前方向控制
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
        M2 = -M2;
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, M2);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET); // 向后方向控制
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, M2);
    }
}

/**
 * @brief 读取速度值
 * @param htim 定时器
 */
int32_t Read_Speed(TIM_HandleTypeDef *htim)
{
    int32_t ReadData = 0;
    ReadData = __HAL_TIM_GetCounter(htim);
    __HAL_TIM_SetCounter(htim, 0);
    return ReadData;
}

/**
 * @brief 直立环PD控制
 * @param Ex_Angle 期望角度
 * @param Angle 当前角度
 * @param Gyro 角速度
 */
float PD_Control(float Ex_Angle, float Angle, float Gyro)
{
    return KS_P * (Angle - Ex_Angle) + KS_D * Gyro;
}

/**
 * @brief 速度环PI控制
 * @param EX_Vercity 期望速度
 * @param Encoder_L 左编码器计数值
 * @param Encoder_R 右编码器计数值
 */
float PI_Control(int32_t EX_Vercity, int32_t Encoder_L, int32_t Encoder_R)
{
    int32_t DIF = 0;                ///< 偏差值
    int32_t DIF_OutPut = 0;         ///< 输出的偏差值
    static float Fliter = 0.7f;     ///< 滤波
    static int32_t DIF_Last = 0;    ///< 最后一次输出的偏差值
    static int32_t Encoder_Sum = 0; ///< 积分

    // 计算偏差+滤波
    DIF = (Encoder_L + Encoder_R) - EX_Vercity;
    DIF_OutPut = (1 - Fliter) * DIF + Fliter * DIF_Last;
    DIF_Last = DIF_OutPut;

    KV_I = KV_P / 200.0f;

    // 计算积分
    Encoder_Sum += DIF_OutPut;

    // 积分限制幅度
    if (Encoder_Sum > 2000)
        Encoder_Sum = 2000;
    else if (Encoder_Sum < -2000)
        Encoder_Sum = -2000;

    // 如果蓝牙接收到停止，则在这里写入停止代码
    if (0)
    {
        Encoder_Sum = 0;
    }
    return KV_P * DIF_OutPut + KV_I * Encoder_Sum;
}

/**
 * @brief 转向环PD控制
 * @param Ex_Angle 期望转向角度
 * @param Gyro 当前角速度
 */
float T_Control(int32_t Ex_Angle, float Gyro)
{
    return KT_P * Ex_Angle + KT_D * Gyro;
}

/**
 * @brief 控制小车PWM
 */
void Control(void)
{
    int32_t PWM_Out = 0; ///< 用于输出PWM的值

    Enco_L = Read_Speed(&htim3);
    Enco_R = -Read_Speed(&htim4);
    mpu_dmp_get_data(&Pitch, &Roll, &Yaw);
    MPU_Get_Gyroscope(&GRX, &GRY, &GRZ);
    MPU_Get_Accelerometer(&ACCX, &ACCY, &ACCZ);

    PI_Out = PI_Control(EX_Speed, Enco_L, Enco_R);

    PD_Out = PD_Control(PI_Out, Pitch, GRY); // 根据不同车辆来

    T_Out = T_Control(EX_Turn, GRZ);

    PWM_Out = PD_Out;
    M_1 = PWM_Out - T_Out;
    M_2 = PWM_Out + T_Out;
    Load_Motor(M_1, M_2);
}

void Parse_Bluetooth_Data(char *data)
{
    char *ptr = data;
    while (*ptr != '\0')
    {
        // 1. 提取前缀（如 "SKD"）
        char prefix[4] = {0};
        for (int i = 0; i < 3 && *ptr != '\0'; i++)
        {
            prefix[i] = *ptr++;
        }
        prefix[3] = '\0';

        // 2. 检查是否为参数行（前缀必须是SKD/SKP/TKD/TKP/VKI/VKP）
        if ((prefix[0] == 'S' || prefix[0] == 'T' || prefix[0] == 'V') &&
            (prefix[1] == 'K' || prefix[1] == 'P' || prefix[1] == 'D' || prefix[1] == 'I'))
        {

            // 3. 检查是否有负号标记 '#'
            int is_negative = 0;
            if (*ptr == '#')
            {
                is_negative = 1;
                ptr++; // 跳过 '#'
            }

            // 4. 提取数值部分
            float value = 0.0f;
            float decimal_place = 0.1f;
            int has_decimal = 0;

            while (*ptr != '\0' && *ptr != '\n')
            {
                if (*ptr == '.')
                {
                    has_decimal = 1;
                }
                else if (*ptr >= '0' && *ptr <= '9')
                {
                    if (has_decimal)
                    {
                        value += (*ptr - '0') * decimal_place;
                        decimal_place *= 0.1f;
                    }
                    else
                    {
                        value = value * 10.0f + (*ptr - '0');
                    }
                }
                ptr++;
            }

            // 5. 处理负值
            if (is_negative)
            {
                value = -value;
            }

            // 6. 更新对应的PID参数
            if (strcmp(prefix, "SKD") == 0)
            { // 注意：这里用strcmp仅作对比，实际可替换为逐字符比较
                KS_D = value;
            }
            else if (strcmp(prefix, "SKP") == 0)
            {
                KS_P = value;
            }
            else if (strcmp(prefix, "TKD") == 0)
            {
                KT_D = value;
            }
            else if (strcmp(prefix, "TKP") == 0)
            {
                KT_P = value;
            }
            else if (strcmp(prefix, "VKI") == 0)
            {
                KV_I = value;
            }
            else if (strcmp(prefix, "VKP") == 0)
            {
                KV_P = value;
            }
        }

        // 7. 移动到下一行
        while (*ptr != '\0' && *ptr != '\n')
        {
            ptr++;
        }
        if (*ptr == '\n')
        {
            ptr++;
        }
    }
}

void USART3_IRQHandler(void)
{
    /* USER CODE BEGIN USART1_IRQn 0 */
    if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE))
    {
        HAL_UART_DMAStop(&huart3);

        Parse_Bluetooth_Data(RxData);

        HAL_UART_Receive_DMA(&huart3, (uint8_t *)RxData, 100);

        __HAL_UART_CLEAR_FLAG(&huart3, UART_FLAG_IDLE);
    }
    /* USER CODE END USART1_IRQn 0 */
    HAL_UART_IRQHandler(&huart3);
    /* USER CODE BEGIN USART1_IRQn 1 */

    /* USER CODE END USART1_IRQn 1 */
}

