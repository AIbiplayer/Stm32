/**
 * @brief 下方旋转舵机控制
 * @date 2025/4/17
 */

#include "main.h"
#include "Flash.h"
#include "stm32u5xx_hal_tim.h"

extern TIM_HandleTypeDef htim2;

uint8_t While_Cabinet_List[6] = {1, 2, 3, 4, 5, 6};

#define Cabinet_Address 0x08080000                       // 柜子数据存储在Flash中
#define TestAngle 345                                    // 5V时舵机角速度，可改变
#define RotateMotor TIM_CHANNEL_1                        // PA0
#define Stop(Mode) __HAL_TIM_SetCompare(&htim2, Mode, 0) // 停止

void ClockWise(void) // 顺时针
{
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 25);
}

void DClockWise(void) // 逆时针
{
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 5);
}

/**
 * @brief 角度改变
 * @param Angle 角度
 */
uint16_t Angle_Change(int16_t Angle)
{
    return Angle * 1000 / TestAngle;
}

void Read_Cabinet(uint8_t *Cabinet)
{
    uint64_t Data = 0;
    Flash_Read(&Data, Cabinet_Address);
    for (uint8_t i = 0; i < 6; i++)
    {
        Cabinet[i] = Data >> (8 * (5 - i));
    }
}

/**
 * @brief 旋转舵机控制，信号线是PA0
 * @param Cabinet_Num 目标柜子序号，范围1~6
 */
void Rotate_Motor_Move(uint8_t Cabinet_Num)
{

    uint8_t Now_Cabinet = While_Cabinet_List[0]; // 当前序号
    HAL_TIM_PWM_Start(&htim2, RotateMotor);

    if (Cabinet_Num - Now_Cabinet == 3 || Cabinet_Num - Now_Cabinet == -3) // 柜子对立面
    {
        ClockWise();
        HAL_Delay(Angle_Change(180));
        Stop(RotateMotor);
    }
    // 柜子左2
    else if (((Now_Cabinet != 5 || Now_Cabinet != 6) && (Cabinet_Num - Now_Cabinet == 2)) || (Now_Cabinet == 5 && Cabinet_Num == 1 || Now_Cabinet == 6 && Cabinet_Num == 2))
    {
        ClockWise();
        HAL_Delay(Angle_Change(120));
        Stop(RotateMotor);
    }
    // 柜子右2
    else if ((Cabinet_Num - Now_Cabinet == 4 || Cabinet_Num - Now_Cabinet == -4) || (Now_Cabinet == 3 && Cabinet_Num == 1 || Now_Cabinet == 4 && Cabinet_Num == 2))
    {
        DClockWise();
        HAL_Delay(Angle_Change(120));
        Stop(RotateMotor);
    }
    // 柜子右1
    else if (Cabinet_Num - Now_Cabinet == -1 || Cabinet_Num - Now_Cabinet == 5)
    {
        DClockWise();
        HAL_Delay(Angle_Change(60));
        Stop(RotateMotor);
    }
    // 柜子左1
    else if (Cabinet_Num - Now_Cabinet == 1 || Cabinet_Num - Now_Cabinet == -5)
    {
        ClockWise();
        HAL_Delay(Angle_Change(60));
        Stop(RotateMotor);
    }
    // 停止
    else if (Cabinet_Num == Now_Cabinet)
    {
        Stop(RotateMotor);
    }
}
