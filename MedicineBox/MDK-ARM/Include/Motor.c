/**
 * @brief 下方旋转舵机控制（TIM2）
 * @date 2025/4/17
 */

#include "main.h"
#include "string.h"
#include "Flash.h"
#include "stm32u5xx_hal_tim.h"

extern TIM_HandleTypeDef htim2;

#define Motor_Status_ADDRESS 0x08080000 ///< Bank1的第64页地址，此页用于存放药柜开关状态

#define RotateMotor TIM_CHANNEL_1 // PA0接口
#define UDMotor TIM_CHANNEL_2     // PA1接口
#define DClockWise 20             // 正转角度
#define ClockWise 120             // 反转角度

static uint64_t Read_Motor_Status = 0; ///< 读取Flash中的舵机状态位，0为关闭
static uint8_t WriteOPEN = 1;
static uint8_t WriteCLOSE = 0;

/**
 * @brief 下方舵机旋转角度改变，一个柜子一个角度
 * @param Cabinet*List 柜子序号
 */
void Rotate_Motor_Move(uint8_t CabinetList)
{
    HAL_TIM_PWM_Start(&htim2, RotateMotor);
    if (CabinetList == 1) // 1号柜
    {
        __HAL_TIM_SET_COMPARE(&htim2, RotateMotor, 145);
    }
    else if (CabinetList == 2) // 2号柜
    {
        __HAL_TIM_SET_COMPARE(&htim2, RotateMotor, 110);
    }
    else if (CabinetList == 3) // 3号柜
    {
        __HAL_TIM_SET_COMPARE(&htim2, RotateMotor, 70);
    }
    else if (CabinetList == 4) // 4号柜
    {
        __HAL_TIM_SET_COMPARE(&htim2, RotateMotor, 0);
    }
    else if (CabinetList == 5) // 5号柜
    {
        __HAL_TIM_SET_COMPARE(&htim2, RotateMotor, 210);
    }
    HAL_Delay(2000);
}

/**
 * @brief 柜子抬升
 */
void Rotate_Motor_Up(void)
{
    HAL_TIM_PWM_Start(&htim2, UDMotor);
    __HAL_TIM_SET_COMPARE(&htim2, UDMotor, 50 + ClockWise * 10 / 9);
    HAL_Delay(2000);
}

/**
 * @brief 柜子下降
 */
void Rotate_Motor_Down(void)
{
    HAL_TIM_PWM_Start(&htim2, UDMotor);
    __HAL_TIM_SET_COMPARE(&htim2, UDMotor, 50 + DClockWise * 10 / 9);
    HAL_Delay(2000);
}

/**
 * @brief 操作舵机函数
 * @param *List 柜子序号
 */
void Motor_Operation(uint8_t *List)
{
    Flash_Read((uint64_t *)&Read_Motor_Status, Motor_Status_ADDRESS); // 通过读取Flash判断柜门状态
    if (*List >= 1 && *List <= 5)
    {
        if (Read_Motor_Status) // 开启状态
        {
            Rotate_Motor_Down();
            Rotate_Motor_Move(*List);
            Rotate_Motor_Up();
        }
        else // 关闭状态
        {
            Rotate_Motor_Move(*List);
            Rotate_Motor_Up();
            Flash_Erase(FLASH_BANK_1, 64, 1);
            Flash_Write(Motor_Status_ADDRESS, (uint32_t)&WriteOPEN);
        }
    }
    else if (*List == 6)
    {
        if (Read_Motor_Status)
        {
            Rotate_Motor_Down();
        }
        Flash_Erase(FLASH_BANK_1, 64, 1);
        Flash_Write(Motor_Status_ADDRESS, (uint32_t)&WriteCLOSE);
    }
    *List = 0;
}
