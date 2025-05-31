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
#define ClockWise 101             // 正转最大速
#define DClockWise 191            // 反转最大速
#define Stop 150                  // 停止
#define DelayTime 500             // 延时时间，可更改

char Read_Motor_Status[] = ""; ///< 读取Flash中的舵机状态位
char WriteOPEN[] = "O";     ///< 写入OPEN
char WriteCLOSE[] = "C";   ///< 写入CLOSE

/**
 * @brief 下方舵机旋转角度改变，一个柜子一个角度
 * @param Cabinet*List 柜子序号
 */
void Rotate_Motor_Move(uint8_t CabinetList)
{
    HAL_TIM_PWM_Start(&htim2, RotateMotor);
    if (CabinetList == 1) // 1号柜
    {
        __HAL_TIM_SET_COMPARE(&htim2, RotateMotor, 64);
    }
    else if (CabinetList == 2) // 2号柜
    {
        __HAL_TIM_SET_COMPARE(&htim2, RotateMotor, 107);
    }
    else if (CabinetList == 3) // 3号柜
    {
        __HAL_TIM_SET_COMPARE(&htim2, RotateMotor, 149);
    }
    else if (CabinetList == 4) // 4号柜
    {
        __HAL_TIM_SET_COMPARE(&htim2, RotateMotor, 188);
    }
    else if (CabinetList == 5) // 5号柜
    {
        __HAL_TIM_SET_COMPARE(&htim2, RotateMotor, 235);
    }
    HAL_Delay(2000);
}

/**
 * @brief 柜子抬升
 */
void Rotate_Motor_Up(void)
{
    HAL_TIM_PWM_Start(&htim2, UDMotor);
    __HAL_TIM_SET_COMPARE(&htim2, UDMotor, ClockWise);
    HAL_Delay(DelayTime);
    __HAL_TIM_SET_COMPARE(&htim2, UDMotor, Stop);
    HAL_Delay(2000);
}

/**
 * @brief 柜子下降
 */
void Rotate_Motor_Down(void)
{
    HAL_TIM_PWM_Start(&htim2, UDMotor);
    __HAL_TIM_SET_COMPARE(&htim2, UDMotor, DClockWise);
    HAL_Delay(DelayTime);
    __HAL_TIM_SET_COMPARE(&htim2, UDMotor, Stop);
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
        if (strcmp(Read_Motor_Status, "O") == 0)
        {
            Rotate_Motor_Down();
            Rotate_Motor_Move(*List);
            Rotate_Motor_Up();
        }
        else
        {
            Rotate_Motor_Move(*List);
            Rotate_Motor_Up();
            Flash_Erase(FLASH_BANK_1, 64, 1);
            Flash_Write(Motor_Status_ADDRESS, (uint32_t)WriteOPEN);
        }
    }
    else if (*List == 6)
    {
        if (strcmp(Read_Motor_Status, "O") == 0)
        {
            Rotate_Motor_Down();
        }
        Flash_Erase(FLASH_BANK_1, 64, 1);
        Flash_Write(Motor_Status_ADDRESS, (uint32_t)WriteCLOSE);
    }
    *List = 0;
}
