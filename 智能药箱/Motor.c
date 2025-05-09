/**
 * @brief 下方旋转舵机控制（TIM2）
 * @date 2025/4/17
 */

#include "main.h"
#include "stm32u5xx_hal_tim.h"

extern TIM_HandleTypeDef htim2;

#define RotateMotor TIM_CHANNEL_1 // PA0接口

/**
 * @brief 角度改变，一个柜子一个角度
 * @param CabinetList 柜子序号
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
}
