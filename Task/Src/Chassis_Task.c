/**
* @file Chassis_Task.c
 * @brief 电机驱动任务
 * @date 2025/11/28
 */

#include "Chassis_Task.h"
#include "main.h"
#include "math.h"
#include "MG310.h"
#include "string.h"
#include "Debug_Tool.h"

CCMRAM_DATA Chassis_Instance_s CH_Instance = {
    CHASSIS_STOP, PS2_MODE, NONE_VISION,
    {0, 0, 0}, {0, 0, 0, 0}
};

extern Motor_Instance_s MG310[4];

/**
 * @brief 底盘任务函数
 */
CCMRAM_CODE void Chassis_Task(void)
{
    Chassis_Behavior();
    MG310_Drive();
}

/**
 * @brief 底盘行为函数
 * @note 此函数循环调用，涉及底盘运动学计算
 */
CCMRAM_CODE void Chassis_Behavior(void)
{
    switch (CH_Instance.Status) //底盘模式
    {
    case CHASSIS_OMNI: // 三轮模式
        CH_Instance.Speed_Set[0] = CH_Instance.Move.y
            + CH_Instance.Move.w;
        CH_Instance.Speed_Set[1] = sin(60 * PI / 180) * CH_Instance.Move.x
            - cos(60 * PI / 180) * CH_Instance.Move.y
            + CH_Instance.Move.w;
        CH_Instance.Speed_Set[2] = -sin(60 * PI / 180) * CH_Instance.Move.x
            - cos(60 * PI / 180) * CH_Instance.Move.y
            + CH_Instance.Move.w;
        break;
    case CHASSIS_MEC: // 麦克纳姆模式
        CH_Instance.Speed_Set[0] = -CH_Instance.Move.x
            - CH_Instance.Move.y
            + CH_Instance.Move.w;
        CH_Instance.Speed_Set[1] = CH_Instance.Move.x
            - CH_Instance.Move.y
            + CH_Instance.Move.w;
        CH_Instance.Speed_Set[2] = -CH_Instance.Move.x
            + CH_Instance.Move.y
            + CH_Instance.Move.w;
        CH_Instance.Speed_Set[3] = CH_Instance.Move.x
            + CH_Instance.Move.y
            + CH_Instance.Move.w;
        break;
    case CHASSIS_STOP: // 停止模式
        memset(CH_Instance.Speed_Set, 0, sizeof(CH_Instance.Speed_Set));
        break;
    default:
        CH_Instance.Status = CHASSIS_STOP;
        break;
    }
}

/**
 * @brief 定时器中断回调函数，计算电机速度并进行PID控制
 * @param htim 定时器句柄
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    if (htim == &GAP_TIM)
    {
        for (uint8_t i = 0; i < 4; i++)
        {
            MG310[i].Speed = MG310[i].Motor_GetSpeed(MG310[i].TIMx);
            MG310[i].PID_Output = PID_Calculate(&MG310[i].PID, MG310[i].Speed, CH_Instance.Speed_Set[i]);
        }
    }
}
