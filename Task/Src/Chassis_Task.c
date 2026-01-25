/**
* @file Chassis_Task.c
 * @brief 电机驱动任务
 * @date 2025/11/28
 */

#include "Chassis_Task.h"
#include "main.h"
#include "math.h"
#include "MG310.h"
#include "Debug_Tool.h"

CCMRAM_DATA Chassis_Instance_s CH_Instance = {
    CHASSIS_MEC, BLUETOOTH_MODE,
    {0, 0, 0}, {0, 0, 0, 0}
};

extern PID_Typedef yaw_pid, pitch_pid;
extern Motor_Instance_s MG310[4];

float target_angle, current_angle;
float target_angle, current_angle;
float pid_output = 0.0f;

float SP1 = 0;
float SP2 = 0;
float SP3 = 0;
float SP4 = 0;

/**
 * @brief 底盘任务函数
 */
void Chassis_Task(void) {
    Chassis_Behavior();
    MG310_Drive();
    SP1 = MG310[0].PID.Output;
    SP2 = MG310[1].PID.Output;
    SP3 = MG310[2].PID.Output;
    SP4 = MG310[3].PID.Output;
}

/**
 * @brief 底盘行为函数
 * @note 此函数循环调用，涉及底盘运动学计算
 */
void Chassis_Behavior(void) {
    switch (CH_Instance.Status) //底盘模式
    {
        case CHASSIS_OMNI_TRI: // 三轮模式
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
            CH_Instance.Speed_Set[1] = -CH_Instance.Move.x
                                       + CH_Instance.Move.y
                                       + CH_Instance.Move.w;
            CH_Instance.Speed_Set[2] = -CH_Instance.Move.x
                                       - CH_Instance.Move.y
                                       - CH_Instance.Move.w;
            CH_Instance.Speed_Set[3] = -CH_Instance.Move.x
                                       + CH_Instance.Move.y
                                       - CH_Instance.Move.w;
            break;
        case CHASSIS_OMNI_SQU: // 四轮全向轮模式
            CH_Instance.Speed_Set[0] = CH_Instance.Move.x + CH_Instance.Move.y + CH_Instance.Move.w;
            CH_Instance.Speed_Set[1] = -CH_Instance.Move.x + CH_Instance.Move.y + CH_Instance.Move.w;
            CH_Instance.Speed_Set[2] = -CH_Instance.Move.x - CH_Instance.Move.y + CH_Instance.Move.w;
            CH_Instance.Speed_Set[3] = CH_Instance.Move.x - CH_Instance.Move.y + CH_Instance.Move.w;
            break;
        default:
            break;
    }
}

/**
 * @brief 定时器中断回调函数，计算电机速度并进行PID控制
 * @param htim 定时器句柄
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim == &GAP_TIM) {
        pitch_pid.Output = PID_Calculate(&pitch_pid, target_angle, current_angle); // pid计算
        yaw_pid.Output = PID_Calculate(&yaw_pid, target_angle, current_angle); // pid计算
        for (uint8_t i = 0; i < 4; i++) {
            MG310[i].Speed = MG310[i].Motor_GetSpeed(MG310[i].TIMx);
            PID_Calculate(&MG310[i].PID, CH_Instance.Speed_Set[i], MG310[i].Speed);
        }
    }
}
