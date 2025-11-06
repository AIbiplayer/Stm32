/**
 * @file Chassis_Task.c
 * @brief 底盘任务程序
 * @author Shen FeiLin
 * @date 2025/10/24
 */

#include "cmsis_os.h"
#include "Motor_Def.h"
#include "DJI_Motor.h"
#include "message_center.h"
#include "can.h"
#include "robot_def.h"

static DJI_Motor_Instance* Omni_Wheel[4];
static Publisher_t* chassis_pub; // 用于发布底盘的数据
static Subscriber_t* chassis_sub; // 用于订阅底盘的控制命令

static void Chassis_Init(void);

/**
 * @brief 底盘FreeRTOS任务
 */
void ChassisTask(void* argument)
{
    /* USER CODE BEGIN ChassisTask */

    Chassis_Init();

    /* Infinite loop */
    for (;;)
    {
        osDelay(1);
    }
}

/**
 * @brief 底盘初始化
 */
static void Chassis_Init(void)
{
    //初始化电机模型
    Motor_Init_s Omni_Chassis = {
        .Can_Init_Config = {.can_handle = &hcan1},
        .Control_Setting = {
            .Loop_Control = SPEED_CONTROL
        },
        .Motor_Type = M3508,
        .Working_Type = MOTOR_ENABLE
    };
    //初始化PID参数
    PID_Param(&Omni_Chassis.Control_Setting.Speed_PID,
              35,
              0,
              0,
              Integral_Limit | Derivative_On_Measurement,
              1,
              0,
              1000,
              8000);
    PID_Param(&Omni_Chassis.Control_Setting.Angle_PID,
              0,
              0,
              0,
              Integral_Limit | Derivative_On_Measurement,
              1,
              0,
              1000,
              8000);

    Omni_Chassis.Can_Init_Config.tx_id = 1;
    Omni_Chassis.Control_Setting.Reverse_Flag = MOTOR_NORMAL;
    Omni_Wheel[0] = DJI_Motor_Init(&Omni_Chassis);

    Omni_Chassis.Can_Init_Config.tx_id = 2;
    Omni_Chassis.Control_Setting.Reverse_Flag = MOTOR_NORMAL;
    Omni_Wheel[1] = DJI_Motor_Init(&Omni_Chassis);

    Omni_Chassis.Can_Init_Config.tx_id = 3;
    Omni_Chassis.Control_Setting.Reverse_Flag = MOTOR_NORMAL;
    Omni_Wheel[2] = DJI_Motor_Init(&Omni_Chassis);

    Omni_Chassis.Can_Init_Config.tx_id = 4;
    Omni_Chassis.Control_Setting.Reverse_Flag = MOTOR_NORMAL;
    Omni_Wheel[3] = DJI_Motor_Init(&Omni_Chassis);

    chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));

}
