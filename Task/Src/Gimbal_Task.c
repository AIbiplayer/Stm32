/**
* @file Gimbal_Task.c
 * @brief 云台控制任务
 * @author Shen FeiLin
 * @date 2025/11/7
 */

#include "main.h"
#include "cmsis_os.h"
#include "INS.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "message_center.h"

CCMRAM INS_t* Gimbal_IMU_Data; ///< 云台IMU数据
CCMRAM DJI_Motor_Instance* Gimbal_Yaw; ///<Yaw轴电机
CCMRAM DJI_Motor_Instance* Gimbal_Pitch; ///<Pitch轴电机

static Publisher_t* gimbal_pub; // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t* gimbal_sub; // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv; // 来自cmd的控制信息

static void Gimbal_Init(void);
static void Gimbal_Status_Serve(void);

/**
 * @brief 云台任务
 */
void GimbalTask(void const* argument)
{
    Gimbal_Init();
    for (;;)
    {
        SubGetMessage(gimbal_sub, &gimbal_cmd_recv);

        // Gimbal_Status_Serve();

        PubPushMessage(gimbal_pub, &gimbal_feedback_data);
        osDelay(1);
    }
}

/**
 * @brief 云台任务初始化
 * @note PID参数在此调整
 */
static void Gimbal_Init(void)
{
    //初始化电机模型
    Motor_Init_s Gimbal = {
        .Can_Init_Config = {.can_handle = &hcan1},
        .Control_Setting = {
            .Loop_Control = ANGLE_SPEED_CONTROL,
            .Angle_Feedback_Source = OTHER_FEEDBACK,
            .Speed_Feedback_Source = OTHER_FEEDBACK,
            .Feedforward_Flag = FEEDFORWARD_NONE,
            .Other_Angle_Feedback_Ptr = &Gimbal_IMU_Data->YawTotalAngle,
            .Other_Speed_Feedback_Ptr = &Gimbal_IMU_Data->Gyro[2],
            .Speed_Feedforward_Ptr = NULL
        },
        .Motor_Type = GM6020,
        .Working_Type = MOTOR_ENABLE
    };

    PID_Param(&Gimbal.Control_Setting.Speed_PID,
              10.0f,
              0,
              150.0f,
              Integral_Limit | Derivative_On_Measurement,
              1,
              0,
              1000,
              8000);
    PID_Param(&Gimbal.Control_Setting.Angle_PID,
              45,
              0,
              0,
              Integral_Limit | Derivative_On_Measurement,
              1,
              0,
              1000,
              8000);

    Gimbal.Can_Init_Config.tx_id = 1;
    Gimbal.Control_Setting.Reverse_Flag = MOTOR_NORMAL;
    // Gimbal_Yaw = DJI_Motor_Init(&Gimbal);

    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
}

/**
 * @brief 云台控制函数
 */
static void Gimbal_Status_Serve(void)
{
    switch (gimbal_cmd_recv.gimbal_mode)
    {
    case GIMBAL_ZERO_FORCE:
        DJI_MotorStop(Gimbal_Yaw);
        // 重置PID积分
        PID_Clean_I(&Gimbal_Yaw->Control_Setting.Angle_PID);
        PID_Clean_I(&Gimbal_Yaw->Control_Setting.Speed_PID);
        break;
    case GIMBAL_GYRO_MODE:
        DJI_MotorEnable(Gimbal_Yaw);
        DJI_MotorSetTarget(Gimbal_Yaw, gimbal_cmd_recv.yaw);
        break;
    default:
        break;
    }
    // gimbal_feedback_data.gimbal_imu_data = *Gimbal_IMU_Data;
    gimbal_feedback_data.yaw_motor_single_round_angle = (uint16_t)Gimbal_Yaw->Measure.Angle;
}
