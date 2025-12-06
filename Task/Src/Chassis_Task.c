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
#include "math.h"
#include "robot_def.h"

CCMRAM static float chassis_vx, chassis_vy, chassis_vw; // 将云台系的速度投影到底盘
CCMRAM static float Chassis_V1, Chassis_V2, Chassis_V3, Chassis_V4; // 四轮速度
CCMRAM static DJI_Motor_Instance* Omni_Wheel[4];
static Chassis_Ctrl_Cmd_s chassis_cmd_recv; // 底盘接收到的控制命令
static Chassis_Upload_Data_s chassis_feedback_data; // 底盘回传的反馈数据
static Publisher_t* chassis_pub; // 用于发布底盘的数据
static Subscriber_t* chassis_sub; // 用于订阅底盘的控制命令

static void Chassis_Init(void);
static void Chassis_Output(void);
static void Chassis_Status_Serve(void);
static void Speed_Calculate(void);

/**
 * @brief 底盘FreeRTOS任务
 */
void ChassisTask(void* argument)
{
    Chassis_Init();
    for (;;)
    {
        SubGetMessage(chassis_sub, &chassis_cmd_recv);

        Speed_Calculate();
        Chassis_Status_Serve();
        Chassis_Output();

        PubPushMessage(chassis_pub, &chassis_feedback_data);

        osDelay(1);
    }
}

/**
 * @brief 底盘初始化
 * @note PID参数在此调整
 */
static void Chassis_Init(void)
{
    //初始化电机模型
    Motor_Init_s Mec_Chassis = {
        .Can_Init_Config = {.can_handle = &hcan1},
        .Control_Setting = {
            .Loop_Control = SPEED_CONTROL,
            .Angle_Feedback_Source = MOTOR_FEEDBACK,
            .Speed_Feedback_Source = MOTOR_FEEDBACK,
            .Feedforward_Flag = FEEDFORWARD_NONE,
            .Other_Angle_Feedback_Ptr = NULL,
            .Other_Speed_Feedback_Ptr = NULL,
            .Speed_Feedforward_Ptr = NULL
        },
        .Motor_Type = M3508,
        .Working_Type = MOTOR_ENABLE
    };
    //初始化PID参数
    PID_Param(&Mec_Chassis.Control_Setting.Speed_PID,
              25,
              0,
              0,
              Integral_Limit | Derivative_On_Measurement,
              1,
              100,
              1000,
              8000);
    PID_Param(&Mec_Chassis.Control_Setting.Angle_PID,
              0.0f,
              0.0f,
              0,
              Integral_Limit | Derivative_On_Measurement,
              1,
              0,
              1000,
              8000);

    Mec_Chassis.Can_Init_Config.tx_id = 1;
    Mec_Chassis.Control_Setting.Reverse_Flag = MOTOR_NORMAL;
    Omni_Wheel[0] = DJI_Motor_Init(&Mec_Chassis);

    Mec_Chassis.Can_Init_Config.tx_id = 2;
    Mec_Chassis.Control_Setting.Reverse_Flag = MOTOR_NORMAL;
    Omni_Wheel[1] = DJI_Motor_Init(&Mec_Chassis);

    Mec_Chassis.Can_Init_Config.tx_id = 3;
    Mec_Chassis.Control_Setting.Reverse_Flag = MOTOR_REVERSE;
    Omni_Wheel[2] = DJI_Motor_Init(&Mec_Chassis);

    Mec_Chassis.Can_Init_Config.tx_id = 4;
    Mec_Chassis.Control_Setting.Reverse_Flag = MOTOR_REVERSE;
    Omni_Wheel[3] = DJI_Motor_Init(&Mec_Chassis);

    chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
}

/**
 * @brief 轮子速度解算函数
 * @note 底盘运动学模型，这里包含底盘与云台的角度计算等
 * @todo 此为全向轮解算，后续改为麦轮
 */
static void Speed_Calculate(void)
{
    static PID_Typedef WZ_ROTATE_PID, WZ_FOLLOW_PID;

    PID_Param(&WZ_ROTATE_PID,
              0.0f,
              4.5f,
              0,
              Integral_Limit | Derivative_On_Measurement,
              1.0f,
              0,
              100,
              300);
    PID_Param(&WZ_FOLLOW_PID,
              2.7f,
              0.0f,
              0.1f,
              Integral_Limit | Derivative_On_Measurement | OutputFilter,
              0.9f,
              5.0f,
              20,
              200);

    static float sin_theta, cos_theta; // 夹角正余弦值
    cos_theta = cosf(chassis_cmd_recv.offset_angle * DEGREE_2_RAD); // 角度*pai/180
    sin_theta = sinf(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
    chassis_vw = chassis_cmd_recv.wz;

    switch (chassis_cmd_recv.chassis_mode)
    {
    case CHASSIS_FOLLOW_GIMBAL_YAW:
        {
            PID_Clean_I(&WZ_ROTATE_PID);
            chassis_vx = chassis_cmd_recv.vx * cos_theta - chassis_cmd_recv.vy * sin_theta;
            chassis_vy = chassis_cmd_recv.vx * sin_theta + chassis_cmd_recv.vy * cos_theta;

            if (fabsf(chassis_cmd_recv.angle_offset_c) >= 8.0f)
                chassis_vw = PID_Calculate(&WZ_FOLLOW_PID, 0, chassis_cmd_recv.angle_offset_c);
            else
                chassis_vw = 0;
            break;
        }
    case CHASSIS_ROTATE:
        {
            PID_Clean_I(&WZ_FOLLOW_PID);
            chassis_vw = PID_Calculate(&WZ_ROTATE_PID, 0.1f, 0);
            chassis_vx = chassis_cmd_recv.vx * cos_theta - chassis_cmd_recv.vy * sin_theta;
            chassis_vy = chassis_cmd_recv.vx * sin_theta + chassis_cmd_recv.vy * cos_theta;
            break;
        }
    case CHASSIS_INDEPENDENCE:
        {
            PID_Clean_I(&WZ_ROTATE_PID);
            PID_Clean_I(&WZ_FOLLOW_PID);
            chassis_vy = chassis_cmd_recv.vy;
            break;
        }
    default:
        PID_Clean_I(&WZ_ROTATE_PID);
        break;
    }
    Chassis_V1 = (chassis_vx - chassis_vy + chassis_vw * LF_CENTER);
    Chassis_V2 = (chassis_vx + chassis_vy + chassis_vw * LB_CENTER);
    Chassis_V3 = (chassis_vx - chassis_vy - chassis_vw * RB_CENTER);
    Chassis_V4 = (chassis_vx + chassis_vy - chassis_vw * RF_CENTER);
}

/**
 * @brief 底盘运动状态识别
 */
static void Chassis_Status_Serve(void)
{
    if (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE)
    {
        DJI_MotorStop(Omni_Wheel[0]);
        DJI_MotorStop(Omni_Wheel[1]);
        DJI_MotorStop(Omni_Wheel[2]);
        DJI_MotorStop(Omni_Wheel[3]);
    }
    else
    {
        DJI_MotorEnable(Omni_Wheel[0]);
        DJI_MotorEnable(Omni_Wheel[1]);
        DJI_MotorEnable(Omni_Wheel[2]);
        DJI_MotorEnable(Omni_Wheel[3]);
    }
}

/**
 * @brief 底盘速度输出至电机
 */
static void Chassis_Output(void)
{
    DJI_MotorSetTarget(Omni_Wheel[0], Chassis_V1);
    DJI_MotorSetTarget(Omni_Wheel[1], Chassis_V2);
    DJI_MotorSetTarget(Omni_Wheel[2], Chassis_V3);
    DJI_MotorSetTarget(Omni_Wheel[3], Chassis_V4);
}
