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
#include "DM_Motor.h"
#include "message_center.h"
#include "TMC.h"

CCMRAM INS_t *Gimbal_IMU_Data; ///< 云台IMU数据
CCMRAM DJI_Motor_Instance *Gimbal_Yaw; ///<Yaw轴电机
CCMRAM DM_Motor_Instance *Gimbal_Pitch_Up; ///< Pitch轴达妙电机小Pitch
CCMRAM DM_Motor_Instance *Gimbal_Pitch_Down; ///<Pitch轴达妙电机大Pitch

extern DM_IMU_Instance_s DM_IMU;

static Publisher_t *gimbal_pub; // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub; // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv; // 来自cmd的控制信息

static void Gimbal_Init(void);

static void Gimbal_Status_Serve(void);

/**
 * @brief 云台任务
 */
void GimbalTask(void const *argument) {
#ifdef MCU_GIMBAL
    taskENTER_CRITICAL();
    Gimbal_Init();
    taskEXIT_CRITICAL();
#endif

    for (;;) {
#ifdef MCU_GIMBAL
        SubGetMessage(gimbal_sub, &gimbal_cmd_recv);

        // PT = Gimbal_Pitch->Control_Setting.Target;
        // PA = Gimbal_Pitch->Control_Setting.Angle_PID.Actual;
        // YT = Gimbal_Yaw->Control_Setting.Target;
        // YA = Gimbal_Yaw->Control_Setting.Angle_PID.Actual;
        // Gimbal_Pitch->Control_Setting.Angle_PID.Kp = PAKP;
        // Gimbal_Pitch->Control_Setting.Angle_PID.Ki = PAKI;
        // Gimbal_Pitch->Control_Setting.Angle_PID.Kd = PAKD;
        // Gimbal_Pitch->Control_Setting.Speed_PID.Kp = PSKP;
        // Gimbal_Pitch->Control_Setting.Speed_PID.Ki = PSKI;
        // Gimbal_Pitch->Control_Setting.Speed_PID.Kd = PSKD;
        // Gimbal_Yaw->Control_Setting.Angle_PID.Kp = YAKP;
        // Gimbal_Yaw->Control_Setting.Angle_PID.Ki = YAKI;
        // Gimbal_Yaw->Control_Setting.Angle_PID.Kd = YAKD;
        // Gimbal_Yaw->Control_Setting.Speed_PID.Kp = YSKP;
        // Gimbal_Yaw->Control_Setting.Speed_PID.Ki = YSKI;
        // Gimbal_Yaw->Control_Setting.Speed_PID.Kd = YSKD;

        Gimbal_Status_Serve();

        PubPushMessage(gimbal_pub, &gimbal_feedback_data);
#endif

        osDelay(1);
    }
}

/**
 * @brief 云台任务初始化
 * @note PID参数在此调整
 */
static void Gimbal_Init(void) {
    //初始化电机模型
    Motor_Init_s Yaw = {
        .Can_Init_Config = {.can_handle = &hcan1},
        .Control_Setting = {
            .Loop_Control = ANGLE_SPEED_CONTROL,
            .Angle_Feedback_Source = OTHER_FEEDBACK,
            .Speed_Feedback_Source = OTHER_FEEDBACK,
            .Feedforward_Flag = FEEDFORWARD_NONE,
            .Other_Angle_Feedback_Ptr = &Gimbal_IMU_Data->YawTotalAngle,
            .Other_Speed_Feedback_Ptr = &Gimbal_IMU_Data->Gyro[2],
            .Feedforward_Ptr = NULL
        },
        .Motor_Type = GM6020,
        .Working_Type = MOTOR_ENABLE
    };

    DM_Motor_Init_s Pitch_up = {
        .Can_Init_Config = {.can_handle = &hcan2},
        .Control_Setting = {
            .Loop_Control = ANGLE_SPEED_CONTROL,
            .Angle_Feedback_Source = OTHER_FEEDBACK,
            .Speed_Feedback_Source = OTHER_FEEDBACK,
            .Feedforward_Flag = FEEDFORWARD_NONE,
            .Other_Angle_Feedback_Ptr = &DM_IMU.Measure.Roll,
            .Other_Speed_Feedback_Ptr = &DM_IMU.Measure.Gyro[0],
            .Feedforward_Ptr = NULL
        }
    };

    DM_Motor_Init_s Pitch_down = {
        .Can_Init_Config = {.can_handle = &hcan2},
        .Control_Setting = {
            .Loop_Control = ANGLE_SPEED_CONTROL,
            .Angle_Feedback_Source = MOTOR_FEEDBACK,
            .Speed_Feedback_Source = MOTOR_FEEDBACK,
            .Feedforward_Flag = FEEDFORWARD_NONE,
            .Other_Angle_Feedback_Ptr = NULL,
            .Other_Speed_Feedback_Ptr = NULL,
            .Feedforward_Ptr = NULL
        }
    };

    PID_Param(&Yaw.Control_Setting.Speed_PID,
              90.0f,
              0.0f,
              1.0f,
              Integral_Limit | Derivative_On_Measurement,
              1,
              0,
              100,
              14000);
    PID_Param(&Yaw.Control_Setting.Angle_PID,
              20.0f,
              0.0f,
              400.0f,
              Integral_Limit | Derivative_On_Measurement,
              1,
              0,
              1000,
              14000);
    PID_Param(&Pitch_up.Control_Setting.Speed_PID,
              1.0f,
              0,
              0.0f,
              Integral_Limit | Derivative_On_Measurement,
              1,
              0,
              1000,
              8000);
    PID_Param(&Pitch_up.Control_Setting.Angle_PID,
              1.0f,
              0.0f,
              0.0f,
              Integral_Limit | Derivative_On_Measurement,
              1,
              0,
              1000,
              8000);
    PID_Param(&Pitch_down.Control_Setting.Speed_PID,
              1.0f,
              0,
              0.0f,
              Integral_Limit | Derivative_On_Measurement,
              1,
              0,
              1000,
              8000);
    PID_Param(&Pitch_down.Control_Setting.Angle_PID,
              1.0f,
              0.0f,
              0.0f,
              Integral_Limit | Derivative_On_Measurement,
              1,
              0,
              1000,
              8000);

    Yaw.Can_Init_Config.tx_id = 2;
    Yaw.Control_Setting.Reverse_Flag = MOTOR_NORMAL;
    Gimbal_Yaw = DJI_Motor_Init(&Yaw);
    Pitch_up.Can_Init_Config.tx_id = 2;
    Pitch_up.Control_Setting.Reverse_Flag = MOTOR_NORMAL;
    Gimbal_Pitch_Up = DM_Motor_Init(&Pitch_up);
    Pitch_down.Can_Init_Config.tx_id = 1;
    Pitch_down.Control_Setting.Reverse_Flag = MOTOR_NORMAL;
    Gimbal_Pitch_Down = DM_Motor_Init(&Pitch_down);

    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
}

/**
 * @brief 云台控制函数
 */
static void Gimbal_Status_Serve(void) {
    gimbal_cmd_recv.pitch = Angle_limit(gimbal_cmd_recv.pitch, 20.0f, -35.0f);
    switch (gimbal_cmd_recv.gimbal_mode) {
        case GIMBAL_DOWN:
            DJI_MotorSetTarget(Gimbal_Yaw,YAW_RESET_ANGLE); // 拨盘回零
            DM_MotorSet(Gimbal_Pitch_Up, PITCH_RESET_ANGLE); // 小Pitch回零
            DM_MotorSet(Gimbal_Pitch_Down, PITCH_HOLD_RESET_ANGLE); // 大Pitch回零
            // 重置PID积分
            break;
        case GIMBAL_NONE:
            DJI_MotorStop(Gimbal_Yaw);
            DM_MotorStop();
            PID_Clean_I(&Gimbal_Yaw->Control_Setting.Angle_PID);
            PID_Clean_I(&Gimbal_Yaw->Control_Setting.Speed_PID);
            PID_Clean_I(&Gimbal_Pitch_Up->Control_Setting.Angle_PID);
            PID_Clean_I(&Gimbal_Pitch_Up->Control_Setting.Speed_PID);
            PID_Clean_I(&Gimbal_Pitch_Down->Control_Setting.Angle_PID);
            PID_Clean_I(&Gimbal_Pitch_Down->Control_Setting.Speed_PID);
            break;
        default:
            DJI_MotorSetTarget(Gimbal_Yaw, gimbal_cmd_recv.yaw);
            DM_MotorSet(Gimbal_Pitch_Up, gimbal_cmd_recv.pitch);
            DM_MotorSet(Gimbal_Pitch_Down, PITCH_HOLD_EXTEND_ANGLE);
            break;
    }
    gimbal_feedback_data.yaw_motor_single_round_angle = (uint16_t) Gimbal_Yaw->Measure.Angle;
}
