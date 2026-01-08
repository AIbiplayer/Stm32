/**
 * @file Chassis_Task.c
 * @brief 底盘任务程序
 * @author Shen FeiLin
 * @date 2025/10/24
 */

#include "cmsis_os.h"
#include "Motor_Def.h"
#include "DJI_Motor.h"
#include "DM_Motor.h"
#include "message_center.h"
#include "can.h"
#include "power_limit.h"
#include "math.h"
#include "robot_def.h"
#include "TMC.h"
#include "can_comm.h"

#ifdef MCU_CHASSIS

static float chassis_vx, chassis_vy, chassis_vw; // 将云台系的速度投影到底盘
static float Mec_V1, Mec_V2, Mec_V3, Mec_V4; // 四轮速度
CCMRAM static DJI_Motor_Instance *Mec_Wheel[4];
CCMRAM static DM_Motor_Instance *Track_Wheel[4];
CCMRAM static Chassis_Ctrl_Cmd_s chassis_cmd_recv; // 底盘接收到的控制命令
CCMRAM static Chassis_Upload_Data_s chassis_feedback_data; // 底盘回传的反馈数据
CCMRAM static PID_Typedef WZ_ROTATE_PID, WZ_FOLLOW_PID;

CCMRAM static TMC_To_Chassis_s *Chassis_Data; // 底盘与云台数据结构体实例
Track_Mode_e Chassis_Track_Mode; ///< 底盘履带模式
extern CANCommInstance *CANCOM;

static void Chassis_Init(void);

static void Chassis_Output(void);

static void Chassis_Status_Serve(void);

static void Speed_Calculate(void);

/**
 * @brief 底盘FreeRTOS任务
 */
void ChassisTask(void *argument) {
    taskENTER_CRITICAL();
    Chassis_Init();
    taskEXIT_CRITICAL();
    for (;;) {
        Speed_Calculate();
        Chassis_Status_Serve();
        Chassis_Output();
        osDelay(1);
    }
}

/**
 * @brief 底盘初始化
 * @note PID参数在此调整
 */
static void Chassis_Init(void) {
    //初始化电机模型
    Motor_Init_s Mec_Chassis = {
        .Can_Init_Config = {.can_handle = &hcan2},
        .Control_Setting = {
            .Loop_Control = SPEED_CONTROL,
            .Angle_Feedback_Source = MOTOR_FEEDBACK,
            .Speed_Feedback_Source = MOTOR_FEEDBACK,
            .Feedforward_Flag = FEEDFORWARD_NONE,
            .Other_Angle_Feedback_Ptr = NULL,
            .Other_Speed_Feedback_Ptr = NULL,
            .Feedforward_Ptr = NULL
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
              0,
              0,
              1000,
              8000);

    Mec_Chassis.Can_Init_Config.tx_id = 1;
    Mec_Chassis.Control_Setting.Reverse_Flag = MOTOR_NORMAL;
    Mec_Wheel[0] = DJI_Motor_Init(&Mec_Chassis);
    PLMotor_Register(Mec_Wheel[0]);

    Mec_Chassis.Can_Init_Config.tx_id = 2;
    Mec_Chassis.Control_Setting.Reverse_Flag = MOTOR_NORMAL;
    Mec_Wheel[1] = DJI_Motor_Init(&Mec_Chassis);
    PLMotor_Register(Mec_Wheel[1]);

    Mec_Chassis.Can_Init_Config.tx_id = 3;
    Mec_Chassis.Control_Setting.Reverse_Flag = MOTOR_REVERSE;
    Mec_Wheel[2] = DJI_Motor_Init(&Mec_Chassis);
    PLMotor_Register(Mec_Wheel[2]);

    Mec_Chassis.Can_Init_Config.tx_id = 4;
    Mec_Chassis.Control_Setting.Reverse_Flag = MOTOR_REVERSE;
    Mec_Wheel[3] = DJI_Motor_Init(&Mec_Chassis);
    PLMotor_Register(Mec_Wheel[3]);

    DM_Motor_Init_s Track = {
        .Can_Init_Config = {.can_handle = &hcan2},
        .DM_Control = {
            .Angle_Feedback_Source = MOTOR_FEEDBACK,
            .Speed_Feedback_Source = MOTOR_FEEDBACK,
            .Other_Angle_Feedback_Ptr = NULL,
            .Other_Speed_Feedback_Ptr = NULL
        },
        .Working_Type = MOTOR_ENABLE,
        .Mode = pos_mode
    };

    Track.Can_Init_Config.rx_id = 1;
    Track.DM_Control.Reverse_Flag = MOTOR_REVERSE;
    Track_Wheel[0] = DM_Motor_Init(&Track);

    Track.Can_Init_Config.rx_id = 3;
    Track.DM_Control.Reverse_Flag = MOTOR_NORMAL;
    Track_Wheel[1] = DM_Motor_Init(&Track);

    Track.Can_Init_Config.can_handle = &hcan1;
    Track.Can_Init_Config.rx_id = 5;
    Track.DM_Control.Reverse_Flag = MOTOR_REVERSE;
    Track_Wheel[2] = DM_Motor_Init(&Track);

    Track.Can_Init_Config.rx_id = 7;
    Track.DM_Control.Reverse_Flag = MOTOR_NORMAL;
    Track_Wheel[3] = DM_Motor_Init(&Track);

    DM_MotorEnable(Track_Wheel[0]);
    DM_MotorEnable(Track_Wheel[1]);
    DM_MotorEnable(Track_Wheel[2]);
    DM_MotorEnable(Track_Wheel[3]);

    DM_MotorSaveZero(Track_Wheel[0]);
    DM_MotorSaveZero(Track_Wheel[1]);
    DM_MotorSaveZero(Track_Wheel[2]);
    DM_MotorSaveZero(Track_Wheel[3]);

    PID_Param(&WZ_ROTATE_PID, 0.0f, 1.0f, 0, Integral_Limit | Derivative_On_Measurement,
              1.0f, 0.0f, 3, 3);
    PID_Param(&WZ_FOLLOW_PID, 2.7f, 0.0f, 0.1f, Integral_Limit | Derivative_On_Measurement | OutputFilter,
              0.9f, 0.0f, 20, 200);

    Chassis_Data = (TMC_To_Chassis_s *) CANCommGet(CANCOM);
}

/**
 * @brief 轮子速度解算函数
 * @note 底盘运动学模型，这里包含底盘与云台的角度计算等
 */
static void Speed_Calculate(void) {
    if (Chassis_Data != NULL)
        chassis_cmd_recv = Chassis_Data->Chassis_Cmd;

    static float sin_theta, cos_theta; // 夹角正余弦值
    cos_theta = cosf(chassis_cmd_recv.offset_angle * DEGREE_2_RAD); // 角度*pai/180
    sin_theta = sinf(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
    chassis_vw = chassis_cmd_recv.wz;

    switch (chassis_cmd_recv.chassis_mode) {
        case CHASSIS_FOLLOW_GIMBAL_YAW: {
            PID_Clean_I(&WZ_ROTATE_PID);
            chassis_vx = chassis_cmd_recv.vx * cos_theta - chassis_cmd_recv.vy * sin_theta;
            chassis_vy = chassis_cmd_recv.vx * sin_theta + chassis_cmd_recv.vy * cos_theta;

            if (fabsf(chassis_cmd_recv.angle_offset_c) >= 8.0f)
                chassis_vw = PID_Calculate(&WZ_FOLLOW_PID, 0, chassis_cmd_recv.angle_offset_c);
            else
                chassis_vw = 0;
            break;
        }
        case CHASSIS_ROTATE: {
            PID_Clean_I(&WZ_FOLLOW_PID);
            chassis_vw = PID_Calculate(&WZ_ROTATE_PID, 0.001f, 0);
            chassis_vx = chassis_cmd_recv.vx * cos_theta - chassis_cmd_recv.vy * sin_theta;
            chassis_vy = chassis_cmd_recv.vx * sin_theta + chassis_cmd_recv.vy * cos_theta;
            break;
        }
        case CHASSIS_INDEPENDENCE: {
            PID_Clean_I(&WZ_ROTATE_PID);
            PID_Clean_I(&WZ_FOLLOW_PID);
            chassis_vx = chassis_cmd_recv.vy;
            break;
        }
        default:
            PID_Clean_I(&WZ_ROTATE_PID);
            break;
    }
    // 线速度为mm/s，角速度为转/s，轮子速度为rpm
    Mec_V1 = (chassis_vx - chassis_vy + chassis_vw * LF_CENTER) / RADIUS_WHEEL * REDUCTION_RATIO_WHEEL * RADS_2_RPM;
    Mec_V2 = -(chassis_vx + chassis_vy - chassis_vw * LB_CENTER) / RADIUS_WHEEL * REDUCTION_RATIO_WHEEL * RADS_2_RPM;
    Mec_V3 = (chassis_vx - chassis_vy - chassis_vw * RB_CENTER) / RADIUS_WHEEL * REDUCTION_RATIO_WHEEL * RADS_2_RPM;
    Mec_V4 = -(chassis_vx + chassis_vy + chassis_vw * RF_CENTER) / RADIUS_WHEEL * REDUCTION_RATIO_WHEEL * RADS_2_RPM;
}

/**
 * @brief 底盘运动状态识别
 */
static void Chassis_Status_Serve(void) {
    if (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE
        && chassis_cmd_recv.chassis_last_mode != chassis_cmd_recv.chassis_mode) {
        DJI_MotorStop(Mec_Wheel[0]);
        DJI_MotorStop(Mec_Wheel[1]);
        DJI_MotorStop(Mec_Wheel[2]);
        DJI_MotorStop(Mec_Wheel[3]);

        DM_MotorStop(Track_Wheel[0]);
        DM_MotorStop(Track_Wheel[1]);
        DM_MotorStop(Track_Wheel[2]);
        DM_MotorStop(Track_Wheel[3]);
    } else if (chassis_cmd_recv.chassis_mode != CHASSIS_ZERO_FORCE
               && chassis_cmd_recv.chassis_last_mode == CHASSIS_ZERO_FORCE) {
        DJI_MotorEnable(Mec_Wheel[0]);
        DJI_MotorEnable(Mec_Wheel[1]);
        DJI_MotorEnable(Mec_Wheel[2]);
        DJI_MotorEnable(Mec_Wheel[3]);

        DM_MotorEnable(Track_Wheel[0]);
        DM_MotorEnable(Track_Wheel[1]);
        DM_MotorEnable(Track_Wheel[2]);
        DM_MotorEnable(Track_Wheel[3]);
    }
}

/**
 * @brief 底盘速度输出至电机
 */
static void Chassis_Output(void) {
    DJI_MotorSetTarget(Mec_Wheel[0], Mec_V1);
    DJI_MotorSetTarget(Mec_Wheel[1], Mec_V2);
    DJI_MotorSetTarget(Mec_Wheel[2], Mec_V3);
    DJI_MotorSetTarget(Mec_Wheel[3], Mec_V4);

    DM_MotorSet(Track_Wheel[0], chassis_cmd_recv.a_track_head * REDUCTION_TRACK * REDUCTION_RATIO_WHEEL, 12.2f);
    DM_MotorSet(Track_Wheel[1], chassis_cmd_recv.a_track_head * REDUCTION_TRACK * REDUCTION_RATIO_WHEEL, 12.2f);
    DM_MotorSet(Track_Wheel[2], chassis_cmd_recv.a_track_back * REDUCTION_TRACK * REDUCTION_RATIO_WHEEL, 12.2f);
    DM_MotorSet(Track_Wheel[3], chassis_cmd_recv.a_track_back * REDUCTION_TRACK * REDUCTION_RATIO_WHEEL, 12.2f);
}

#endif
