/**
* @file Gimbal_Task.c
 * @brief 云台控制任务
 * @author Shen FeiLin
 * @date 2025/11/7
 */

#include "main.h"
#include "cmsis_os.h"
#include "math.h"
#include "INS.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "DM_Motor.h"
#include "message_center.h"
#include "TMC.h"
#include "daemon.h"
#include "Trajectory_planning.h"

CCMRAM INS_t *Gimbal_IMU_Data; ///< 云台IMU数据
CCMRAM DJI_Motor_Instance *Gimbal_Yaw; ///<Yaw轴电机
CCMRAM DM_Motor_Instance *Gimbal_Pitch_Up; ///< Pitch轴达妙电机小Pitch
CCMRAM DM_Motor_Instance *Gimbal_Pitch_Down; ///<Pitch轴达妙电机大Pitch

extern DM_IMU_Instance_s DM_IMU;

typedef enum {
    YAW_FEEDBACK_MOTOR = 0,
    YAW_FEEDBACK_IMU
} yaw_feedback_mode_e;

typedef enum {
    PITCH_DOWN_FEEDBACK_MOTOR = 0,
    PITCH_DOWN_FEEDBACK_IMU
} pitch_down_feedback_mode_e;

static Publisher_t *gimbal_pub; // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub; // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv; // 来自cmd的控制信息
static float gimbal_actual_pos[GIMBAL_AXIS_COUNT];
static float gimbal_plan_start_pos[GIMBAL_AXIS_COUNT];
static float gimbal_target_pos[GIMBAL_AXIS_COUNT];
static float yaw_motor_actual_angle;
static float yaw_imu_actual_angle;
static float yaw_imu_target_offset;
static yaw_feedback_mode_e yaw_feedback_mode = YAW_FEEDBACK_MOTOR;
static float pitch_up_gravity_feedforward;
static float pitch_up_friction_feedforward;
static float pitch_up_feedforward_total;
static float pitch_down_motor_actual_angle;
static float pitch_down_imu_actual_angle;
static pitch_down_feedback_mode_e pitch_down_feedback_mode = PITCH_DOWN_FEEDBACK_MOTOR;

float PT, PA, PP, PSA, PST; // Pitch轴目标角度和实际角度
float PAKP = 0, PAKI = 0, PAKD = 0; // Pitch轴角度环PID参数
float PSKP = 0, PSKI = 0, PSKD = 0;
float YAKP = 0, YAKI = 0, YAKD = 0, YSKP = 0, YSKI = 0, YSKD = 0;
float YA, PUA, PDA, YSA, YST, POUT, gyro1;
float PU, PD, YT, Yang, PUang, PDang;

static void Gimbal_Init(void);

static void Gimbal_Status_Serve(void);

static void Pitch_Up_Update_Feedforward(void);

static void Yaw_Switch_To_Motor_Feedback(void);

static void Yaw_Switch_To_IMU_Feedback(float cmd_yaw);

static float Yaw_Get_Effective_Target(float cmd_yaw);

static void Pitch_Down_Switch_To_Motor_Feedback(void);

static void Pitch_Down_Switch_To_IMU_Feedback(void);

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
        YSA = Gimbal_Yaw->Control_Setting.Speed_PID.Actual;
        YST = Gimbal_Yaw->Control_Setting.Speed_PID.Target;

        PSA = Gimbal_Pitch_Up->Control_Setting.Speed_PID.Actual;
        PST = Gimbal_Pitch_Up->Control_Setting.Speed_PID.Target;

        YA = Gimbal_Yaw->Control_Setting.Angle_PID.Actual;
        PUA = Gimbal_Pitch_Up->Control_Setting.Angle_PID.Actual;
        PDA = Gimbal_Pitch_Down->Control_Setting.Angle_PID.Actual;
        PU = Gimbal_Pitch_Up->Control_Setting.Target;
        PD = Gimbal_Pitch_Down->Control_Setting.Target;
        YT = Gimbal_Yaw->Control_Setting.Target;
        Yang = Gimbal_Yaw->Measure.Angle;
        PUang = Gimbal_Pitch_Up->Measure.angle;
        PDang = Gimbal_Pitch_Down->Measure.angle;
        POUT = Gimbal_Pitch_Up->Control_Setting.Power_Output;
        // PT = Gimbal_Pitch_Down->Control_Setting.Target;
        // PA = Gimbal_Pitch_Down->Control_Setting.Angle_PID.Actual;
        // PSA = Gimbal_Pitch_Down->Control_Setting.Speed_PID.Actual;
        // PST = Gimbal_Pitch_Down->Control_Setting.Speed_PID.Target;
        // PP = Gimbal_Pitch_Down->Control_Setting.Power_Output;
        // YT = Gimbal_Yaw->Control_Setting.Target;
        // YA = Gimbal_Yaw->Control_Setting.Angle_PID.Actual;
        // Gimbal_Pitch_Up->Control_Setting.Angle_PID.Kp = PAKP;
        // Gimbal_Pitch_Up->Control_Setting.Angle_PID.Ki = PAKI;
        // Gimbal_Pitch_Up->Control_Setting.Angle_PID.Kd = PAKD;
        // Gimbal_Pitch_Up->Control_Setting.Speed_PID.Kp = PSKP;
        // Gimbal_Pitch_Up->Control_Setting.Speed_PID.Ki = PSKI;
        // Gimbal_Pitch_Up->Control_Setting.Speed_PID.Kd = PSKD;
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
            .Angle_Feedback_Source = MOTOR_FEEDBACK,
            .Speed_Feedback_Source = OTHER_FEEDBACK,
            .Feedforward_Flag = FEEDFORWARD_NONE,
            .Other_Angle_Feedback_Ptr = &DM_IMU.Measure.YawTotalAngle,
            .Other_Speed_Feedback_Ptr = &DM_IMU.Measure.Gyro[2],
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
            .Feedforward_Flag = CURRENT_FEEDFORWARD,
            .Other_Angle_Feedback_Ptr = &DM_IMU.Measure.Pitch,
            .Other_Speed_Feedback_Ptr = &DM_IMU.Measure.Gyro[1],
            .Feedforward_Ptr = &pitch_up_feedforward_total
        }
    };

    DM_Motor_Init_s Pitch_down = {
        .Can_Init_Config = {.can_handle = &hcan2},
        .Control_Setting = {
            .Loop_Control = ANGLE_SPEED_CONTROL,
            .Angle_Feedback_Source = OTHER_FEEDBACK,
            .Speed_Feedback_Source = MOTOR_FEEDBACK,
            .Feedforward_Flag = FEEDFORWARD_NONE,
            .Other_Angle_Feedback_Ptr = &pitch_down_motor_actual_angle,
            .Other_Speed_Feedback_Ptr = NULL,
            .Feedforward_Ptr = NULL
        }
    };

    PID_Param(&Yaw.Control_Setting.Speed_PID,
              15.0f,
              0.0f,
              0.0f,
              Integral_Limit | Derivative_On_Measurement,
              1,
              0,
              100,
              14000);
    PID_Param(&Yaw.Control_Setting.Angle_PID,
              150.0f,
              0.0f,
              0.0f,
              Integral_Limit | Derivative_On_Measurement,
              1,
              0,
              1000,
              14000);
    PID_Param(&Pitch_up.Control_Setting.Speed_PID,
              -20.0f,
              0,
              0.0f,
              Integral_Limit | Derivative_On_Measurement,
              1,
              0,
              1000,
              8000);
    PID_Param(&Pitch_up.Control_Setting.Angle_PID,
              15.0f,
              0.0f,
              0.0f,
              Integral_Limit | Derivative_On_Measurement,
              1,
              0,
              1000,
              8000);
    PID_Param(&Pitch_down.Control_Setting.Speed_PID,
              -40.0f,
              -0.1f,
              0.0f,
              Integral_Limit | Derivative_On_Measurement,
              1,
              0,
              1000,
              8000);
    PID_Param(&Pitch_down.Control_Setting.Angle_PID,
              30.0f,
              0.0f,
              0.0f,
              Integral_Limit | Derivative_On_Measurement,
              1,
              0,
              1000,
              8000);

    Daemon_Init_Config_s Yaw_Daemon_Config = {
        .reload_count = 100, // 100ms超时
        .init_count = 1000, // 1s上线等待时间
        .callback = DJI_motor_offline, // 可以设置一个回调函数来处理离线情况
    };

    Yaw.Can_Init_Config.tx_id = 2;
    Yaw.Control_Setting.Reverse_Flag = MOTOR_NORMAL;
    Gimbal_Yaw = DJI_Motor_Init(&Yaw);
    Pitch_up.Can_Init_Config.tx_id = 1;
    Pitch_up.Control_Setting.Reverse_Flag = MOTOR_NORMAL;
    Gimbal_Pitch_Up = DM_Motor_Init(&Pitch_up);
    Pitch_down.Can_Init_Config.tx_id = 2;
    Pitch_down.Control_Setting.Reverse_Flag = MOTOR_NORMAL;
    Gimbal_Pitch_Down = DM_Motor_Init(&Pitch_down);
    Gimbal_Yaw->Control_Setting.Target = YAW_TIGHTEN_ANGLE;
    Gimbal_Pitch_Down->Control_Setting.Target = PITCH_TIGHTEN_ANGLE;
    Gimbal_Pitch_Up->Control_Setting.Target = PITCH_HEAD_ANGLE;

    Yaw_Daemon_Config.owner_id = &Gimbal_Yaw;
    Gimbal_Yaw->Motor_Can_Instance->daemon_instance = DaemonRegister(&Yaw_Daemon_Config); // 注册守护程序

    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    GimbalTrajectory_Init();
}

static void Yaw_Switch_To_Motor_Feedback(void) {
    Gimbal_Yaw->Control_Setting.Angle_Feedback_Source = MOTOR_FEEDBACK;
    Gimbal_Yaw->Control_Setting.Other_Angle_Feedback_Ptr = NULL;
    Gimbal_Yaw->Control_Setting.Target = yaw_motor_actual_angle;
    PID_Clean_I(&Gimbal_Yaw->Control_Setting.Angle_PID);
    yaw_feedback_mode = YAW_FEEDBACK_MOTOR;
}

static void Yaw_Switch_To_IMU_Feedback(float cmd_yaw) {
    Gimbal_Yaw->Control_Setting.Angle_Feedback_Source = OTHER_FEEDBACK;
    Gimbal_Yaw->Control_Setting.Other_Angle_Feedback_Ptr = &DM_IMU.Measure.YawTotalAngle;
    yaw_imu_target_offset = yaw_imu_actual_angle - cmd_yaw;
    Gimbal_Yaw->Control_Setting.Target = yaw_imu_actual_angle;
    PID_Clean_I(&Gimbal_Yaw->Control_Setting.Angle_PID);
    yaw_feedback_mode = YAW_FEEDBACK_IMU;
}

static float Yaw_Get_Effective_Target(float cmd_yaw) {
    if (yaw_feedback_mode == YAW_FEEDBACK_IMU) {
        return cmd_yaw + yaw_imu_target_offset;
    }
    return cmd_yaw;
}

static void Pitch_Up_Update_Feedforward(void) {
    const float angle_offset_rad = (DM_IMU.Measure.Pitch - PITCH_UP_GRAVITY_COMP_HORIZON_ANGLE) * DEGREE_2_RAD;
    const float pitch_up_error = Gimbal_Pitch_Up->Control_Setting.Target - DM_IMU.Measure.Pitch;
    pitch_up_gravity_feedforward = PITCH_UP_GRAVITY_COMP_CURRENT * cosf(angle_offset_rad);
    pitch_up_friction_feedforward = 0.0f;

    if (pitch_up_error > PITCH_UP_FRICTION_COMP_DEADBAND) {
        pitch_up_friction_feedforward = PITCH_UP_FRICTION_COMP_POS_CURRENT;
    } else if (pitch_up_error < -PITCH_UP_FRICTION_COMP_DEADBAND) {
        pitch_up_friction_feedforward = PITCH_UP_FRICTION_COMP_NEG_CURRENT;
    }

    pitch_up_feedforward_total = pitch_up_gravity_feedforward + pitch_up_friction_feedforward;
}

static void Pitch_Down_Switch_To_Motor_Feedback(void) {
    Gimbal_Pitch_Down->Control_Setting.Angle_Feedback_Source = OTHER_FEEDBACK;
    Gimbal_Pitch_Down->Control_Setting.Speed_Feedback_Source = MOTOR_FEEDBACK;
    Gimbal_Pitch_Down->Control_Setting.Other_Angle_Feedback_Ptr = &pitch_down_motor_actual_angle;
    Gimbal_Pitch_Down->Control_Setting.Other_Speed_Feedback_Ptr = NULL;
    Gimbal_Pitch_Down->Control_Setting.Target = pitch_down_motor_actual_angle;
    PID_Clean_I(&Gimbal_Pitch_Down->Control_Setting.Angle_PID);
    PID_Clean_I(&Gimbal_Pitch_Down->Control_Setting.Speed_PID);
    pitch_down_feedback_mode = PITCH_DOWN_FEEDBACK_MOTOR;
}

static void Pitch_Down_Switch_To_IMU_Feedback(void) {
    Gimbal_Pitch_Down->Control_Setting.Angle_Feedback_Source = OTHER_FEEDBACK;
    Gimbal_Pitch_Down->Control_Setting.Speed_Feedback_Source = OTHER_FEEDBACK;
    Gimbal_Pitch_Down->Control_Setting.Other_Angle_Feedback_Ptr = &DM_IMU.Measure.Pitch;
    Gimbal_Pitch_Down->Control_Setting.Other_Speed_Feedback_Ptr = &DM_IMU.Measure.Gyro[1];
    Gimbal_Pitch_Down->Control_Setting.Target = pitch_down_imu_actual_angle;
    PID_Clean_I(&Gimbal_Pitch_Down->Control_Setting.Angle_PID);
    PID_Clean_I(&Gimbal_Pitch_Down->Control_Setting.Speed_PID);
    pitch_down_feedback_mode = PITCH_DOWN_FEEDBACK_IMU;
}

/**
 * @brief 云台控制函数
 */
static void Gimbal_Status_Serve(void) {
    static gimbal_mode_e last_gimbal_mode = GIMBAL_NONE;
    static uint8_t startup_tighten_latch = 1u;
    uint8_t yaw_motor_can_offline = 0;
    gimbal_mode_e active_gimbal_mode = gimbal_cmd_recv.gimbal_mode;
    float yaw_target_cmd;

    if (startup_tighten_latch) {
        if (active_gimbal_mode == GIMBAL_NONE) {
            active_gimbal_mode = GIMBAL_TIGHTEN;
        } else {
            startup_tighten_latch = 0u;
        }
    }

    gimbal_cmd_recv.pitch = Angle_limit(gimbal_cmd_recv.pitch, PITCH_MAX_ANGLE, PITCH_MIN_ANGLE);
    yaw_motor_actual_angle = Gimbal_Yaw->Measure.Total_Angle;
    yaw_imu_actual_angle = DM_IMU.Measure.YawTotalAngle;
    Pitch_Up_Update_Feedforward();
    pitch_down_motor_actual_angle = Gimbal_Pitch_Down->Measure.angle;
    pitch_down_imu_actual_angle = DM_IMU.Measure.Pitch;
    gimbal_actual_pos[GIMBAL_YAW_INDEX] = yaw_feedback_mode == YAW_FEEDBACK_MOTOR
                                              ? yaw_motor_actual_angle
                                              : yaw_imu_actual_angle;
    gimbal_actual_pos[GIMBAL_PITCH_DOWN_INDEX] = pitch_down_motor_actual_angle;
    gimbal_actual_pos[GIMBAL_PITCH_UP_INDEX] = DM_IMU.Measure.Pitch;

    if (active_gimbal_mode == GIMBAL_TIGHTEN && yaw_feedback_mode != YAW_FEEDBACK_MOTOR) {
        Yaw_Switch_To_Motor_Feedback();
        gimbal_actual_pos[GIMBAL_YAW_INDEX] = yaw_motor_actual_angle;
    }
    if (active_gimbal_mode == GIMBAL_TIGHTEN && pitch_down_feedback_mode != PITCH_DOWN_FEEDBACK_MOTOR) {
        Pitch_Down_Switch_To_Motor_Feedback();
        gimbal_actual_pos[GIMBAL_PITCH_DOWN_INDEX] = pitch_down_motor_actual_angle;
    }

    gimbal_plan_start_pos[GIMBAL_YAW_INDEX] = Gimbal_Yaw->Control_Setting.Target;
    gimbal_plan_start_pos[GIMBAL_PITCH_DOWN_INDEX] = Gimbal_Pitch_Down->Control_Setting.Target;
    gimbal_plan_start_pos[GIMBAL_PITCH_UP_INDEX] = Gimbal_Pitch_Up->Control_Setting.Target;

    if (last_gimbal_mode == GIMBAL_NONE && active_gimbal_mode != GIMBAL_NONE) {
        gimbal_plan_start_pos[GIMBAL_YAW_INDEX] = yaw_feedback_mode == YAW_FEEDBACK_MOTOR
                                                      ? yaw_motor_actual_angle
                                                      : yaw_imu_actual_angle;
        gimbal_plan_start_pos[GIMBAL_PITCH_DOWN_INDEX] = gimbal_actual_pos[GIMBAL_PITCH_DOWN_INDEX];
        gimbal_plan_start_pos[GIMBAL_PITCH_UP_INDEX] = gimbal_actual_pos[GIMBAL_PITCH_UP_INDEX];
    }

    if (Gimbal_Yaw->Motor_Can_Instance != NULL && Gimbal_Yaw->Motor_Can_Instance->daemon_instance != NULL) {
        yaw_motor_can_offline = !DaemonIsOnline(Gimbal_Yaw->Motor_Can_Instance->daemon_instance);
    }

    switch (active_gimbal_mode) {
        case GIMBAL_NONE:
            GimbalTrajectory_Reset();
            if (yaw_feedback_mode == YAW_FEEDBACK_IMU) {
                yaw_imu_target_offset = 0.0f;
            }
            DJI_MotorSetTarget(Gimbal_Yaw, yaw_feedback_mode == YAW_FEEDBACK_MOTOR
                                               ? yaw_motor_actual_angle
                                               : yaw_imu_actual_angle);
            DJI_MotorStop(Gimbal_Yaw);
            DM_MotorStop();
            break;
        default:
            DJI_MotorEnable(Gimbal_Yaw);
            DM_MotorEnable();
            yaw_target_cmd = Yaw_Get_Effective_Target(gimbal_cmd_recv.yaw);
            GimbalTrajectory_SetRealtimeTarget(yaw_target_cmd, gimbal_cmd_recv.pitch);
            GimbalTrajectory_Update(gimbal_plan_start_pos, active_gimbal_mode);
            GimbalTrajectory_GetTarget(gimbal_target_pos);
            if ((active_gimbal_mode == GIMBAL_GYRO_MODE || active_gimbal_mode == GIMBAL_VISION) &&
                yaw_feedback_mode == YAW_FEEDBACK_MOTOR &&
                GimbalTrajectory_GetState() == GIMBAL_TRAJECTORY_READY) {
                Yaw_Switch_To_IMU_Feedback(gimbal_cmd_recv.yaw);
                gimbal_target_pos[GIMBAL_YAW_INDEX] = yaw_imu_actual_angle;
            }
            DJI_MotorSetTarget(Gimbal_Yaw, gimbal_target_pos[GIMBAL_YAW_INDEX]);
            DM_MotorSet(Gimbal_Pitch_Down, gimbal_target_pos[GIMBAL_PITCH_DOWN_INDEX]);
            DM_MotorSet(Gimbal_Pitch_Up, gimbal_target_pos[GIMBAL_PITCH_UP_INDEX]);
            break;
    }
    gimbal_feedback_data.yaw_motor_single_round_angle = (uint16_t) Gimbal_Yaw->Measure.Angle;
    gimbal_feedback_data.yaw_motor_total_angle = yaw_motor_actual_angle;
    gimbal_feedback_data.gimbal_imu_data = *Gimbal_IMU_Data;
    gimbal_feedback_data.gimbal_imu_data.Yaw = DM_IMU.Measure.Yaw;
    gimbal_feedback_data.gimbal_imu_data.Pitch = DM_IMU.Measure.Pitch;
    gimbal_feedback_data.gimbal_imu_data.YawTotalAngle = DM_IMU.Measure.YawTotalAngle;
    gimbal_feedback_data.yaw_motor_offline = yaw_motor_can_offline;
    last_gimbal_mode = active_gimbal_mode;
}
