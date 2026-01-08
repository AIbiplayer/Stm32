/**
* @file Shoot_Task.c
 * @brief 射击任务程序
 * @author Shen FeiLin
 * @date 2025/12/4
 */


#include "bsp_dwt.h"
#include "cmsis_os.h"
#include "message_center.h"
#include "Motor_Def.h"
#include "robot_def.h"
#include "DJI_Motor.h"
#include "TMC.h"

static void Shoot_Init(void);

static void Shoot_Status_Serve(void);

CCMRAM static DJI_Motor_Instance *Friction_L, *Friction_R;
CCMRAM DJI_Motor_Instance *Load_bullet;
static Publisher_t *shoot_pub;
static Shoot_Ctrl_Cmd_s shoot_cmd_recv;
static Shoot_Upload_Data_s shoot_feedback_data;
static Subscriber_t *shoot_sub;
static uint8_t Shoot_One_Bullet_Flag = 0;
static float Shoot_Relieve_Time = 0;
static uint8_t Shoot_Relieve_Flag = 0;

/**
 * @brief 射击FreeRTOS任务
 */
void ShootTask(void const *argument) {
#ifdef MCU_GIMBAL
    taskENTER_CRITICAL();
    Shoot_Init();
    taskEXIT_CRITICAL();
#endif
    for (;;) {
#ifdef MCU_GIMBAL
        SubGetMessage(shoot_sub, &shoot_cmd_recv);
        Shoot_Status_Serve();
        PubPushMessage(shoot_pub, &shoot_feedback_data);
        osDelay(1);
#endif
    }
}

/**
 * @brief 射击初始化
 * @note PID参数在此调整
 */
static void Shoot_Init(void) {
    Motor_Init_s Load = {
        .Can_Init_Config = {.can_handle = &hcan1},
        .Control_Setting = {
            .Loop_Control = SPEED_CONTROL,
            .Angle_Feedback_Source = MOTOR_FEEDBACK,
            .Speed_Feedback_Source = MOTOR_FEEDBACK,
            .Feedforward_Flag = FEEDFORWARD_NONE,
            .Other_Angle_Feedback_Ptr = NULL,
            .Other_Speed_Feedback_Ptr = NULL,
            .Feedforward_Ptr = NULL
        },
        .Motor_Type = M2006,
        .Working_Type = MOTOR_ENABLE
    };
    Motor_Init_s Friction = {
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
    PID_Param(&Load.Control_Setting.Speed_PID,
              21,
              0,
              1,
              Integral_Limit | Derivative_On_Measurement,
              1,
              100,
              1000,
              8000);
    PID_Param(&Load.Control_Setting.Angle_PID,
              8,
              0,
              0,
              Integral_Limit | Derivative_On_Measurement,
              1,
              100,
              1000,
              8000);
    PID_Param(&Friction.Control_Setting.Speed_PID,
              11,
              0,
              0,
              Integral_Limit | Derivative_On_Measurement,
              1,
              100,
              1000,
              8000);

    Load.Can_Init_Config.tx_id = 1;
    Load.Control_Setting.Reverse_Flag = MOTOR_NORMAL;
    Load_bullet = DJI_Motor_Init(&Load);

    Friction.Can_Init_Config.tx_id = 2;
    Friction.Control_Setting.Reverse_Flag = MOTOR_REVERSE;
    Friction_L = DJI_Motor_Init(&Friction);

    Friction.Can_Init_Config.tx_id = 3;
    Friction.Control_Setting.Reverse_Flag = MOTOR_NORMAL;
    Friction_R = DJI_Motor_Init(&Friction);

    shoot_sub = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
}

/**
 * @brief 射击控制函数
 */
static void Shoot_Status_Serve(void) {
    if (shoot_cmd_recv.shoot_mode == SHOOT_OFF) {
        DJI_MotorStop(Friction_L);
        DJI_MotorStop(Friction_R);
        DJI_MotorStop(Load_bullet);
        return;
    }
    DJI_MotorEnable(Friction_L);
    DJI_MotorEnable(Friction_R);
    DJI_MotorEnable(Load_bullet);

    // 摩擦轮控制,单位m/s
    switch (shoot_cmd_recv.friction_mode) {
        case FRICTION_ON:
            DJI_MotorSetTarget(Friction_L, 10 * RADS_2_RPM / RADIUS_FRICTION * SHOOT_COMPENSATION_K * 1000.0f);
            DJI_MotorSetTarget(Friction_R, 10 * RADS_2_RPM / RADIUS_FRICTION * SHOOT_COMPENSATION_K * 1000.0f);
            break;
        case FRICTION_OFF:
            DJI_MotorSetTarget(Friction_L, 0);
            DJI_MotorSetTarget(Friction_R, 0);
            break;
    }

    // 堵转检测
    abs(Load_bullet->Measure.Speed < 150) && abs(Load_bullet->Measure.Current > 7000)
        ? (Load_bullet->Measure.Block_CNT++)
        : (Load_bullet->Measure.Block_CNT = 0);
    Load_bullet->Measure.Block_CNT > 10
        ? (Load_bullet->Measure.Block_Flag = 1, Load_bullet->Measure.Block_CNT = 10)
        : (Load_bullet->Measure.Block_Flag = 0);

    // 堵转检测与保护
    loader_mode_e previous_mode = shoot_cmd_recv.load_mode;
    shoot_cmd_recv.load_mode = Load_bullet->Measure.Block_Flag ? LOAD_REVERSE : shoot_cmd_recv.load_mode;
    if (previous_mode != LOAD_REVERSE && shoot_cmd_recv.load_mode == LOAD_REVERSE) {
        Shoot_Relieve_Flag = 1;
        Shoot_Relieve_Time = DWT_GetTimeline_ms();
    }
    // 反转保护时间到后恢复原来模式,为500ms
    Shoot_Relieve_Flag
        ? DWT_GetTimeline_ms() - Shoot_Relieve_Time > 500
              ? (Shoot_Relieve_Flag = 0)
              : (shoot_cmd_recv.load_mode = LOAD_REVERSE)
        : 0;
    // 上弹控制
    switch (shoot_cmd_recv.load_mode) {
        case LOAD_STOP:
            DJI_MotorChangeLoop(Load_bullet, SPEED_CONTROL);
            DJI_MotorSetTarget(Load_bullet, 0);
            Shoot_One_Bullet_Flag = 0;
            break;
        case LOAD_1_BULLET: // 单发控制，打一发子弹后停止而不是间隔打
            DJI_MotorChangeLoop(Load_bullet, ANGLE_SPEED_CONTROL);
            Shoot_One_Bullet_Flag = Shoot_One_Bullet_Flag == 2 ? 2 : 1;
            if (Shoot_One_Bullet_Flag == 1) {
                DJI_MotorSetTarget(Load_bullet,
                                   Load_bullet->Measure.Total_Angle + ONE_BULLET_DELTA_ANGLE * REDUCTION_RATIO_LOADER *
                                   REDUCTION_SHOOT);
                Shoot_One_Bullet_Flag = 2;
            }
            break;
        // 连发控制，单位Hz
        case LOAD_BURSTFIRE:
            DJI_MotorChangeLoop(Load_bullet, SPEED_CONTROL);
            DJI_MotorSetTarget(Load_bullet,
                               shoot_cmd_recv.shoot_rate * REDUCTION_RATIO_LOADER * REDUCTION_SHOOT * RADS_2_RPM / NUM_PER_CIRCLE);
            break;
        // 反转控制,单位rpm
        case LOAD_REVERSE:
            DJI_MotorChangeLoop(Load_bullet, SPEED_CONTROL);
            DJI_MotorSetTarget(Load_bullet, -8000);
            break;
    }
}
