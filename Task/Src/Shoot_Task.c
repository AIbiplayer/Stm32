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
#include "alg_queue.h"
#include "DJI_Motor.h"
#include "TMC.h"
#include "can_comm.h"
#include "referee.h"

static void Shoot_Init(void);

static void Calorie_Monitor(void);

static void FireContorl(void);

static void Shoot_Status_Serve(void);

static uint16_t DecodeHeatLimit(uint16_t raw_heat_limit);

CCMRAM static DJI_Motor_Instance *Friction_L, *Friction_R;
CCMRAM DJI_Motor_Instance *Load_bullet;
static Publisher_t *shoot_pub;
static Shoot_Ctrl_Cmd_s shoot_cmd_recv;
static Shoot_Upload_Data_s shoot_feedback_data;
static Subscriber_t *shoot_sub;
static uint8_t Shoot_One_Bullet_Flag = 0;
static float Shoot_Relieve_Time = 0;
static uint8_t Shoot_Relieve_Flag = 0;

// �?量�?�测代�?
static shoot_detection_e shoot_state = SHOOT_DETECTION_STOP; //发射状�?
static Shooter_Type_e shooter_type; //发射机构模式，爆发优先和冷却优先
static float now_heat_cd = 0; //当前冷却�?
static float now_heat = 0; //17mm�?口当前热�?
static float now_heat_calorie_monitor = 0; //�?量监测用的当前热�?,每�?�进入监测时更新
static uint16_t heat_limit_max = 0; //17mm�?口热量上�?
static float Total_Ammo_Num = 0; //总弹�?
static float Current_Queue_Sum = 0; //电流队列�?
static Queue_t Current_Queue; //电流队列
static uint32_t Calorie_Monitor_CNT = 0; //�?量监测时间戳
static float Current_Queue_Sum_Threshold = 160; //电流队列和的判定阈�?,需要根�?实际情况调整
CCMRAM static TMC_To_Gimbal_s *Shoot_Data; // 底盘与云台数�?结构体实�?

extern CANCommInstance *CANCOM;
extern volatile uint8_t shoot_speed;

float Total_angle_target, angle_actual;

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

        // Calorie_Monitor(); //�?量自检

        Shoot_Status_Serve();
        Total_angle_target = Load_bullet->Control_Setting.Target;
        angle_actual = Load_bullet->Control_Setting.Angle_PID.Actual;
        PubPushMessage(shoot_pub, &shoot_feedback_data);
        osDelay(1);
#endif
    }
}

/**
 * @brief 射击初�?�化
 * @note PID参数在�?�调�?
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
              18,
              0,
              1,
              Integral_Limit | Derivative_On_Measurement,
              1,
              100,
              1000,
              14000);
    PID_Param(&Load.Control_Setting.Angle_PID,
              5,
              0,
              0,
              Integral_Limit | Derivative_On_Measurement,
              1,
              100,
              1000,
              14000);
    PID_Param(&Friction.Control_Setting.Speed_PID,
              8,
              0,
              0,
              Integral_Limit | Derivative_On_Measurement,
              1,
              100,
              1000,
              14000);

    Load.Can_Init_Config.tx_id = 1;
    Load.Control_Setting.Reverse_Flag = MOTOR_NORMAL;
    Load_bullet = DJI_Motor_Init(&Load);

    Friction.Can_Init_Config.tx_id = 2;
    Friction.Control_Setting.Reverse_Flag = MOTOR_NORMAL;
    Friction_L = DJI_Motor_Init(&Friction);

    Friction.Can_Init_Config.tx_id = 1;
    Friction.Control_Setting.Reverse_Flag = MOTOR_REVERSE;
    Friction_R = DJI_Motor_Init(&Friction);

    Shoot_Data = (TMC_To_Gimbal_s *) CANCommGet(CANCOM); // 获取底盘与云台数�?结构体实�?

    shoot_sub = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
}

/**
 * @brief 射击控制函数
 */
static void Shoot_Status_Serve(void) {
    float friction_target_speed = shoot_speed == 0u ? 15.0f : (float) shoot_speed;

    if (shoot_cmd_recv.shoot_mode == SHOOT_OFF) {
        DJI_MotorStop(Friction_L);
        DJI_MotorStop(Friction_R);
        DJI_MotorStop(Load_bullet);
        return;
    }
    DJI_MotorEnable(Friction_L);
    DJI_MotorEnable(Friction_R);
    DJI_MotorEnable(Load_bullet);

    // �?量�?�测与控制
    if (Shoot_Data->Shoot_Upload_Data.reference_online_state == false) {
        // 裁判系统掉线或远�?信息不可�?
        if (shooter_type == Robot_Booster_Type_BURST) {
            //爆发优先模式
            heat_limit_max = booster_burst_first_heat_max[abs(Shoot_Data->Shoot_Upload_Data.robot_level - 1)];
            now_heat_cd = booster_burst_first_heat_cd[abs(Shoot_Data->Shoot_Upload_Data.robot_level - 1)];
        } else if (shooter_type == Robot_Booster_Type_CD) {
            //冷却优先模式
            heat_limit_max = booster_cd_first_heat_max[abs(Shoot_Data->Shoot_Upload_Data.robot_level - 1)];
            now_heat_cd = booster_cd_first_heat_cd[abs(Shoot_Data->Shoot_Upload_Data.robot_level - 1)];
        }
        now_heat = now_heat_calorie_monitor;
    } else {
        // 裁判系统�?�?
        heat_limit_max = DecodeHeatLimit(Shoot_Data->Shoot_Upload_Data.shooter_heat_limit);
        now_heat_cd = Shoot_Data->Shoot_Upload_Data.shooter_barrel_cooling_value;
        now_heat = Shoot_Data->Shoot_Upload_Data.heat;
    }

    // FireContorl();//根据�?量调整射�?

    // 摩擦�?控制,单位m/s
    switch (shoot_cmd_recv.friction_mode) {
        case FRICTION_ON:
            DJI_MotorSetTarget(Friction_L,
                               friction_target_speed * RADS_2_RPM / RADIUS_FRICTION * SHOOT_COMPENSATION_K *
                               1000.0f);
            DJI_MotorSetTarget(Friction_R,
                               friction_target_speed * RADS_2_RPM / RADIUS_FRICTION * SHOOT_COMPENSATION_K *
                               1000.0f);
            break;
        case FRICTION_OFF:
            DJI_MotorSetTarget(Friction_L, 0);
            DJI_MotorSetTarget(Friction_R, 0);
            break;
    }

    // 堵转检�?
    abs(Load_bullet->Measure.Speed) < 150 && abs(Load_bullet->Measure.Current) > 8000
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
    // 反转保护时间到后恢�?�原来模�?,�?500ms
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
        case LOAD_1_BULLET: // 单发控制，打一发子弹后停�?�而不�?间隔�?
            DJI_MotorChangeLoop(Load_bullet, ANGLE_SPEED_CONTROL);
            Shoot_One_Bullet_Flag = Shoot_One_Bullet_Flag == 2 ? 2 : 1;
            if (Shoot_One_Bullet_Flag == 1) {
                DJI_MotorSetTarget(Load_bullet,
                                   Load_bullet->Measure.Total_Angle - ONE_BULLET_DELTA_ANGLE * REDUCTION_RATIO_LOADER);
                Shoot_One_Bullet_Flag = 2;
            }
            break;
        // 连发控制，单位Hz
        case LOAD_BURSTFIRE:
            DJI_MotorChangeLoop(Load_bullet, SPEED_CONTROL);
            DJI_MotorSetTarget(Load_bullet,
                               -shoot_cmd_recv.shoot_rate * 2 * PI * REDUCTION_RATIO_LOADER *
                               RADS_2_RPM / NUM_PER_CIRCLE);
            break;
        // 反转控制,单位rpm
        case LOAD_REVERSE:
            DJI_MotorChangeLoop(Load_bullet, SPEED_CONTROL);
            DJI_MotorSetTarget(Load_bullet, -8000);
            break;
    }
}

/**
 * @brief �?量�?�测函�?
 */
static void Calorie_Monitor(void) //�?量监测使�?(�?完成)
{
    switch (shoot_state) {
        case SHOOT_DETECTION_STOP: {
            // 停机状态，当摩擦轮�?动且达到了目标速度时进入开机状�?
            if ((Friction_L->Control_Setting.Work_Type == MOTOR_ENABLE && Friction_R->Control_Setting.Work_Type ==
                 MOTOR_ENABLE) && (
                    (float) Friction_L->Control_Setting.Power_Output > 0.0f && (float) Friction_R->Control_Setting.
                    Power_Output < 0.0f)
                && ((float) Friction_L->Measure.Speed >= (float) Friction_L->Control_Setting.Power_Output * 0.95f &&
                    (float) Friction_R->Measure.Speed <= (float) Friction_R->Control_Setting.Power_Output * 0.95f)) {
                // 摩擦�?达到了目标速度且电机存�?->开机状�?
                shoot_state = SHOOT_DETECTION_READY;
            }
            break;
        }
        case SHOOT_DETECTION_READY: {
            // 开机状�?

            float now_current = (Friction_L->Measure.Current - Friction_R->Measure.Current) / 16384 * 20.0f;
            // 当前电流�?,单位A,需要根�?实际情况调整�?换系�?
            Current_Queue_Sum += now_current;
            Queue_Push(&Current_Queue, now_current);

            // 计算窗口内电流和
            // 如果队列满了，先弹出一�?旧数�?，并从总和�?减去
            if (Queue_Is_Full(&Current_Queue)) {
                float old_val = 0;
                Queue_Pop(&Current_Queue, &old_val); // 弹出最旧的数据
                Current_Queue_Sum -= old_val; // 减去它的�?
            }

            // 如果超过了判定阈值则认为�?打出子弹
            if (Current_Queue_Sum > Current_Queue_Sum_Threshold) {
                // 触发后清空，防�?�连�?�?�?
                Queue_Clear(&Current_Queue);
                Current_Queue_Sum = 0.0f;
                now_heat += 10.0f;
                Total_Ammo_Num++;
            }

            // 计算冷却
            float dt = DWT_GetDeltaT(&Calorie_Monitor_CNT);
            now_heat -= (float) (Shoot_Data->Shoot_Upload_Data.shooter_barrel_cooling_value) * dt;
            if (now_heat < 0.0f) {
                now_heat = 0.0f;
            }

            if ((Friction_L->Control_Setting.Work_Type == MOTOR_STOP && Friction_R->Control_Setting.Work_Type ==
                 MOTOR_STOP)
                || (Friction_L->Control_Setting.Power_Output == 0 && Friction_R->Control_Setting.Power_Output == 0)) {
                // 电机掉线->关机状�?
                Queue_Clear(&Current_Queue);
                Current_Queue_Sum = 0.0f;
                now_heat = 0.0f;

                shoot_state = SHOOT_DETECTION_STOP;
            }
            break;
        }
        default:
            shoot_state = SHOOT_DETECTION_STOP;
            break;
    }
    now_heat_calorie_monitor = now_heat;
}

static uint16_t DecodeHeatLimit(uint16_t raw_heat_limit) {
    return raw_heat_limit >= 1024u ? (uint16_t) (raw_heat_limit / 1024u) : raw_heat_limit;
}

static void FireContorl(void) //发射状态机,防�?�热量超�?
{
    float Qnow = now_heat; // 获取当前�?�?
    uint16_t Qlimit = heat_limit_max; // 裁判系统�?量上�? (例�?? 240/360)

    // 定义阈�? (假�?�逻辑：热量越高，射速越�?)
    // 下面这些值需要根�?实际情况调整
    float Q_safe = (float) Qlimit - 100; // 安全�? (对应图中�? Qres �?)
    float Q_warn = (float) Qlimit - 40; // 警告�? (开始减�?)
    float Q_stop = (float) Qlimit - 10; // 停�?�区 (对应图中�? Qres �?)

    float target_rate = 15; // 原�?��?�定的最大射频，不�?�直接用 shoot_cmd_recv.shoot_rate �?�?

    // 计算冷却对应的射�? (n_cd)
    float cd_rate = now_heat_cd / HEAT_OF_PROJECTILE;

    if (shoot_cmd_recv.load_mode == LOAD_BURSTFIRE) {
        if (Qnow < Q_safe) {
            // [安全区] �?量很低，全速发�?
            shoot_cmd_recv.shoot_rate = target_rate;
        } else if (Qnow >= Q_safe && Qnow < Q_warn) {
            // [线性限制区] �?量升高，�? target_rate 线性降低到 cd_rate
            // 类似于图�?斜坡的“镜像�?
            float ratio = (float) (Qnow - Q_safe) / (Q_warn - Q_safe);
            shoot_cmd_recv.shoot_rate = target_rate - ratio * (target_rate - cd_rate);
        } else if (Qnow >= Q_warn && Qnow < Q_stop) {
            // [饱和区] �?允�?�以冷却速度发射
            shoot_cmd_recv.shoot_rate = cd_rate;
        } else {
            // [危险区] �?量即将超限，停�?�发�?
            shoot_cmd_recv.shoot_rate = 0;
            // �?选：强制切回停�?�模�?
            // shoot_cmd_recv.load_mode = LOAD_STOP;
        }
    }
}
