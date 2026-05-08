/**
* @file Robot_Cmd_Task.c
 * @brief 机甲控制程序
 * @author Shen FeiLin
 * @date 2025/10/29
 */

#include "bsp_dwt.h"
#include "bsp_usb.h"
#include  "usb_device.h"
#include "math.h"
#include "main.h"
#include  "referee.h"
#include "DM_Motor.h"
#include "cmsis_os.h"
#include "can_comm.h"
#include "remote_control.h"
#include "message_center.h"
#include "robot_def.h"
#include "Pos.h"
#include "super_cap.h"
#include "Video_link.h"
#include "TMC.h"
#include "master_process.h"
#include "UI.h"
#include "ui_queue.h"
#include <string.h>

CCMRAM RC_ctrl_t *RC_data = NULL; // 遥控器数据,初始化时返回
CCMRAM VL_ctrl_t *VL_data = NULL; // 图传遥控器数据,初始化时返回
CCMRAM Referee_Interactive_info_t *ui_data; // UI绘制需要的机器人状态,初始化时返回
referee_info_t *Referee_data = NULL; // 裁判系统数据
SuperCapInstance *cap = NULL; // 超级电容实例
CCMRAM CANCommInstance *CANCOM; // 底盘或云台的CAN通信实例指针
CCMRAM float Timeout = 0.0f; // 遥控器信号丢失计时
CCMRAM uint8_t Timeout_flag = 0; // 遥控器信号丢失标志
CCMRAM Vision_Recv_s *vision_recv_data; // 视觉接收数据
CCMRAM volatile uint8_t shoot_speed = 15u; // 当前弹速档位,键鼠模式下会覆盖为10/15/20m/s
static uint8_t software_reset_pending = 0u; // 置位后先发送一帧reset_flag，再执行软件复位

extern TMC_To_Chassis_s *Chassis_Rec; // 底盘与云台数据结构体实例
extern TMC_To_Gimbal_s *Gimbal_Rec; // 云台与底盘数据结构体实例

/* cmd应用包含的模块实例指针和交互信息存储*/
static Chassis_Ctrl_Cmd_s chassis_cmd_send; // 发送给底盘应用的信息,包括控制信息和UI绘制相关

static Publisher_t *gimbal_cmd_pub; // 云台控制消息发布者
static Subscriber_t *gimbal_feed_sub; // 云台反馈信息订阅者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send; // 传递给云台的控制信息
static Gimbal_Upload_Data_s gimbal_fetch_data; // 从云台获取的反馈信息

static Publisher_t *shoot_cmd_pub; // 发射控制消息发布者
static Subscriber_t *shoot_feed_sub; // 发射反馈信息订阅者
static Shoot_Ctrl_Cmd_s shoot_cmd_send; // 传递给发射的控制信息
static Shoot_Upload_Data_s shoot_fetch_data; // 从发射获取的反馈信息

#ifdef MCU_GIMBAL
static TMC_To_Chassis_s Chassis_Data; // 底盘与云台数据结构体实例
#endif
#ifdef MCU_CHASSIS
static TMC_To_Gimbal_s Gimbal_Data; // 云台与底盘数据结构体实例
#endif

#ifdef MCU_CHASSIS // 如果是底盘板
CCMRAM static CANComm_Init_Config_s TMC_CANComm_Config = {
    .can_config = {
        .can_handle = &hcan1,
        .tx_id = TMC_CHASSIS_CAN_ID,
        .rx_id = TMC_GIMBAL_CAN_ID,
    },
    .send_data_len = sizeof(TMC_To_Gimbal_s),
    .recv_data_len = sizeof(TMC_To_Chassis_s)
};
#elif defined(MCU_GIMBAL)
CCMRAM static CANComm_Init_Config_s TMC_CANComm_Config = {
    .can_config = {
        .can_handle = &hcan1,
        .tx_id = TMC_GIMBAL_CAN_ID,
        .rx_id = TMC_CHASSIS_CAN_ID,
    },
    .send_data_len = sizeof(TMC_To_Chassis_s),
    .recv_data_len = sizeof(TMC_To_Gimbal_s)
};
#endif

static void Robot_Cmd_Init(void);

static void Emergency_Stop(void);

static void CalcOffsetAngle(void);

static float ResolveVisionPitchTarget(void);

static void Robot_Cmd_Serve(void);

static void RemoteControl_Cmd(void);

static void VL_keyboard_cmd(void);

static void Keyboard_Cmd(void);

static void Gimbal_Mode_Transition_Handle(void);

static float GetNearestYawTightenTarget(float current_yaw);

static uint8_t GetVisionModeByKeyCount(uint8_t key_count);

static uint8_t GetShootSpeedByKeyCount(uint8_t key_count);

static void ApplyManualFireControl(uint8_t fire_pressed, uint8_t burst_toggle_count);

static friction_mode_e GetFrictionModeByKeyCount(uint8_t key_count);

static void UpdateSoftwareResetRequest(uint8_t z_press_count, uint8_t *last_z_press_count);

static void EnforceTightenShootSafety(void);

static void ClearChassisSpeedOnModeSwitch(void);

#ifdef MCU_CHASSIS
static void Chassis_data_tidy(void);
#endif

static int16_t GetTightenMouseWz(int16_t mouse_x);

static int16_t GetTightenRockerWz(int16_t rocker_x);

/**
 * @brief 命令读取与发送FreeRTOS任务
 * @note DJI电机控制函数在此调用
 * @todo 底盘和云台的USART6波特率不一样！
 */
void CmdTask(void *argument) {
#ifdef MCU_GIMBAL
    uint8_t can_comm_send_count = 0u;
#endif

    taskENTER_CRITICAL();
    DWT_Init(168);
    MX_USB_DEVICE_Init();
    Robot_Cmd_Init();
    taskEXIT_CRITICAL();

    for (;;) {
#ifdef MCU_GIMBAL
        SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);
        SubGetMessage(shoot_feed_sub, &shoot_fetch_data);

        CalcOffsetAngle();
        Robot_Cmd_Serve();
        Gimbal_Mode_Transition_Handle();

        if (gimbal_cmd_send.gimbal_mode == GIMBAL_NONE || gimbal_fetch_data.yaw_motor_offline) {
            gimbal_cmd_send.yaw = gimbal_fetch_data.yaw_target;
        }

        EnforceTightenShootSafety();
        ClearChassisSpeedOnModeSwitch();
        Chassis_Data.Chassis_Cmd = chassis_cmd_send;
        Chassis_Data.loader_mode = shoot_cmd_send.load_mode;
        Chassis_Data.friction_mode = shoot_cmd_send.friction_mode;
        Chassis_Data.gimbal_mode = gimbal_fetch_data.gimbal_mode;
        Chassis_Data.distance_status = vision_recv_data->enemy_data.distance_status;
        Chassis_Data.isfind_enemy = vision_recv_data->enemy_data.isfind_enemy;
        Chassis_Data.shoot_speed = shoot_speed;
        Chassis_Data.reset_flag = software_reset_pending;
        Chassis_Data.vision_flag = VisionIsOnline();

#endif

        DJI_Motor_Control();
        DM_Motor_Control();
        Timeout_flag = DWT_GetTimeline_ms() - Timeout > 100.0f ? 1 : 0; // 遥控器信号丢失标志,丢失时间超过100ms则置位

#ifdef MCU_CHASSIS

        Chassis_data_tidy();

        // UI发送服务：每3ms调用一次，内部控制约100ms发送一个UI帧（符合裁判系统10Hz限制）
        UI_Queue_SendService();
        UI_Init_Check();

        // UI动态更新：约300ms触发一次，仅在初始化完成后执行
        if (UI_Queue_GetInitState() == UI_INIT_DONE) {
            UI_count++;
            if (UI_count >= 100) {
                UI_Update_Upgroup_Data();
                UI_count = 0;
            }
        }

        DM_count++;
        DM_count >= 10 ? (DM_Motor_Control(), DM_count = 0) : 0; // 达妙电机控制频率为30ms

        // SuperCap_count++;
        // SuperCap_count >= 500
        //     ? (SuperCapSend(cap, Referee_data->GameRobotState.chassis_power_limit,
        //                     Referee_data->PowerHeatData.buffer_energy, 0.9f), SuperCap_count = 0)
        //     : 0; // 超级电容控制频率为1s,根据裁判系统等级调整功率限制

        Timeout_flag == 1 ? RefereeLostCallback() : 0; // 遥控器信号丢失则调用裁判系统丢失回调函数

        CANCommSend(CANCOM, (uint8_t *) &Gimbal_Data);

#elif defined(MCU_GIMBAL)
        can_comm_send_count++;
        if (can_comm_send_count >= 2u || software_reset_pending) {
            CANCommSend(CANCOM, (uint8_t *) &Chassis_Data);
            can_comm_send_count = 0u;
        }

        if (software_reset_pending) {
            Chassis_Data.reset_flag = 0u;
            software_reset_pending = 0u;
            __set_FAULTMASK(1); //关闭所有中断
            NVIC_SystemReset(); //软件复位
        }

        PubPushMessage(shoot_cmd_pub, (void *) &shoot_cmd_send);
        PubPushMessage(gimbal_cmd_pub, (void *) &gimbal_cmd_send);
#endif

        osDelay(2);
    }
}

/**
 * @brief 机甲命令初始化
 */
static void Robot_Cmd_Init(void) {
#ifdef MCU_GIMBAL
    // 重新配置串口6波特率（图传遥控器需要921600）
    extern DMA_HandleTypeDef hdma_usart6_rx;
    extern DMA_HandleTypeDef hdma_usart6_tx;

    huart6.Init.BaudRate = 921600;
    HAL_UART_Init(&huart6);
    // HAL_UART_Init后需要重新链接DMA句柄，否则DMA接收无法工作
    __HAL_LINKDMA(&huart6, hdmarx, hdma_usart6_rx);
    __HAL_LINKDMA(&huart6, hdmatx, hdma_usart6_tx);

    /******************************************************/
    RC_data = RemoteControlInit(&huart3); // 遥控器和图传二选一
    VL_data = VLRemoteControlInit(&huart6);
    vision_recv_data = VisionInit();
    /******************************************************/

    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12, GPIO_PIN_RESET); //USB DP拉低，重新枚举

#elif defined(MCU_CHASSIS)
    // 重新配置串口6波特率（裁判系统需要115200）
    extern DMA_HandleTypeDef hdma_usart6_rx;
    extern DMA_HandleTypeDef hdma_usart6_tx;

    huart6.Init.BaudRate = 115200;
    HAL_UART_Init(&huart6);
    // HAL_UART_Init后需要重新链接DMA句柄，否则DMA接收无法工作
    __HAL_LINKDMA(&huart6, hdmarx, hdma_usart6_rx);
    __HAL_LINKDMA(&huart6, hdmatx, hdma_usart6_tx);

    Referee_data = UIInit(&huart6, ui_data);

    // 初始化UI发送队列
    UI_Queue_Init();

    SuperCap_Init_Config_s cap_conf = {
        .enableDCDC = 1, //允许开启DCDC电路
        .systemRestart = 0, //不需要重启系统
        .clearError = 1, //需要清除错误
        .enableActiveChargingLimit = 1, //禁用主动充电限制---类似于手机电池保护每次充到95%就停止充电，不让电池充满从而延长超级电容使用寿命
        .useNewFeedbackMessage = 1, //使用新消息

        .can_config = {
            .can_handle = &hcan1,
            .tx_id = 0x61, // 超级电容默认接收id
            .rx_id = 0x52, // 超级电容默认发送id,注意tx和rx在其他人看来是反的
        }
    };
    cap = SuperCapInit(&cap_conf); // 超级电容初始化

#endif
    Timeout = DWT_GetTimeline_ms(); // 超时同时判断图传和裁判系统是否失灵
    CANCOM = CANCommInit(&TMC_CANComm_Config);

    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    shoot_cmd_pub = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    shoot_feed_sub = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
    gimbal_cmd_send.gimbal_mode = GIMBAL_TIGHTEN;
    gimbal_cmd_send.yaw = YAW_TIGHTEN_ANGLE;
    gimbal_cmd_send.pitch = 0.0f;
}

/**
 * @brief 根据gimbal app传回的当前电机角度计算和零位的误差
 *        单圈绝对角度的范围是0~360,说明文档中有图示
 */
static void CalcOffsetAngle(void) {
    static float angle;
    angle = gimbal_fetch_data.yaw_motor_single_round_angle;

    // 计算 offset_angle (0~360度)
    chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    if (chassis_cmd_send.offset_angle < 0)
        chassis_cmd_send.offset_angle += 360.0f;

    // 转换为 -180~180 度，方便PID控制
    if (chassis_cmd_send.offset_angle > 180.0f)
        gimbal_cmd_send.angle_offset_g = chassis_cmd_send.offset_angle - 360.0f;
    else
        gimbal_cmd_send.angle_offset_g = chassis_cmd_send.offset_angle;
}

static float GetNearestYawTightenTarget(float current_yaw) {
    return YAW_TIGHTEN_ANGLE + roundf((current_yaw - YAW_TIGHTEN_ANGLE) / GIMBAL_ANGLE_PERIOD) * GIMBAL_ANGLE_PERIOD;
}

static float ResolveVisionPitchTarget(void) {
    return vision_recv_data->gimbal_data.pitch + gimbal_fetch_data.pitch_down_total_angle;
}

static void Gimbal_Mode_Transition_Handle(void) {
    static gimbal_mode_e last_gimbal_mode = GIMBAL_TIGHTEN;
    static uint16_t release_hold_ticks = 0;
    static float latched_tighten_yaw_target = YAW_TIGHTEN_ANGLE;
    uint8_t non_tighten_request = gimbal_cmd_send.gimbal_mode != GIMBAL_NONE && gimbal_cmd_send.gimbal_mode !=
                                  GIMBAL_TIGHTEN;
    float current_yaw_reference = gimbal_fetch_data.yaw_motor_offline
                                      ? gimbal_cmd_send.yaw
                                      : gimbal_fetch_data.yaw_motor_total_angle;

    if (gimbal_cmd_send.gimbal_mode == GIMBAL_TIGHTEN && last_gimbal_mode != GIMBAL_TIGHTEN) {
        latched_tighten_yaw_target = GetNearestYawTightenTarget(current_yaw_reference);
        release_hold_ticks = 0u;
    }

    if (gimbal_cmd_send.gimbal_mode == GIMBAL_TIGHTEN) {
        gimbal_cmd_send.yaw = latched_tighten_yaw_target;
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
    } else if (non_tighten_request && last_gimbal_mode == GIMBAL_TIGHTEN && release_hold_ticks == 0u) {
        release_hold_ticks = (uint16_t) (PITCH_DOWN_RELEASE_DURATION * 500.0f) + 20u;
    }

    if (release_hold_ticks > 0 && non_tighten_request) {
        gimbal_cmd_send.yaw = latched_tighten_yaw_target;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
        release_hold_ticks--;
    }

    last_gimbal_mode = gimbal_cmd_send.gimbal_mode;
}

static uint8_t GetVisionModeByKeyCount(uint8_t key_count) {
    switch (key_count % 3u) {
        case 1u:
            return 1u;
        case 2u:
            return 2u;
        default:
            return 0u;
    }
}

static uint8_t GetShootSpeedByKeyCount(uint8_t key_count) {
    switch (key_count % 3u) {
        case 1u:
            return 15u;
        case 2u:
            return 20u;
        default:
            return 10u;
    }
}

static void ApplyManualFireControl(uint8_t fire_pressed, uint8_t burst_toggle_count) {
    if (gimbal_cmd_send.gimbal_mode == GIMBAL_TIGHTEN) {
        shoot_cmd_send.load_mode = LOAD_STOP;
        return;
    }

    if (!fire_pressed) {
        shoot_cmd_send.load_mode = LOAD_STOP;
        return;
    }

    switch (burst_toggle_count % 2u) {
        case 0u:
            shoot_cmd_send.load_mode = LOAD_1_BULLET;
            break;
        default:
            shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
            break;
    }
    shoot_cmd_send.shoot_rate = 15.0f; // 固定射速，后续可以改为根据按键次数切换不同射速
}

static friction_mode_e GetFrictionModeByKeyCount(uint8_t key_count) {
    if (gimbal_cmd_send.gimbal_mode == GIMBAL_TIGHTEN) {
        return FRICTION_OFF;
    }

    return key_count % 2u == 0u ? FRICTION_OFF : FRICTION_ON;
}

static int16_t GetTightenMouseWz(int16_t mouse_x) {
    return (int16_t) (-(float) mouse_x / 660.0f * (float) MAX_FOLLOW_SPEED);
}

static int16_t GetTightenRockerWz(int16_t rocker_x) {
    return (int16_t) (-(float) rocker_x / 660.0f * (float) MAX_FOLLOW_SPEED);
}

static void UpdateSoftwareResetRequest(uint8_t z_press_count, uint8_t *last_z_press_count) {
    if (z_press_count != *last_z_press_count) {
        software_reset_pending = 1u;
        *last_z_press_count = z_press_count;
    }
}

static void EnforceTightenShootSafety(void) {
    if (gimbal_cmd_send.gimbal_mode == GIMBAL_TIGHTEN) {
        shoot_cmd_send.shoot_mode = SHOOT_OFF;
        shoot_cmd_send.load_mode = LOAD_STOP;
        shoot_cmd_send.friction_mode = FRICTION_OFF;
    }
}

static void ClearChassisSpeedOnModeSwitch(void) {
    static uint8_t initialized = 0u;
    static uint8_t last_chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
    static uint8_t last_gimbal_tighten_state = 1u;
    uint8_t current_chassis_mode = chassis_cmd_send.chassis_mode;
    uint8_t current_gimbal_tighten_state = (gimbal_fetch_data.gimbal_mode == GIMBAL_TIGHTEN) ? 1u : 0u;

    if (!initialized) {
        last_chassis_mode = current_chassis_mode;
        last_gimbal_tighten_state = current_gimbal_tighten_state;
        initialized = 1u;
        return;
    }

    if (current_chassis_mode != last_chassis_mode || current_gimbal_tighten_state != last_gimbal_tighten_state) {
        chassis_cmd_send.vx = 0;
        chassis_cmd_send.vy = 0;
        chassis_cmd_send.wz = 0;
    }

    last_chassis_mode = current_chassis_mode;
    last_gimbal_tighten_state = current_gimbal_tighten_state;
}

#ifdef MCU_CHASSIS
static void Chassis_data_tidy(void) {
    if (Referee_data == NULL) {
        memset(&Gimbal_Data.Shoot_Upload_Data, 0, sizeof(Gimbal_Data.Shoot_Upload_Data));
        return;
    }

    Gimbal_Data.Shoot_Upload_Data.heat = Referee_data->PowerHeatData.shooter_17mm_1_barrel_heat;
    Gimbal_Data.Shoot_Upload_Data.reference_online_state = Referee_data->referee_online_state;
    Gimbal_Data.Shoot_Upload_Data.robot_level = Referee_data->GameRobotState.robot_level;
    Gimbal_Data.Shoot_Upload_Data.shooter_barrel_cooling_value =
        Referee_data->GameRobotState.shooter_barrel_cooling_value;
    Gimbal_Data.Shoot_Upload_Data.shooter_heat_limit = Referee_data->GameRobotState.shooter_barrel_heat_limit;
    Gimbal_Data.Shoot_Upload_Data.robot_color = Referee_data->referee_id.Robot_Color;
    Gimbal_Data.Shoot_Upload_Data.bullet_speed = (uint16_t) (Referee_data->ShootData.initial_speed * 1024.0f);
    Gimbal_Data.Shoot_Upload_Data.rec_vx = 0;
    Gimbal_Data.Shoot_Upload_Data.rec_vy = 0;
}
#endif

/**
 * @brief 命令解析
 * @todo 紧急停止，键盘控制，遥控器控制三种模式的切换还没完成
 */
static void Robot_Cmd_Serve(void) {
    static uint8_t startup_tighten_latch = 1u;
    uint8_t rc_emergency = RC_data != NULL && switch_is_down(RC_data[TEMP].rc.switch_left) &&
                           switch_is_down(RC_data[TEMP].rc.switch_right);
    uint8_t vl_emergency = VL_data != NULL && VL_data[TEMP].rc.mode_sw == 2;
    uint8_t rc_release_startup = RC_data != NULL && !rc_emergency;
    uint8_t vl_release_startup = VL_data != NULL && VL_data[TEMP].rc.mode_sw != 2;

    // 左右两杆均拨下，紧急断电
    if (startup_tighten_latch && (rc_release_startup || vl_release_startup)) {
        startup_tighten_latch = 0u;
    }

    if (!startup_tighten_latch && (rc_emergency || vl_emergency || Timeout_flag)) {
        Emergency_Stop();
        return;
    }
    // 射击指令
    shoot_cmd_send.shoot_mode = SHOOT_ON;
    VisionSetControlState(0u, 0u);
#ifdef MCU_GIMBAL
    Chassis_Data.UI_reset = 0u;
    Chassis_Data.superpower_flag = 0u;
    Chassis_Data.shoot_speed = shoot_speed;
#endif
    LED_Red_Down;

    if (startup_tighten_latch) {
        chassis_cmd_send.vx = 0;
        chassis_cmd_send.vy = 0;
        chassis_cmd_send.wz = 0;
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
        gimbal_cmd_send.gimbal_mode = GIMBAL_TIGHTEN;
        return;
    }

    // 键盘控制优先级最高，图传遥控器次之，遥控器优先级最低
    if ((switch_is_up(RC_data[TEMP].rc.switch_left) && switch_is_up(RC_data[TEMP].rc.switch_right)))
        Keyboard_Cmd();
    else if (VL_data != NULL && VL_data[TEMP].rc.mode_sw == 1)
        VL_keyboard_cmd();
    else
        RemoteControl_Cmd();
}

/**
 * @brief 紧急断电状态
 */
static void Emergency_Stop(void) {
    chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
    gimbal_cmd_send.gimbal_mode = GIMBAL_NONE;
    shoot_cmd_send.shoot_mode = SHOOT_OFF;
    shoot_cmd_send.friction_mode = FRICTION_OFF;
    shoot_cmd_send.load_mode = LOAD_STOP;
    VisionSetControlState(0u, 0u);
    LED_Red_Up;
}

/**
 * @brief 键盘控制指令解析
 * @note 这是基于图传的键鼠控制
 */
static void VL_keyboard_cmd(void) {
    static uint8_t last_z_press_count = 0u;
    const uint16_t max_speed = 3500;
    uint8_t vision_mode = GetVisionModeByKeyCount(VL_data[TEMP].key_count[KEY_PRESS][Key_X]);
    if (VL_data[TEMP].key_count[KEY_PRESS][Key_C] % 2 == 1) // C键切换云台缩紧模式
        gimbal_cmd_send.gimbal_mode = GIMBAL_TIGHTEN;
    else
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;

    if (gimbal_cmd_send.gimbal_mode == GIMBAL_TIGHTEN) {
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
    } else if (VL_data[TEMP].key_count[KEY_PRESS][Key_Ctrl] % 2 == 1) // 按下Ctrl键底盘独立控制
        chassis_cmd_send.chassis_mode = CHASSIS_INDEPENDENCE;
    else if (VL_data[TEMP].key_count[KEY_PRESS][Key_Shift] % 2 == 1) // 按下Shift键底盘小陀螺
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
    else
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW; // 默认底盘跟随云台yaw

    //WASD方向平移
    if (VL_data[TEMP].key[KEY_PRESS].w) {
        if (chassis_cmd_send.vy < max_speed) {
            chassis_cmd_send.vy += 20u;
        } else {
            chassis_cmd_send.vy = max_speed;
        }
    } else if (VL_data[TEMP].key[KEY_PRESS].s) {
        if (chassis_cmd_send.vy > -max_speed) {
            chassis_cmd_send.vy -= 20u;
        } else {
            chassis_cmd_send.vy = -max_speed;
        }
    } else {
        chassis_cmd_send.vy = 0;
    }

    if (chassis_cmd_send.chassis_mode == CHASSIS_INDEPENDENCE) {
        if (VL_data[TEMP].key[KEY_PRESS].d) {
            if (chassis_cmd_send.wz < MAX_FOLLOW_SPEED) {
                chassis_cmd_send.wz += 4;
            } else {
                chassis_cmd_send.wz = MAX_FOLLOW_SPEED;
            }
        } else if (VL_data[TEMP].key[KEY_PRESS].a) {
            if (chassis_cmd_send.wz > -MAX_FOLLOW_SPEED) {
                chassis_cmd_send.wz -= 4;
            } else {
                chassis_cmd_send.wz = -MAX_FOLLOW_SPEED;
            }
        } else {
            chassis_cmd_send.wz = 0;
        }
    } else {
        // 按D键vx减小(向右), 按A键vx增大(向左)
        if (VL_data[TEMP].key[KEY_PRESS].d) {
            if (chassis_cmd_send.vx > -max_speed) {
                chassis_cmd_send.vx -= 20u;
            } else {
                chassis_cmd_send.vx = -max_speed;
            }
        } else if (VL_data[TEMP].key[KEY_PRESS].a) {
            if (chassis_cmd_send.vx < max_speed) {
                chassis_cmd_send.vx += 20u;
            } else {
                chassis_cmd_send.vx = max_speed;
            }
        } else {
            chassis_cmd_send.vx = 0;
        }
    }

    if (gimbal_cmd_send.gimbal_mode == GIMBAL_TIGHTEN) {
        chassis_cmd_send.wz = GetTightenMouseWz(VL_data[TEMP].mouse.x);
    }

    VisionSetControlState(0u, vision_mode);
    ApplyManualFireControl(VL_data[TEMP].mouse.mouse_left, VL_data[TEMP].key_count[KEY_PRESS][Key_R]);

    if (gimbal_cmd_send.gimbal_mode != GIMBAL_TIGHTEN) {
        if (VL_data[TEMP].mouse.mouse_right) //长按鼠标右键进入自瞄模式
        {
            uint8_t vision_online = VisionIsOnline();
            gimbal_cmd_send.gimbal_mode = GIMBAL_VISION; //只做头部跟随
            chassis_cmd_send.chassis_mode = CHASSIS_INDEPENDENCE;
            if (vision_online) {
                gimbal_cmd_send.yaw = vision_recv_data->gimbal_data.yaw;
                gimbal_cmd_send.pitch = ResolveVisionPitchTarget();
            } else {
                gimbal_cmd_send.yaw -= (float) VL_data[TEMP].mouse.x / 660 * 2.4f;
                gimbal_cmd_send.pitch -= (float) VL_data[TEMP].mouse.y / 660 * 1.3f;
            }
            gimbal_cmd_send.pitch = Angle_limit(gimbal_cmd_send.pitch, PITCH_MAX_ANGLE, PITCH_MIN_ANGLE);
            if (vision_online) {
                shoot_cmd_send.shoot_rate = (float) vision_recv_data->gimbal_data.loader_frequency;
                VisionSetControlState(1u, vision_mode);
            }
            if (vision_online && VL_data[TEMP].mouse.mouse_left && vision_recv_data->gimbal_data.fire_flag)
            //鼠标左键短按单发，长按连发，前提是视觉模块的fire_flag为真
            {
                ApplyManualFireControl(VL_data[TEMP].mouse.mouse_left, VL_data[TEMP].key_count[KEY_PRESS][Key_R]);
            } else if (vision_online) shoot_cmd_send.load_mode = LOAD_STOP; //停止发射
        } else {
            gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
            gimbal_cmd_send.yaw -= (float) VL_data[TEMP].mouse.x / 660 * 2.4f; //3.5
            gimbal_cmd_send.pitch -= (float) VL_data[TEMP].mouse.y / 660 * 1.3f; //-2.5
            gimbal_cmd_send.pitch = Angle_limit(gimbal_cmd_send.pitch, PITCH_MAX_ANGLE, PITCH_MIN_ANGLE);
        }
    }

    shoot_cmd_send.friction_mode = GetFrictionModeByKeyCount(VL_data[TEMP].key_count[KEY_PRESS][Key_F]);
    shoot_speed = GetShootSpeedByKeyCount(VL_data[TEMP].key_count[KEY_PRESS][Key_V]);
    Chassis_Data.superpower_flag = (uint8_t) (VL_data[TEMP].key_count[KEY_PRESS][Key_B] % 2u);
    Chassis_Data.UI_reset = (uint8_t) VL_data[TEMP].key[KEY_PRESS].g;
    UpdateSoftwareResetRequest(VL_data[TEMP].key_count[KEY_PRESS][Key_Z], &last_z_press_count);
}

/**
 * @brief 遥控器控制指令解析
 */
static void RemoteControl_Cmd(void) {
    VisionSetControlState(0u, 0u);
    if (gimbal_cmd_send.gimbal_mode == GIMBAL_TIGHTEN) {
        shoot_cmd_send.friction_mode = FRICTION_OFF;
    } else {
        abs(RC_data[TEMP].rc.dial) > 100 // 拨盘大于100 启动电机
            ? (shoot_cmd_send.friction_mode = FRICTION_ON)
            : (shoot_cmd_send.friction_mode = FRICTION_OFF);
    }
    if (RC_data[TEMP].rc.dial > 500) {
        shoot_cmd_send.load_mode = LOAD_BURSTFIRE; //连发
        shoot_cmd_send.shoot_rate = 15.0f; //单位Hz
    } else if (RC_data[TEMP].rc.dial < -500)
        shoot_cmd_send.load_mode = LOAD_1_BULLET; //单发
    else
        shoot_cmd_send.load_mode = LOAD_STOP; //停止

    //左杆在下，右杆在中，底盘随云台控制
    if (switch_is_down(RC_data[TEMP].rc.switch_left) && switch_is_mid(RC_data[TEMP].rc.switch_right)) {
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    }
    //左杆在下，右杆在上，底盘小陀螺，云台自由模式
    else if (switch_is_down(RC_data[TEMP].rc.switch_left) && switch_is_up(RC_data[TEMP].rc.switch_right)) {
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    }
    // 左杆在中，右杆在上，底盘独立控制，云台自由模式
    else if (switch_is_mid(RC_data[TEMP].rc.switch_left) && switch_is_up(RC_data[TEMP].rc.switch_right)) {
        chassis_cmd_send.chassis_mode = CHASSIS_INDEPENDENCE;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    }
    // 左杆在中，右杆在中，云台锁紧模式
    else if (switch_is_mid(RC_data[TEMP].rc.switch_left) && switch_is_mid(RC_data[TEMP].rc.switch_right)) {
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
        gimbal_cmd_send.gimbal_mode = GIMBAL_TIGHTEN;
    }
    // 左杆在上，右杆在中，自瞄模式
    else if (switch_is_up(RC_data[TEMP].rc.switch_left) && switch_is_mid(RC_data[TEMP].rc.switch_right)) {
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
        gimbal_cmd_send.gimbal_mode = GIMBAL_VISION;
    }

    // 跟随云台转动，右拨杆控制云台俯仰和偏航
    if (gimbal_cmd_send.gimbal_mode == GIMBAL_GYRO_MODE) {
        gimbal_cmd_send.yaw -= 0.0008f * (float) RC_data[TEMP].rc.rocker_r_;
        gimbal_cmd_send.pitch -= 0.0006f * (float) RC_data[TEMP].rc.rocker_r1;
        gimbal_cmd_send.pitch = Angle_limit(gimbal_cmd_send.pitch, PITCH_MAX_ANGLE, PITCH_MIN_ANGLE);
    } else if (gimbal_cmd_send.gimbal_mode == GIMBAL_TIGHTEN) {
        chassis_cmd_send.wz = GetTightenRockerWz(RC_data[TEMP].rc.rocker_r_);
    }
    // 视觉模式
    else if (gimbal_cmd_send.gimbal_mode == GIMBAL_VISION) {
        uint8_t vision_online = VisionIsOnline();
        chassis_cmd_send.chassis_mode = CHASSIS_INDEPENDENCE;
        if (vision_online) {
            gimbal_cmd_send.yaw = vision_recv_data->gimbal_data.yaw;
            gimbal_cmd_send.pitch = ResolveVisionPitchTarget();
        } else {
            gimbal_cmd_send.yaw -= 0.0008f * (float) RC_data[TEMP].rc.rocker_r_;
            gimbal_cmd_send.pitch -= 0.0006f * (float) RC_data[TEMP].rc.rocker_r1;
        }
        gimbal_cmd_send.pitch = Angle_limit(gimbal_cmd_send.pitch, PITCH_MAX_ANGLE, PITCH_MIN_ANGLE);
    }
    // 平移速度设置
    chassis_cmd_send.vy = ((float) RC_data[TEMP].rc.rocker_l1) * 0.151f * 30.0f; // 最高3m/s
    chassis_cmd_send.chassis_mode == CHASSIS_INDEPENDENCE
        ? (chassis_cmd_send.wz = RC_data[TEMP].rc.rocker_l_ * 0.00151f * 100.0f) // 最高1转/s
        : (chassis_cmd_send.vx = -(float) RC_data[TEMP].rc.rocker_l_ * 0.151f * 30.0f); // 最高3m/s
}

/**
 * @brief DR16键盘控制指令解析
 */
static void Keyboard_Cmd(void) {
    static uint8_t last_z_press_count = 0u;
    const uint16_t max_speed = 3500;
    uint8_t vision_mode = GetVisionModeByKeyCount(RC_data[TEMP].key_count[KEY_PRESS][Key_X]);
    if (RC_data[TEMP].key_count[KEY_PRESS][Key_C] % 2 == 1) // C键切换云台缩紧模式
        gimbal_cmd_send.gimbal_mode = GIMBAL_TIGHTEN;
    else
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;

    if (gimbal_cmd_send.gimbal_mode == GIMBAL_TIGHTEN) {
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
    } else if (RC_data[TEMP].key_count[KEY_PRESS][Key_Ctrl] % 2 == 1) // 按下Ctrl键底盘独立控制
        chassis_cmd_send.chassis_mode = CHASSIS_INDEPENDENCE;
    else if (RC_data[TEMP].key_count[KEY_PRESS][Key_Shift] % 2 == 1) // 按下Shift键底盘小陀螺
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
    else
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW; // 默认底盘跟随云台yaw

    //WASD方向平移
    if (RC_data[TEMP].key[KEY_PRESS].w) {
        if (chassis_cmd_send.vy < max_speed) {
            chassis_cmd_send.vy += 20u;
        } else {
            chassis_cmd_send.vy = max_speed;
        }
    } else if (RC_data[TEMP].key[KEY_PRESS].s) {
        if (chassis_cmd_send.vy > -max_speed) {
            chassis_cmd_send.vy -= 20u;
        } else {
            chassis_cmd_send.vy = -max_speed;
        }
    } else {
        chassis_cmd_send.vy = 0;
    }

    if (chassis_cmd_send.chassis_mode == CHASSIS_INDEPENDENCE) {
        if (RC_data[TEMP].key[KEY_PRESS].d) {
            if (chassis_cmd_send.wz < MAX_FOLLOW_SPEED) {
                chassis_cmd_send.wz += 4;
            } else {
                chassis_cmd_send.wz = MAX_FOLLOW_SPEED;
            }
        } else if (RC_data[TEMP].key[KEY_PRESS].a) {
            if (chassis_cmd_send.wz > -MAX_FOLLOW_SPEED) {
                chassis_cmd_send.wz -= 4;
            } else {
                chassis_cmd_send.wz = -MAX_FOLLOW_SPEED;
            }
        } else {
            chassis_cmd_send.wz = 0;
        }
    } else {
        // 按D键vx减小(向右), 按A键vx增大(向左)
        if (RC_data[TEMP].key[KEY_PRESS].d) {
            if (chassis_cmd_send.vx > -max_speed) {
                chassis_cmd_send.vx -= 20u;
            } else {
                chassis_cmd_send.vx = -max_speed;
            }
        } else if (RC_data[TEMP].key[KEY_PRESS].a) {
            if (chassis_cmd_send.vx < max_speed) {
                chassis_cmd_send.vx += 20u;
            } else {
                chassis_cmd_send.vx = max_speed;
            }
        } else {
            chassis_cmd_send.vx = 0;
        }
    }

    if (gimbal_cmd_send.gimbal_mode == GIMBAL_TIGHTEN) {
        chassis_cmd_send.wz = GetTightenMouseWz(RC_data[TEMP].mouse.x);
    }

    VisionSetControlState(0u, vision_mode);
    ApplyManualFireControl(RC_data[TEMP].mouse.press_l, RC_data[TEMP].key_count[KEY_PRESS][Key_R]);

    if (gimbal_cmd_send.gimbal_mode != GIMBAL_TIGHTEN) {
        if (RC_data[TEMP].mouse.press_r) //长按鼠标右键进入自瞄模式
        {
            uint8_t vision_online = VisionIsOnline();
            gimbal_cmd_send.gimbal_mode = GIMBAL_VISION; //只做头部跟随
            chassis_cmd_send.chassis_mode = CHASSIS_INDEPENDENCE;
            if (vision_online) {
                gimbal_cmd_send.yaw = vision_recv_data->gimbal_data.yaw;
                gimbal_cmd_send.pitch = ResolveVisionPitchTarget();
            } else {
                gimbal_cmd_send.yaw -= (float) RC_data[TEMP].mouse.x / 660 * 2.4f;
                gimbal_cmd_send.pitch -= (float) RC_data[TEMP].mouse.y / 660 * 1.3f;
            }
            gimbal_cmd_send.pitch = Angle_limit(gimbal_cmd_send.pitch, PITCH_MAX_ANGLE, PITCH_MIN_ANGLE);
            if (vision_online) {
                shoot_cmd_send.shoot_rate = (float) vision_recv_data->gimbal_data.loader_frequency;
                VisionSetControlState(1u, vision_mode);
            }
            if (vision_online && RC_data[TEMP].mouse.press_l && vision_recv_data->gimbal_data.fire_flag)
            //鼠标左键短按单发，长按连发，前提是视觉模块的fire_flag为真
            {
                ApplyManualFireControl(RC_data[TEMP].mouse.press_l, RC_data[TEMP].key_count[KEY_PRESS][Key_R]);
            } else if (vision_online) shoot_cmd_send.load_mode = LOAD_STOP; //停止发射
        } else {
            gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
            gimbal_cmd_send.yaw -= (float) RC_data[TEMP].mouse.x / 660 * 2.4f; //3.5
            gimbal_cmd_send.pitch -= (float) RC_data[TEMP].mouse.y / 660 * 1.3f; //-2.5
            gimbal_cmd_send.pitch = Angle_limit(gimbal_cmd_send.pitch, PITCH_MAX_ANGLE, PITCH_MIN_ANGLE);
        }
    }

    shoot_cmd_send.friction_mode = GetFrictionModeByKeyCount(RC_data[TEMP].key_count[KEY_PRESS][Key_F]);
    shoot_speed = GetShootSpeedByKeyCount(RC_data[TEMP].key_count[KEY_PRESS][Key_V]);
    Chassis_Data.superpower_flag = (uint8_t) (RC_data[TEMP].key_count[KEY_PRESS][Key_B] % 2u);
    Chassis_Data.UI_reset = (uint8_t) RC_data[TEMP].key[KEY_PRESS].g;
    UpdateSoftwareResetRequest(RC_data[TEMP].key_count[KEY_PRESS][Key_Z], &last_z_press_count);
}

// /**
//  * @brief 裁判系统数据整理
//  * @note 目前主要是把裁判系统的数据转发到CAN总线上，后续可以根据需要增加一些处理逻辑
//  */
// static void Chassis_data_tidy(void) {
//     // 将裁判系统的数据转发到CAN总线上，后续可以根据需要增加一些处理逻辑
//     Gimbal_Data.CH_Gyro[0] = (int8_t) INS.Gyro[0];
//     Gimbal_Data.CH_Gyro[1] = (int8_t) INS.Gyro[1];
//     Gimbal_Data.CH_Gyro[2] = (int8_t) INS.Gyro[2];
//     Gimbal_Data.CH_Pitch = (int8_t) INS.Pitch;
//     Gimbal_Data.CH_Roll = (int8_t) INS.Roll;
//     Gimbal_Data.Shoot_Upload_Data.heat = Referee_data->PowerHeatData.shooter_17mm_1_barrel_heat;
//     Gimbal_Data.Shoot_Upload_Data.reference_online_state = Referee_data->referee_online_state;
//     Gimbal_Data.Shoot_Upload_Data.robot_level = Referee_data->GameRobotState.robot_level;
//     Gimbal_Data.Shoot_Upload_Data.shooter_barrel_cooling_value = Referee_data->GameRobotState.
//             shooter_barrel_cooling_value;
//     Gimbal_Data.Shoot_Upload_Data.shooter_heat_limit = Referee_data->GameRobotState.shooter_barrel_heat_limit;
// }
//
// /**
//  * @brief UI数据更新函数
//  * @note 收集动态数据并调用UI更新函数
//  */
// static void UI_Update_Upgroup_Data(void) {
//     // 状态字符串
//     const char *chassis_mode_str = "NONE";
//     const char *gimbal_mode_str = "NONE";
//     const char *friction_mode_str = "OFF";
//
//     switch (Chassis_Rec->friction_mode) {
//         case FRICTION_OFF: friction_mode_str = "OFF";
//             break;
//         case FRICTION_ON: friction_mode_str = "ON ";
//             break;
//         default: break;
//     }
//     switch (Chassis_Rec->gimbal_mode) {
//         case GIMBAL_ZERO_FORCE: gimbal_mode_str = "ZERO";
//             break;
//         case GIMBAL_GYRO_MODE: gimbal_mode_str = "GYRO";
//             break;
//         case GIMBAL_VISION: gimbal_mode_str = "VIS ";
//             break;
//         case GIMBAL_FREE_MODE: gimbal_mode_str = "FREE";
//             break;
//         default: break;
//     }
//     switch (Chassis_Rec->Chassis_Cmd.chassis_mode) {
//         case CHASSIS_ZERO_FORCE: chassis_mode_str = "ZERO";
//             break;
//         case CHASSIS_FOLLOW_GIMBAL_YAW: chassis_mode_str = "FOLL";
//             break;
//         case CHASSIS_INDEPENDENCE: chassis_mode_str = "INDE";
//             break;
//         case CHASSIS_ROTATE: chassis_mode_str = "ROTA";
//             break;
//         default: break;
//     }
//
//     // 计算图形数据
//     uint32_t track_head = (uint32_t) Chassis_Rec->Chassis_Cmd.a_track_head;
//     uint32_t track_back = (uint32_t) Chassis_Rec->Chassis_Cmd.a_track_back;
//     int32_t speed_target = Chassis_Rec->speed_target * 100;
//
//     uint32_t power_end_x = 773;
//     uint32_t capenergy_end_x = 773;
//     if (Referee_data->referee_online_state && Referee_data->GameRobotState.robot_id != 0) {
//         power_end_x = (uint32_t) (cap->cap_msg.chassisPower / (float) Referee_data->GameRobotState.chassis_power_limit *
//                                   400.0f) + 773;
//         capenergy_end_x = (uint32_t) (cap->cap_msg.capEnergy * 4.0f) + 773;
//     }
//
//     // 调用UI更新函数
//     UI_Update_Upgroup(track_head, track_back, speed_target, power_end_x, capenergy_end_x,
//                       chassis_mode_str, gimbal_mode_str, friction_mode_str);
// }
//
// /**
//  * @brief UI初始化检测
//  */
// static void UI_Init_Check(void) {
//     UI_Init_State_t init_state = UI_Queue_GetInitState();
//
//     if (init_state == UI_INIT_DONE) return; // 已完成初始化
//
//     if (init_state == UI_INIT_SENDING) {
//         // 正在发送初始化帧，等待队列清空
//         if (UI_Queue_IsEmpty()) {
//             UI_Queue_SetInitState(UI_INIT_DONE); // 标记初始化完成
//         }
//         return;
//     }
//
//     // init_state == UI_INIT_NOT_STARTED，等待裁判系统上线
//     if (Referee_data->referee_online_state) {
//         DetermineRobotID();
//
//         // UI初始化：3组图形分别发送
//         UI_Init_Midgroup();
//         UI_Init_Upgroup();
//         UI_Init_Ungroup();
//
//         UI_Queue_SetInitState(UI_INIT_SENDING); // 进入发送阶段
//     }
// }
