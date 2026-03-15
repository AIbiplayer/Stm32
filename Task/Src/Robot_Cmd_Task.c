/**
* @file Robot_Cmd_Task.c
 * @brief 机甲控制程序
 * @author Shen FeiLin
 * @date 2025/10/29
 */

#include "bsp_dwt.h"
#include "bsp_usb.h"
#include  "usb_device.h"
#include "main.h"
#include  "referee.h"
#include "DM_Motor.h"
#include "cmsis_os.h"
#include "can_comm.h"
#include "remote_control.h"
#include "message_center.h"
#include "robot_def.h"
#include "UI.h"
#include "super_cap.h"
#include "Video_link.h"
#include "TMC.h"

CCMRAM RC_ctrl_t *RC_data; // 遥控器数据,初始化时返回
CCMRAM VL_ctrl_t *VL_data; // 图传遥控器数据,初始化时返回
CCMRAM CANCommInstance *CANCOM; // 底盘或云台的CAN通信实例指针
CCMRAM float Timeout = 0.0f; // 遥控器信号丢失计时
CCMRAM uint8_t Timeout_flag = 0; // 遥控器信号丢失标志
CCMRAM referee_info_t *Referee_data; // 裁判系统数据
CCMRAM PID_Typedef UPPID; // 上台阶履带PID
CCMRAM Vision_Recv_s *vision_recv_data; // 视觉接收数据

extern INS_t INS; // IMU数据,包含底盘的姿态和角速度等信息
extern TMC_To_Gimbal_s *Gimbal_Rec; // 云台与底盘数据结构体实例

/* cmd应用包含的模块实例指针和交互信息存储*/
static Chassis_Ctrl_Cmd_s chassis_cmd_send; // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Chassis_Upload_Data_s chassis_fetch_data; // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等

static Publisher_t *gimbal_cmd_pub; // 云台控制消息发布者
static Subscriber_t *gimbal_feed_sub; // 云台反馈信息订阅者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send; // 传递给云台的控制信息
static Gimbal_Upload_Data_s gimbal_fetch_data; // 从云台获取的反馈信息

static Publisher_t *shoot_cmd_pub; // 发射控制消息发布者
static Subscriber_t *shoot_feed_sub; // 发射反馈信息订阅者
static Shoot_Ctrl_Cmd_s shoot_cmd_send; // 传递给发射的控制信息
static Shoot_Upload_Data_s shoot_fetch_data; // 从发射获取的反馈信息

static TMC_To_Chassis_s Chassis_Data; // 底盘与云台数据结构体实例
static TMC_To_Gimbal_s Gimbal_Data; // 云台与底盘数据结构体实例

static SuperCapInstance *cap; // 超级电容
static uint8_t SuperCap_count; // 超级电容控制计数器
static uint8_t DM_count; // 达妙电机控制计数器
static Referee_Interactive_info_t ui_data; // UI数据，将底盘中的数据传入此结构体的对应变量中，UI会自动检测是否变化，对应显示UI

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
#elifdef MCU_GIMBAL
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

static void Robot_Cmd_Serve(void);

static void RemoteControl_Cmd(void);

static void Keyboard_Cmd(void);

/**
 * @brief 命令读取与发送FreeRTOS任务
 * @note DJI电机控制函数在此调用
 * @todo 底盘和云台的USART6波特率不一样！
 */
void CmdTask(void *argument) {
    taskENTER_CRITICAL();
    DWT_Init(168);
    MX_USB_DEVICE_Init();
    Robot_Cmd_Init();
    taskEXIT_CRITICAL();

    for (;;) {
#ifdef MCU_GIMBAL
        SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);
        SubGetMessage(shoot_feed_sub, (void *) &shoot_fetch_data);

        CalcOffsetAngle();
        Robot_Cmd_Serve();
        Chassis_Data.Chassis_Cmd = chassis_cmd_send;
#endif
        DJI_Motor_Control();
        Timeout_flag = DWT_GetTimeline_ms() - Timeout > 100.0f ? 1 : 0; // 遥控器信号丢失标志,丢失时间超过100ms则置位

#ifdef MCU_CHASSIS
        DM_count++;
        DM_count >= 15 ? (DM_Motor_Control(), DM_count = 0) : 0; // 达妙电机控制频率为30ms

        // SuperCap_count++;
        // SuperCap_count >= 250 ? (SuperCapSend(cap, 45, 60, 0.9f)) : 0; // 超级电容控制频率为500ms,根据裁判系统等级调整功率限制

        Timeout_flag == 1 ? RefereeLostCallback() : 0; // 遥控器信号丢失则调用裁判系统丢失回调函数

        Gimbal_Data.CH_Gyro[0] = (int8_t) INS.Gyro[0];
        Gimbal_Data.CH_Gyro[1] = (int8_t) INS.Gyro[1];
        Gimbal_Data.CH_Gyro[2] = (int8_t) INS.Gyro[2];
        Gimbal_Data.CH_Pitch = (int8_t) INS.Pitch;
        Gimbal_Data.CH_Roll = (int8_t) INS.Roll;
        Gimbal_Data.Shoot_Upload_Data.heat = Referee_data->PowerHeatData.shooter_17mm_1_barrel_heat;
        Gimbal_Data.Shoot_Upload_Data.reference_online_state = Referee_data->referee_online_state;
        Gimbal_Data.Shoot_Upload_Data.robot_level = Referee_data->GameRobotState.robot_level;
        Gimbal_Data.Shoot_Upload_Data.shooter_barrel_cooling_value = Referee_data->GameRobotState.
                shooter_barrel_cooling_value;
        Gimbal_Data.Shoot_Upload_Data.shooter_heat_limit = Referee_data->GameRobotState.shooter_barrel_heat_limit;

        CANCommSend(CANCOM, (uint8_t *) &Gimbal_Data);
#elifdef MCU_GIMBAL
        CANCommSend(CANCOM, (uint8_t *) &Chassis_Data);

        VisionSend();

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
    huart6.Init.BaudRate = 921600;
    HAL_UART_Init(&huart6);

    RC_data = RemoteControlInit(&huart3); // 遥控器和图传二选一
    vision_recv_data = VisionInit();
    // VL_data = VLRemoteControlInit(&huart6);

    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12, GPIO_PIN_RESET); //USB DP拉低，重新枚举

#elifdef MCU_CHASSIS
    huart6.Init.BaudRate = 115200;
    HAL_UART_Init(&huart6);
    Referee_data = UIInit(&huart6, &ui_data);

    SuperCap_Init_Config_s cap_conf = {
        .enableDCDC = 1, //允许开启DCDC电路
        .systemRestart = 0, //不需要重启系统
        .clearError = 1, //需要清除错误
        .enableActiveChargingLimit = 0, //禁用主动充电限制---类似于手机电池保护每次充到95%就停止充电，不让电池充满从而延长超级电容使用寿命
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

    // 上台阶履带PID参数
    PID_Param(&UPPID, 0.0f, -1.0f, 0.0f,
              Integral_Limit | Derivative_On_Measurement,
              1.0f, 6, 0.2f, 0.2f);

    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    shoot_cmd_pub = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    shoot_feed_sub = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
}

/**
 * @brief 根据gimbal app传回的当前电机角度计算和零位的误差
 *        单圈绝对角度的范围是0~360,说明文档中有图示
 */
static void CalcOffsetAngle(void)
{
    static float angle;
    angle = gimbal_fetch_data.yaw_motor_single_round_angle;

    // 计算 offset_angle (0~360度)
    chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    if (chassis_cmd_send.offset_angle < 0)
        chassis_cmd_send.offset_angle += 360.0f;

    // 转换为 -180~180 度，方便PID控制
    if (chassis_cmd_send.offset_angle > 180.0f)
        chassis_cmd_send.angle_offset_c = chassis_cmd_send.offset_angle - 360.0f;
    else
        chassis_cmd_send.angle_offset_c = chassis_cmd_send.offset_angle;

    gimbal_cmd_send.angle_offset_g = chassis_cmd_send.angle_offset_c;
}

/**
 * @brief 命令解析
 * @todo 紧急停止，键盘控制，遥控器控制三种模式的切换还没完成
 */
static void Robot_Cmd_Serve(void) {
    // 左右两杆均拨下，紧急断电
    chassis_cmd_send.chassis_last_mode = chassis_cmd_send.chassis_mode;
    if ((switch_is_down(RC_data[TEMP].rc.switch_left) && switch_is_down(RC_data[TEMP].rc.switch_right)) ||
        Timeout_flag || VL_data[TEMP].rc.mode_sw == 2) {
        Emergency_Stop();
        return;
    }
    // 射击指令
    shoot_cmd_send.shoot_mode = SHOOT_ON;
    LED_Red_Down;

    // 左右拨杆均拨上，键盘控制,否则遥控器控制
    if ((switch_is_up(RC_data[TEMP].rc.switch_left) && switch_is_up(RC_data[TEMP].rc.switch_right)))
        Keyboard_Cmd();
    else
        RemoteControl_Cmd();
}

/**
 * @brief 紧急断电状态
 */
static void Emergency_Stop(void) {
    chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
    chassis_cmd_send.track = TRACK_NONE;
    gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
    shoot_cmd_send.shoot_mode = SHOOT_OFF;
    shoot_cmd_send.friction_mode = FRICTION_OFF;
    shoot_cmd_send.load_mode = LOAD_STOP;
    LED_Red_Up;
}

/**
 * @brief 键盘控制指令解析
 * @note 这是基于图传的键鼠控制
 */
static void Keyboard_Cmd(void) {
    static float max_speed = 2500.0f; //默认最大速度2.5m/s

    if (VL_data[TEMP].key[KEY_PRESS].ctrl) //Ctrl键按住升高模式
    {
        chassis_cmd_send.chassis_mode = CHASSIS_INDEPENDENCE;
        chassis_cmd_send.track = TRACK_EXTEND;
    } else if (VL_data[TEMP].key_count[KEY_PRESS][Key_C] % 2 == 1) // C键切换履带上台阶
    {
        chassis_cmd_send.chassis_mode = CHASSIS_INDEPENDENCE;
        chassis_cmd_send.track = TRACK_UP;
    } else // 默认底盘随云台转动
    {
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
        chassis_cmd_send.track = TRACK_NONE;
    }

    //WASD方向平移
    if (VL_data[TEMP].key[KEY_PRESS].w)
        chassis_cmd_send.vy = (chassis_cmd_send.vy < max_speed) ? (chassis_cmd_send.vy += 10.0f) : max_speed;
    else if (VL_data[TEMP].key[KEY_PRESS].s)
        chassis_cmd_send.vy = (chassis_cmd_send.vy > -max_speed) ? (chassis_cmd_send.vy -= 10.0f) : -max_speed;
    else chassis_cmd_send.vy = 0;

    if (chassis_cmd_send.chassis_mode == CHASSIS_INDEPENDENCE) {
        if (VL_data[TEMP].key[KEY_PRESS].d)
            chassis_cmd_send.wz = (chassis_cmd_send.wz < 0.5f) ? (chassis_cmd_send.wz += 0.01f) : 0.5f;
        else if (VL_data[TEMP].key[KEY_PRESS].a)
            chassis_cmd_send.wz = (chassis_cmd_send.wz > -0.5f) ? (chassis_cmd_send.wz -= 0.01f) : -0.5f;
        else
            chassis_cmd_send.wz = 0;
    } else {
        if (VL_data[TEMP].key[KEY_PRESS].d)
            chassis_cmd_send.vx = (chassis_cmd_send.vx < max_speed) ? (chassis_cmd_send.vx -= 10.0f) : max_speed;
        else if (VL_data[TEMP].key[KEY_PRESS].a)
            chassis_cmd_send.vx = (chassis_cmd_send.vx > -max_speed) ? (chassis_cmd_send.vx += 10.0f) : -max_speed;
        else
            chassis_cmd_send.vx = 0;
    }

    // 按住Shift键底盘小陀螺 todo 后续增加与履带协同的小陀螺
    if (VL_data[TEMP].key[KEY_PRESS].shift)
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;

    if (VL_data[TEMP].mouse.mouse_right) //长按鼠标右键进入自瞄模式
    {
        gimbal_cmd_send.gimbal_mode = GIMBAL_VISION; //只做头部跟随
        gimbal_cmd_send.yaw = vision_recv_data->gimbal_data.yaw;
        gimbal_cmd_send.pitch = vision_recv_data->gimbal_data.pitch;
        gimbal_cmd_send.pitch = Angle_limit(gimbal_cmd_send.pitch, 13.0f, -35.0f);

        shoot_cmd_send.friction_mode = vision_recv_data->fire_control_data.friction_flag;
        shoot_cmd_send.load_mode = vision_recv_data->fire_control_data.fire_mode_flag;
        shoot_cmd_send.load_mode = vision_recv_data->fire_control_data.fire_flag == 0
                                       ? LOAD_STOP
                                       : shoot_cmd_send.load_mode; //如果发射标志位为0则停止发射，否则保持原有模式
        shoot_cmd_send.shoot_rate = vision_recv_data->fire_control_data.loader_frequency;
    }

    //不按右键云台自由移动
    else {
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
        gimbal_cmd_send.yaw -= (float) VL_data[TEMP].mouse.x / 660 * 2.4f; //3.5
        gimbal_cmd_send.pitch += (float) VL_data[TEMP].mouse.y / 660 * 1.3f; //-2.5
        gimbal_cmd_send.pitch = Angle_limit(gimbal_cmd_send.pitch, 13.0f, -35.0f);
    }

    switch (VL_data[TEMP].key_count[KEY_PRESS][Key_F] % 2) // F键开关摩擦轮，不受自瞄模式影响
    {
        case 0:
            shoot_cmd_send.friction_mode = FRICTION_OFF;
            break;
        default:
            shoot_cmd_send.friction_mode = FRICTION_ON;
            break;
    }
    switch (VL_data[TEMP].key_count[KEY_PRESS][Key_V] % 4) //V键调整速度
    {
        case 0:
            max_speed = 1500.0f;
            break;
        case 1:
            max_speed = 2000.0f;
            break;
        case 2:
            max_speed = 2500.0f;
            break;
        default:
            max_speed = 3000.0f;
    }
    if (VL_data[TEMP].mouse.mouse_left) //鼠标左键短按单发，长按连发
    {
        const float current_time = DWT_GetTimeline_ms();
        static float mouse_press_time = 0.0f;
        if (shoot_cmd_send.load_mode == LOAD_STOP) {
            mouse_press_time = current_time; //获取时间
            shoot_cmd_send.load_mode = LOAD_1_BULLET;
        } // 单发模式后连续按下一段时间进入连发模式
        else if (shoot_cmd_send.load_mode == LOAD_1_BULLET) {
            if (current_time - mouse_press_time >= 500.0f) {
                shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
                shoot_cmd_send.shoot_rate = 10.0f; // 单位Hz
            }
            // 短按则继续单发
            else
                shoot_cmd_send.load_mode = LOAD_STOP;
        }
    }

    // 履带控制,和遥控器稍微不同
    static uint8_t flag = 0; //高度差计数，flag履带状态标志
    // 第二阶段重置角度
    if (flag == 1) {
        chassis_cmd_send.a_track_head = 40.0f;
        chassis_cmd_send.a_track_back = 105.0f;
        flag = 2;
    }

    if (flag == 2 && Gimbal_Rec->CH_Pitch > -1)
        flag = 3;

    if (flag == 4) // 上台阶完成
        chassis_cmd_send.track = TRACK_NONE;

    switch (chassis_cmd_send.track) {
        case TRACK_UP:
            chassis_cmd_send.chassis_mode = CHASSIS_INDEPENDENCE;

            if (Gimbal_Rec->CH_Pitch < -7 && flag == 0) // 云台俯角大于7度，认为开始上台阶，调整履带角度准备上台阶
                flag = 1; // 云台俯角大于10度，认为上完台阶，收回履带
            if (Gimbal_Rec->CH_Pitch < -7 && flag == 3) // 上台阶过程中如果云台俯角再次大于7度，认为上完台阶，收回履带
                flag = 4;

            chassis_cmd_send.a_track_head += (float) VL_data[TEMP].mouse.z / 660 * 1.5f; // 鼠标滚轮控制上台阶角度微调
            chassis_cmd_send.a_track_head = Angle_limit(chassis_cmd_send.a_track_head, 150, 40.0f);
            chassis_cmd_send.a_track_back += PID_Calculate(&UPPID, 0.0f, Gimbal_Rec->CH_Pitch);
            chassis_cmd_send.a_track_back = Angle_limit(chassis_cmd_send.a_track_back, MAX_ANGLE_TRACK, 105.0f);
            break;
        case TRACK_EXTEND:
            chassis_cmd_send.chassis_mode = CHASSIS_INDEPENDENCE;
            chassis_cmd_send.a_track_head = MAX_ANGLE_TRACK;
            chassis_cmd_send.a_track_back = 155;
            flag = 0;
            break;
        case TRACK_ROTATE:
            chassis_cmd_send.a_track_head = 0.0f;
            chassis_cmd_send.a_track_back = 0.0f;
            flag = 0;
            // @todo 小陀螺先不写
            break;
        case TRACK_NONE:
            chassis_cmd_send.a_track_head = 0.0f;
            chassis_cmd_send.a_track_back = 0.0f;
            chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
            flag = 0;
            break;
    }
}

/**
 * @brief 遥控器控制指令解析
 */
static void RemoteControl_Cmd(void) {
    abs(RC_data[TEMP].rc.dial) > 100 // 拨盘大于100 启动电机
        ? (shoot_cmd_send.friction_mode = FRICTION_ON)
        : (shoot_cmd_send.friction_mode = FRICTION_OFF);
    if (RC_data[TEMP].rc.dial > 500) {
        shoot_cmd_send.load_mode = LOAD_BURSTFIRE; //连发
        shoot_cmd_send.shoot_rate = 15.0f; //单位Hz
    } else if (RC_data[TEMP].rc.dial < -500)
        shoot_cmd_send.load_mode = LOAD_1_BULLET; //单发
    else
        shoot_cmd_send.load_mode = LOAD_STOP; //停止

    static uint8_t flag = 0; //flag履带状态标志
    //左杆在下，右杆在中，底盘小陀螺
    if (switch_is_down(RC_data[TEMP].rc.switch_left) && switch_is_mid(RC_data[TEMP].rc.switch_right)) {
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
        chassis_cmd_send.track = TRACK_ROTATE;
        flag = 0;
    }
    //左杆在下，右杆在上，底盘随云台控制
    else if (switch_is_down(RC_data[TEMP].rc.switch_left) && switch_is_up(RC_data[TEMP].rc.switch_right)) {
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
        chassis_cmd_send.track = TRACK_NONE;
        flag = 0;
    }
    // 左杆在中，右杆在中，履带为下台阶模式
    else if (switch_is_mid(RC_data[TEMP].rc.switch_left) && switch_is_mid(RC_data[TEMP].rc.switch_right)) {
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
        chassis_cmd_send.track = TRACK_DOWN;
        flag = 0;
    }
    // 左杆在中，右杆在下，履带为降下模式
    else if (switch_is_mid(RC_data[TEMP].rc.switch_left) && switch_is_down(RC_data[TEMP].rc.switch_right)) {
        chassis_cmd_send.chassis_mode = CHASSIS_INDEPENDENCE;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
        chassis_cmd_send.track = TRACK_EXTEND;
        flag = 0;
    }
    // 左杆在中，右杆在上，履带为上台阶模式
    else if (switch_is_mid(RC_data[TEMP].rc.switch_left) && switch_is_up(RC_data[TEMP].rc.switch_right)) {
        chassis_cmd_send.chassis_mode = CHASSIS_INDEPENDENCE;
        gimbal_cmd_send.gimbal_mode = GIMBAL_FREE_MODE;
        chassis_cmd_send.track = TRACK_UP;
    }
    // 左杆在上，右杆在中，开启自瞄模式
    else if (switch_is_up(RC_data[TEMP].rc.switch_left) && switch_is_mid(RC_data[TEMP].rc.switch_right)) {
        chassis_cmd_send.chassis_mode = CHASSIS_INDEPENDENCE;
        gimbal_cmd_send.gimbal_mode = GIMBAL_VISION;
        chassis_cmd_send.track = TRACK_NONE;
        flag = 0;
    }

    if (flag == 1) // 上台阶第二阶段，重置角度
    {
        chassis_cmd_send.a_track_head = 40.0f;
        chassis_cmd_send.a_track_back = 105.0f;
        flag = 2;
    }
    if (flag == 2 && Gimbal_Rec->CH_Pitch > -1) // 上台阶完成，重置角度
        flag = 3;
    if (flag == 4) // 上台阶完成，重置角度
        chassis_cmd_send.track = TRACK_NONE;

    switch (chassis_cmd_send.track) {
        case TRACK_UP:
            if (Gimbal_Rec->CH_Pitch < -8 && flag == 0)
                flag = 1; //俯仰角过小，认为上台阶第一阶段完成
            if (Gimbal_Rec->CH_Pitch < -8 && flag == 3)
                flag = 4; //俯仰角过小，认为上台阶第二阶段完成

            chassis_cmd_send.a_track_head += (float) RC_data[TEMP].rc.rocker_r1 * 0.0006f;
            chassis_cmd_send.a_track_head = Angle_limit(chassis_cmd_send.a_track_head,
                                                        MAX_ANGLE_TRACK, 40.0f);
            chassis_cmd_send.a_track_back += PID_Calculate(&UPPID, 0.0f, Gimbal_Rec->CH_Pitch);
            chassis_cmd_send.a_track_back = Angle_limit(chassis_cmd_send.a_track_back, MAX_ANGLE_TRACK,
                                                        105.0f);
            break;
        case TRACK_EXTEND:
            chassis_cmd_send.a_track_head = MAX_ANGLE_TRACK;
            chassis_cmd_send.a_track_back = 155;
            break;
        case TRACK_ROTATE:
            chassis_cmd_send.a_track_head = MAX_ANGLE_TRACK * fabsf(sinf(DWT_GetTimeline_s()));
            chassis_cmd_send.a_track_head = Angle_limit(chassis_cmd_send.a_track_head, MAX_ANGLE_TRACK, 90.0f);
            chassis_cmd_send.a_track_back = MAX_ANGLE_TRACK * fabsf(sinf(DWT_GetTimeline_s()));
            chassis_cmd_send.a_track_back = Angle_limit(chassis_cmd_send.a_track_back, MAX_ANGLE_TRACK, 105.0f);
            break;
        case TRACK_NONE:
            chassis_cmd_send.a_track_head = 0;
            chassis_cmd_send.a_track_back = 0;
            break;
        default:
            break;
    }
    // 跟随云台转动，右拨杆控制云台俯仰和偏航
    if (gimbal_cmd_send.gimbal_mode == GIMBAL_GYRO_MODE) {
        gimbal_cmd_send.yaw -= 0.0008f * (float) RC_data[TEMP].rc.rocker_r_;
        gimbal_cmd_send.pitch -= 0.0006f * (float) RC_data[TEMP].rc.rocker_r1;
        gimbal_cmd_send.pitch = Angle_limit(gimbal_cmd_send.pitch, 13.0f, -35.0f);
    }
    // 视觉模式
    else if (gimbal_cmd_send.gimbal_mode == GIMBAL_VISION) {
        gimbal_cmd_send.yaw = vision_recv_data->gimbal_data.yaw;
        gimbal_cmd_send.pitch = vision_recv_data->gimbal_data.pitch;
        gimbal_cmd_send.pitch = Angle_limit(gimbal_cmd_send.pitch, 13.0f, -35.0f);
    }
    // 平移速度设置
    chassis_cmd_send.vy = (float) RC_data[TEMP].rc.rocker_l1 * 0.151f * 35.0f; // 最高3.5m/s
    chassis_cmd_send.chassis_mode == CHASSIS_INDEPENDENCE
        ? (chassis_cmd_send.wz = (float) RC_data[TEMP].rc.rocker_l_ * 0.00151f * 1.0f) // 最高1转/s
        : (chassis_cmd_send.vx = -(float) RC_data[TEMP].rc.rocker_l_ * 0.151f * 35.0f); // 最高3.5m/s
}
