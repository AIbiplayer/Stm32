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
#include "ui_g.h"
#include "super_cap.h"
#include "Video_link.h"
#include "TMC.h"
#include "UI.h"
#include "ui_interface.h"
#include "ui_queue.h"

CCMRAM RC_ctrl_t *RC_data = NULL; // 遥控器数据,初始化时返回
CCMRAM VL_ctrl_t *VL_data = NULL; // 图传遥控器数据,初始化时返回
CCMRAM Referee_Interactive_info_t *ui_data; // UI绘制需要的机器人状态,初始化时返回
CCMRAM CANCommInstance *CANCOM; // 底盘或云台的CAN通信实例指针
CCMRAM float Timeout = 0.0f; // 遥控器信号丢失计时
CCMRAM uint8_t Timeout_flag = 0; // 遥控器信号丢失标志
CCMRAM referee_info_t *Referee_data; // 裁判系统数据
CCMRAM PID_Typedef UPPID; // 上台阶履带PID
CCMRAM Vision_Recv_s *vision_recv_data; // 视觉接收数据

extern INS_t INS; // IMU数据,包含底盘的姿态和角速度等信息
extern TMC_To_Chassis_s *Chassis_Rec; // 底盘与云台数据结构体实例
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
static uint16_t SuperCap_count; // 超级电容控制计数器
static uint8_t DM_count; // 达妙电机控制计数器
static uint8_t UI_count; // UI更新计数器

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

static void VL_keyboard_cmd(void);

static void Keyboard_Cmd(void);

static void Chassis_data_tidy(void);

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
        SubGetMessage(shoot_feed_sub, &shoot_fetch_data);

        CalcOffsetAngle();
        Robot_Cmd_Serve();
        Chassis_Data.Chassis_Cmd = chassis_cmd_send;
        Chassis_Data.friction_mode = shoot_cmd_send.friction_mode;
        Chassis_Data.gimbal_mode = gimbal_cmd_send.gimbal_mode;

#endif
        DJI_Motor_Control();
        Timeout_flag = DWT_GetTimeline_ms() - Timeout > 100.0f ? 1 : 0; // 遥控器信号丢失标志,丢失时间超过100ms则置位

#ifdef MCU_CHASSIS

        ui_self_id = 3; // todo 这个ID应该从裁判系统数据里获取,现在先写死为3
        Chassis_data_tidy();

        // UI发送服务：每3ms调用一次，内部控制约10ms发送一个UI帧（100Hz）
        UI_Queue_SendService();

        UI_count++;
        if (Referee_data->init_flag == 1 && UI_count >= 100) // 裁判系统初始化完成且UI更新计数器达到100,约300ms触发一次UI更新
        {
            // UI更新数据加入队列（每次更新4个帧，将在40ms内按100Hz发送完成）
            ui_update_g_Upgroup();
            UI_count = 0;
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

#elifdef MCU_GIMBAL
        CANCommSend(CANCOM, (uint8_t *) &Chassis_Data);

        PubPushMessage(shoot_cmd_pub, (void *) &shoot_cmd_send);
        PubPushMessage(gimbal_cmd_pub, (void *) &gimbal_cmd_send);
#endif

        osDelay(3);
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
    // VL_data = VLRemoteControlInit(&huart6);
    vision_recv_data = VisionInit();
    /******************************************************/

    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12, GPIO_PIN_RESET); //USB DP拉低，重新枚举

#elifdef MCU_CHASSIS
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
static void CalcOffsetAngle(void) {
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
    if ((switch_is_down(RC_data[TEMP].rc.switch_left) && switch_is_down(RC_data[TEMP].rc.switch_right) && RC_data !=
         NULL) || (VL_data != NULL && VL_data[TEMP].rc.mode_sw != 0) || Timeout_flag) {
        Emergency_Stop();
        return;
    }
    // 射击指令
    shoot_cmd_send.shoot_mode = SHOOT_ON;
    LED_Red_Down;

    // 键盘控制优先级最高，图传遥控器次之，遥控器优先级最低
    if ((switch_is_up(RC_data[TEMP].rc.switch_left) && switch_is_up(RC_data[TEMP].rc.switch_right)))
        Keyboard_Cmd();
    else if (VL_data != NULL && VL_data[TEMP].rc.mode_sw == 0)
        VL_keyboard_cmd();
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
static void VL_keyboard_cmd(void) {
    static uint16_t max_speed = 1000; //默认最大速度1m/s
    if (VL_data[TEMP].key[KEY_PRESS].ctrl) //Ctrl键按住升高模式
    {
        chassis_cmd_send.chassis_mode = CHASSIS_INDEPENDENCE;
        chassis_cmd_send.track = TRACK_EXTEND;
    } else // 默认底盘随云台转动
    {
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
        chassis_cmd_send.track = TRACK_NONE;
    }
    if (VL_data[TEMP].key_count[KEY_PRESS][Key_C] % 2 == 1) // C键切换履带上台阶
    {
        chassis_cmd_send.chassis_mode = CHASSIS_INDEPENDENCE;
        chassis_cmd_send.track = TRACK_UP;
    }
    if (VL_data[TEMP].key_count[KEY_PRESS][Key_Z] % 2 == 1) {
        // Z键重置UI
        ui_self_id = Referee_data->GameRobotState.robot_id;
        ui_init_g_Midgroup();
        ui_init_g_Upgroup();
        ui_init_g_Ungroup();
    }

    //WASD方向平移
    if (VL_data[TEMP].key[KEY_PRESS].w)
        chassis_cmd_send.vy = (chassis_cmd_send.vy < (float) max_speed)
                                  ? (chassis_cmd_send.vy += 10.0f)
                                  : (float) max_speed;
    else if (VL_data[TEMP].key[KEY_PRESS].s)
        chassis_cmd_send.vy = (chassis_cmd_send.vy > -(float) max_speed)
                                  ? (chassis_cmd_send.vy -= 10.0f)
                                  : -(float) max_speed;
    else chassis_cmd_send.vy = 0;

    if (chassis_cmd_send.chassis_mode == CHASSIS_INDEPENDENCE) {
        if (VL_data[TEMP].key[KEY_PRESS].d)
            chassis_cmd_send.wz = (chassis_cmd_send.wz < 0.5f) ? (chassis_cmd_send.wz += 0.01f) : 0.5f;
        else if (VL_data[TEMP].key[KEY_PRESS].a)
            chassis_cmd_send.wz = (chassis_cmd_send.wz > -0.5f) ? (chassis_cmd_send.wz -= 0.01f) : -0.5f;
        else
            chassis_cmd_send.wz = 0;
    } else {
        // 按D键vx减小(向右), 按A键vx增大(向左)
        if (VL_data[TEMP].key[KEY_PRESS].d)
            chassis_cmd_send.vx = (chassis_cmd_send.vx > -(float) max_speed)
                                      ? (chassis_cmd_send.vx -= 10.0f)
                                      : -(float) max_speed;
        else if (VL_data[TEMP].key[KEY_PRESS].a)
            chassis_cmd_send.vx = (chassis_cmd_send.vx < (float) max_speed)
                                      ? (chassis_cmd_send.vx += 10.0f)
                                      : (float) max_speed;
        else
            chassis_cmd_send.vx = 0;
    }

    // 按住Shift键底盘小陀螺 todo 后续增加与履带协同的小陀螺
    if (VL_data[TEMP].key[KEY_PRESS].shift)
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;

    if (VL_data[TEMP].mouse.mouse_left) //鼠标左键短按单发，长按连发
    {
        switch (VL_data[TEMP].key_count[KEY_PRESS][Key_R] % 2) // R键切换单发连发模式
        {
            case 0:
                shoot_cmd_send.load_mode = LOAD_1_BULLET;
                break;
            case 1:
                shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
                shoot_cmd_send.shoot_rate = 15.0f; // 单位Hz
                break;
            default:
                break;
        }
    } else shoot_cmd_send.load_mode = LOAD_STOP; //停止发射

    if (VL_data[TEMP].mouse.mouse_right) //长按鼠标右键进入自瞄模式
    {
        gimbal_cmd_send.gimbal_mode = GIMBAL_VISION; //只做头部跟随
        chassis_cmd_send.chassis_mode = CHASSIS_INDEPENDENCE;
        gimbal_cmd_send.yaw = vision_recv_data->gimbal_data.yaw;
        gimbal_cmd_send.pitch = vision_recv_data->gimbal_data.pitch;
        gimbal_cmd_send.pitch = Angle_limit(gimbal_cmd_send.pitch, 13.0f, -35.0f);
        if (VL_data[TEMP].mouse.mouse_left && vision_recv_data->fire_control_data.fire_flag)
        //鼠标左键短按单发，长按连发，前提是视觉模块的fire_flag为真
        {
            switch (VL_data[TEMP].key_count[KEY_PRESS][Key_R] % 2) // R键切换单发连发模式
            {
                case 0:
                    shoot_cmd_send.load_mode = LOAD_1_BULLET;
                    break;
                case 1:
                    shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
                    shoot_cmd_send.shoot_rate = 15.0f; // 单位Hz
                    break;
                default:
                    break;
            }
        } else shoot_cmd_send.load_mode = LOAD_STOP; //停止发射
    }

    //不按右键云台自由移动
    else {
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
        gimbal_cmd_send.yaw -= (float) VL_data[TEMP].mouse.x / 660 * 2.4f; //3.5
        gimbal_cmd_send.pitch -= (float) VL_data[TEMP].mouse.y / 660 * 1.3f; //-2.5
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
            max_speed = 1000;
            break;
        case 1:
            max_speed = 1500;
            break;
        case 2:
            max_speed = 2000;
            break;
        default:
            max_speed = 2500;
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

            chassis_cmd_send.a_track_head += (float) VL_data[TEMP].mouse.z / 660 * 3.5f; // 鼠标滚轮控制上台阶角度微调
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
            flag = 0;
            break;
    }
    Chassis_Data.speed_target = (uint8_t) (max_speed / 100);
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
            chassis_cmd_send.a_track_head = 0;
            chassis_cmd_send.a_track_back = 0;
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
        chassis_cmd_send.chassis_mode = CHASSIS_INDEPENDENCE;
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

/**
 * @brief DR16键盘控制指令解析
 */
static void Keyboard_Cmd(void) {
    static uint16_t max_speed = 1000; //默认最大速度1m/s
    if (RC_data[TEMP].key[KEY_PRESS].ctrl) {
        chassis_cmd_send.chassis_mode = CHASSIS_INDEPENDENCE;
        chassis_cmd_send.track = TRACK_EXTEND;
    } else // 默认底盘随云台转动
    {
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
        chassis_cmd_send.track = TRACK_NONE;
    }

    if (RC_data[TEMP].key_count[KEY_PRESS][Key_C] % 2 == 1) // C键切换履带上台阶
    {
        chassis_cmd_send.chassis_mode = CHASSIS_INDEPENDENCE;
        chassis_cmd_send.track = TRACK_UP;
    }

    if (RC_data[TEMP].key_count[KEY_PRESS][Key_Z] % 2 == 1) {
        // Z键重置UI
        ui_self_id = Referee_data->GameRobotState.robot_id;
        ui_init_g_Midgroup();
        ui_init_g_Upgroup();
        ui_init_g_Ungroup();
    }

    //WASD方向平移
    if (RC_data[TEMP].key[KEY_PRESS].w)
        chassis_cmd_send.vy = (chassis_cmd_send.vy < (float) max_speed)
                                  ? (chassis_cmd_send.vy += 10.0f)
                                  : (float) max_speed;
    else if (RC_data[TEMP].key[KEY_PRESS].s)
        chassis_cmd_send.vy = (chassis_cmd_send.vy > -(float) max_speed)
                                  ? (chassis_cmd_send.vy -= 10.0f)
                                  : -(float) max_speed;
    else chassis_cmd_send.vy = 0;

    if (chassis_cmd_send.chassis_mode == CHASSIS_INDEPENDENCE) {
        if (RC_data[TEMP].key[KEY_PRESS].d)
            chassis_cmd_send.wz = (chassis_cmd_send.wz < 0.5f) ? (chassis_cmd_send.wz += 0.01f) : 0.5f;
        else if (RC_data[TEMP].key[KEY_PRESS].a)
            chassis_cmd_send.wz = (chassis_cmd_send.wz > -0.5f) ? (chassis_cmd_send.wz -= 0.01f) : -0.5f;
        else
            chassis_cmd_send.wz = 0;
    } else {
        // 按D键vx减小(向右), 按A键vx增大(向左)
        if (RC_data[TEMP].key[KEY_PRESS].d)
            chassis_cmd_send.vx = (chassis_cmd_send.vx > -(float) max_speed)
                                      ? (chassis_cmd_send.vx -= 10.0f)
                                      : -(float) max_speed;
        else if (RC_data[TEMP].key[KEY_PRESS].a)
            chassis_cmd_send.vx = (chassis_cmd_send.vx < (float) max_speed)
                                      ? (chassis_cmd_send.vx += 10.0f)
                                      : (float) max_speed;
        else
            chassis_cmd_send.vx = 0;
    }

    // 按住Shift键底盘小陀螺 todo 后续增加与履带协同的小陀螺
    if (RC_data[TEMP].key[KEY_PRESS].shift)
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;


    if (RC_data[TEMP].mouse.press_l) //鼠标左键短按单发，长按连发
    {
        switch (RC_data[TEMP].key_count[KEY_PRESS][Key_R] % 2) // R键切换单发连发模式
        {
            case 0:
                shoot_cmd_send.load_mode = LOAD_1_BULLET;
                break;
            case 1:
                shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
                shoot_cmd_send.shoot_rate = 15.0f; // 单位Hz
                break;
            default:
                break;
        }
    } else shoot_cmd_send.load_mode = LOAD_STOP; //停止发射

    if (RC_data[TEMP].mouse.press_r) //长按鼠标右键进入自瞄模式
    {
        gimbal_cmd_send.gimbal_mode = GIMBAL_VISION; //只做头部跟随
        chassis_cmd_send.chassis_mode = CHASSIS_INDEPENDENCE;
        gimbal_cmd_send.yaw = vision_recv_data->gimbal_data.yaw;
        gimbal_cmd_send.pitch = vision_recv_data->gimbal_data.pitch;
        gimbal_cmd_send.pitch = Angle_limit(gimbal_cmd_send.pitch, 13.0f, -35.0f);
        if (RC_data[TEMP].mouse.press_l && vision_recv_data->fire_control_data.fire_flag)
        //鼠标左键短按单发，长按连发，前提是视觉模块的fire_flag为真
        {
            switch (RC_data[TEMP].key_count[KEY_PRESS][Key_R] % 2) // R键切换单发连发模式
            {
                case 0:
                    shoot_cmd_send.load_mode = LOAD_1_BULLET;
                    break;
                case 1:
                    shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
                    shoot_cmd_send.shoot_rate = 15.0f; // 单位Hz
                    break;
                default:
                    break;
            }
        } else shoot_cmd_send.load_mode = LOAD_STOP; //停止发射
    }

    //不按右键云台自由移动
    else {
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
        gimbal_cmd_send.yaw -= (float) RC_data[TEMP].mouse.x / 660 * 2.4f; //3.5
        gimbal_cmd_send.pitch -= (float) RC_data[TEMP].mouse.y / 660 * 1.3f; //-2.5
        gimbal_cmd_send.pitch = Angle_limit(gimbal_cmd_send.pitch, 13.0f, -35.0f);
    }

    switch (RC_data[TEMP].key_count[KEY_PRESS][Key_F] % 2) // F键开关摩擦轮，不受自瞄模式影响
    {
        case 0:
            shoot_cmd_send.friction_mode = FRICTION_OFF;
            break;
        default:
            shoot_cmd_send.friction_mode = FRICTION_ON;
            break;
    }
    switch (RC_data[TEMP].key_count[KEY_PRESS][Key_V] % 4) //V键调整速度
    {
        case 0:
            max_speed = 1000;
            break;
        case 1:
            max_speed = 1500;
            break;
        case 2:
            max_speed = 2000;
            break;
        default:
            max_speed = 2500;
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

            chassis_cmd_send.a_track_head += (float) RC_data[TEMP].key[KEY_PRESS].q ? 0.2f : 0; // 鼠标滚轮控制上台阶角度微调
            chassis_cmd_send.a_track_head -= (float) RC_data[TEMP].key[KEY_PRESS].e ? 0.2f : 0; // 鼠标滚轮控制上台阶角度微调

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
            flag = 0;
            break;
    }
    Chassis_Data.speed_target = (uint8_t) (max_speed / 100);
}

/**
 * @brief 裁判系统数据整理
 * @note 目前主要是把裁判系统的数据转发到CAN总线上，后续可以根据需要增加一些处理逻辑
 */
static void Chassis_data_tidy(void) {
    // 将裁判系统的数据转发到CAN总线上，后续可以根据需要增加一些处理逻辑
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

    // UI显示
    switch (Chassis_Rec->friction_mode) {
        case FRICTION_OFF:
            memcpy(ui_g_Upgroup_friction_status->string, "OFF", 4);
            ui_g_Upgroup_friction_status->str_length = 4;
            // 包含 '\0'
            break;
        case FRICTION_ON:
            memcpy(ui_g_Upgroup_friction_status->string, "ON ", 4);
            ui_g_Upgroup_friction_status->str_length = 4;
            break;
        default:
            break;
    }
    switch (Chassis_Rec->gimbal_mode) {
        case GIMBAL_ZERO_FORCE:
            memcpy(ui_g_Upgroup_gimbal_status->string, "ZERO", 5);
            ui_g_Upgroup_gimbal_status->str_length = 5;
            break;
        case GIMBAL_GYRO_MODE:
            memcpy(ui_g_Upgroup_gimbal_status->string, "GYRO", 5);
            ui_g_Upgroup_gimbal_status->str_length = 5;
            break;
        case GIMBAL_VISION:
            memcpy(ui_g_Upgroup_gimbal_status->string, "VIS ", 5);
            ui_g_Upgroup_gimbal_status->str_length = 5;
            break;
        case GIMBAL_FREE_MODE:
            memcpy(ui_g_Upgroup_gimbal_status->string, "FREE", 5);
            ui_g_Upgroup_gimbal_status->str_length = 5;
            break;
        default:
            break;
    }
    switch (Chassis_Rec->Chassis_Cmd.chassis_mode) {
        case CHASSIS_ZERO_FORCE:
            memcpy(ui_g_Upgroup_chassis_status->string, "ZERO", 5);
            ui_g_Upgroup_chassis_status->str_length = 5;
            break;
        case CHASSIS_FOLLOW_GIMBAL_YAW:
            memcpy(ui_g_Upgroup_chassis_status->string, "FOLL", 5);
            ui_g_Upgroup_chassis_status->str_length = 5;
            break;
        case CHASSIS_INDEPENDENCE:
            memcpy(ui_g_Upgroup_chassis_status->string, "INDE", 5);
            ui_g_Upgroup_chassis_status->str_length = 5;
            break;
        case CHASSIS_ROTATE:
            memcpy(ui_g_Upgroup_chassis_status->string, "ROTA", 5);
            ui_g_Upgroup_chassis_status->str_length = 5;
            break;
        default:
            break;
    }
    if (Referee_data->referee_online_state && Referee_data->GameRobotState.robot_id != 0) {
        ui_self_id = Referee_data->GameRobotState.robot_id;
        ui_g_Upgroup_trackhead_angle->end_angle = (uint32_t) Chassis_Rec->Chassis_Cmd.a_track_head;
        ui_g_Upgroup_trackback_angle->start_angle = (uint32_t) (360.0f - Chassis_Rec->Chassis_Cmd.a_track_back);
        ui_g_Upgroup_targetspeed_num->number = Chassis_Rec->speed_target * 100;
        ui_g_Upgroup_power_num->end_x = (int32_t) (cap->cap_msg.chassisPower / (float) Referee_data->GameRobotState.
                                                   chassis_power_limit * 400.0f) + 773;
        ui_g_Upgroup_capenergy_num->end_x = (int32_t) (cap->cap_msg.capEnergy * 4.0f) + 773;
    }

    if (Referee_data->referee_online_state && Referee_data->init_flag == 0) {
        Referee_data->init_flag = 1;
        // UI初始化：3组共12个帧加入队列，将在约120ms内按100Hz发送完成
        // 每个帧间隔约10ms发送，避免扎堆发送
        ui_init_g_Midgroup();
        ui_init_g_Upgroup();
        ui_init_g_Ungroup();
    }
}
