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
#include "bsp_usart.h"
#include "DM_Motor.h"
#include "cmsis_os.h"
#include "can_comm.h"
#include "remote_control.h"
#include "message_center.h"
#include "robot_def.h"
#include "TMC.h"

RC_ctrl_t* RC_data; // 遥控器数据,初始化时返回
CANCommInstance* CANCOM; // 底盘或云台的CAN通信实例指针

/* cmd应用包含的模块实例指针和交互信息存储*/
static Publisher_t* chassis_cmd_pub; // 底盘控制消息发布者
static Subscriber_t* chassis_feed_sub; // 底盘反馈信息订阅者
static Chassis_Ctrl_Cmd_s chassis_cmd_send; // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Chassis_Upload_Data_s chassis_fetch_data; // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等

static Publisher_t* gimbal_cmd_pub; // 云台控制消息发布者
static Subscriber_t* gimbal_feed_sub; // 云台反馈信息订阅者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send; // 传递给云台的控制信息
static Gimbal_Upload_Data_s gimbal_fetch_data; // 从云台获取的反馈信息

static Publisher_t* shoot_cmd_pub; // 发射控制消息发布者
static Subscriber_t* shoot_feed_sub; // 发射反馈信息订阅者
static Shoot_Ctrl_Cmd_s shoot_cmd_send; // 传递给发射的控制信息
static Shoot_Upload_Data_s shoot_fetch_data; // 从发射获取的反馈信息

#ifdef MCU_CHASSIS // 如果是底盘板
static CANComm_Init_Config_s TMC_CANComm_Config = {
    .can_config = {
        .can_handle = &hcan2,
        .tx_id = TMC_CHASSIS_CAN_ID,
        .rx_id = TMC_GIMBAL_CAN_ID,
    },
    .send_data_len = sizeof(TMC_To_Chassis_s),
    .recv_data_len = sizeof(TMC_To_Gimbal_s)
};
#else
static CANComm_Init_Config_s TMC_CANComm_Config = {
    .can_config = {
        .can_handle = &hcan1,
        .tx_id = TMC_GIMBAL_CAN_ID,
        .rx_id = TMC_CHASSIS_CAN_ID,
    },
    .send_data_len = sizeof(uint64_t),
    .recv_data_len = sizeof(uint64_t)
};
#endif

static void Robot_Cmd_Init(void);
static void Emergency_Stop(void);
static void CalcOffsetAngle(void);
static void Remote_Control_Cmd_Serve(void);

static PID_Typedef UPPID; // 上台阶履带PID

extern float HC_Measure; // 超声波测距值
extern USB_Control_t g_usb_dev; // 全局USB设备实例

/**
 * @brief 命令读取与发送FreeRTOS任务
 * @note DJI电机控制函数在此调用
 */
void CmdTask(void* argument)
{
    taskENTER_CRITICAL();
    DWT_Init(168);
    Robot_Cmd_Init();
    MX_USB_DEVICE_Init();
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12, GPIO_PIN_RESET); //USB DP拉低，重新枚举
    taskEXIT_CRITICAL();

    for (;;)
    {
        SubGetMessage(chassis_feed_sub, (void*)&chassis_fetch_data);
        SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);
        SubGetMessage(shoot_feed_sub, (void*)&shoot_fetch_data);

        CalcOffsetAngle();
        Remote_Control_Cmd_Serve();
        DJI_Motor_Control();
        DM_Motor_Control();

        uint64_t TEST = 0x1111111111111111;
        CANCommSend(CANCOM, (uint8_t*)&TEST);

        bsp_usb_transmit((uint8_t*)&TEST, sizeof(TEST));

        if (g_usb_dev.rx_flag)
            LED_Blue_Up;

        PubPushMessage(shoot_cmd_pub, (void*)&shoot_cmd_send);
        PubPushMessage(chassis_cmd_pub, (void*)&chassis_cmd_send);
        PubPushMessage(gimbal_cmd_pub, (void*)&gimbal_cmd_send);

        osDelay(1);
    }
}

/**
 * @brief 机甲命令初始化
 */
static void Robot_Cmd_Init(void)
{
    RC_data = RemoteControlInit(&huart3);
    CANCOM = CANCommInit(&TMC_CANComm_Config);

    // 上台阶履带PID参数
    PID_Param(&UPPID, -6.0f, -3.0f, 0.0f,
              Integral_Limit | Derivative_On_Measurement,
              1.0f, 10, 10, 90);

    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    shoot_cmd_pub = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    shoot_feed_sub = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
}

/**
 * @brief 根据gimbal app传回的当前电机角度计算和零位的误差
 *        单圈绝对角度的范围是0~360,说明文档中有图示
 *
 */
static void CalcOffsetAngle(void) //计算偏移角度角度范围0-360
{
    static float angle;
    angle = gimbal_fetch_data.yaw_motor_single_round_angle; // 从云台获取的当前yaw电机单圈角度
    if (angle > YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else if (angle <= YAW_ALIGN_ANGLE && angle >= YAW_ALIGN_ANGLE - 180.0f)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE + 360.0f;
    else
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE + 360.0f;

    if (chassis_cmd_send.offset_angle > 180.0f && chassis_cmd_send.offset_angle <= 360.0f)
        chassis_cmd_send.angle_offset_c = chassis_cmd_send.offset_angle - 360.0f;
    else
        chassis_cmd_send.angle_offset_c = chassis_cmd_send.offset_angle;
    gimbal_cmd_send.angle_offset_g = chassis_cmd_send.angle_offset_c; //统一角度差
}

/**
 * @brief 遥控器命令解析
 */
static void Remote_Control_Cmd_Serve(void)
{
    // 左右两杆均拨下，紧急断电
    if (switch_is_down(RC_data[TEMP].rc.switch_left) && switch_is_down(RC_data[TEMP].rc.switch_right))
    {
        Emergency_Stop();
        return;
    }
    // 射击指令
    chassis_cmd_send.chassis_last_mode = chassis_cmd_send.chassis_mode;
    shoot_cmd_send.shoot_mode = SHOOT_ON;
    LED_Red_Down;

    abs(RC_data[TEMP].rc.dial) > 100
        ? (shoot_cmd_send.friction_mode = FRICTION_ON)
        : (shoot_cmd_send.friction_mode = FRICTION_OFF);
    if (RC_data[TEMP].rc.dial > 500)
        shoot_cmd_send.load_mode = LOAD_BURSTFIRE; //连发
    else if (RC_data[TEMP].rc.dial < -500)
        shoot_cmd_send.load_mode = LOAD_1_BULLET; //单发
    else
        shoot_cmd_send.load_mode = LOAD_STOP; //停止

    static uint8_t up_count, down_count, flag = 0;
    //两杆为中间，小陀螺模式
    if (switch_is_mid(RC_data[TEMP].rc.switch_left) && switch_is_mid(RC_data[TEMP].rc.switch_right))
    {
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
        chassis_cmd_send.track = TRACK_ROTATE;
        flag = 0;
    }
    // 左杆在中，右杆在上，上台阶模式
    else if (switch_is_mid(RC_data[TEMP].rc.switch_left) && switch_is_up(RC_data[TEMP].rc.switch_right))
    {
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
        chassis_cmd_send.track = TRACK_UP;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    }
    // 左杆在中，右杆在下，底盘自由控制，履带为升高模式
    else if (switch_is_mid(RC_data[TEMP].rc.switch_left) && switch_is_down(RC_data[TEMP].rc.switch_right))
    {
        chassis_cmd_send.chassis_mode = CHASSIS_INDEPENDENCE;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
        chassis_cmd_send.track = TRACK_EXTEND;
        flag = 0;
    }

    if (flag == 2)
        chassis_cmd_send.track = TRACK_NONE;
    // @todo 履带控制指令，由于遥控器中云台和履带共用右拨杆，目前只能使用一个
    switch (chassis_cmd_send.track)
    {
    case TRACK_UP:
        up_count = HC_Measure > 20.0f && flag == 0 ? up_count + 1 : 0;
        down_count = HC_Measure < 10.0f && flag == 1 ? down_count + 1 : 0;
        flag = up_count > 20 ? 1 : flag;
        flag = down_count > 20 ? 2 : flag;

        chassis_cmd_send.a_track_head += (float)RC_data[TEMP].rc.rocker_r1 * 0.00034f;
        chassis_cmd_send.a_track_head = Angle_limit(chassis_cmd_send.a_track_head, 180.0f, 0.0f);
        chassis_cmd_send.a_track_back = 105 + PID_Calculate(&UPPID, 0.0f, gimbal_fetch_data.gimbal_imu_data.Roll);
        chassis_cmd_send.a_track_back = Angle_limit(chassis_cmd_send.a_track_back, 180.0f, 105.0f);
        break;
    case TRACK_EXTEND:
        chassis_cmd_send.a_track_head = 190.0f;
        chassis_cmd_send.a_track_back = 190.0f;
        break;
    case TRACK_ROTATE:
        chassis_cmd_send.a_track_head = 0.0f;
        chassis_cmd_send.a_track_back = 0.0f;
        // @todo 小陀螺先不写
        break;
    case TRACK_NONE:
        chassis_cmd_send.a_track_head = 0.0f;
        chassis_cmd_send.a_track_back = 0.0f;
        break;
    }
    if (gimbal_cmd_send.gimbal_mode == GIMBAL_GYRO_MODE)
    {
        gimbal_cmd_send.yaw -= 0.00034f * (float)RC_data[TEMP].rc.rocker_r_;
        gimbal_cmd_send.pitch += 0.0009f * (float)RC_data[TEMP].rc.rocker_r1;
    }
    chassis_cmd_send.vy = (float)RC_data[TEMP].rc.rocker_l1 / 0.151f * 3.0f; // 最高3m/s
    chassis_cmd_send.chassis_mode == CHASSIS_INDEPENDENCE
        ? (chassis_cmd_send.wz = (float)RC_data[TEMP].rc.rocker_l_ / 0.151f)
        : (chassis_cmd_send.vx = -(float)RC_data[TEMP].rc.rocker_l_ / 0.151f * 3.0f);
}

/**
 * @brief 紧急断电状态
 */
static void Emergency_Stop(void)
{
    chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
    chassis_cmd_send.track = TRACK_NONE;
    gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
    shoot_cmd_send.shoot_mode = SHOOT_OFF;
    shoot_cmd_send.friction_mode = FRICTION_OFF;
    shoot_cmd_send.load_mode = LOAD_STOP;
    LED_Red_Up;
}
