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
#include "DM_Motor.h"
#include "cmsis_os.h"
#include "can_comm.h"
#include "remote_control.h"
#include "message_center.h"
#include "robot_def.h"
#include "TMC.h"

CCMRAM RC_ctrl_t *RC_data; // 遥控器数据,初始化时返回
CCMRAM CANCommInstance *CANCOM; // 底盘或云台的CAN通信实例指针

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

CCMRAM static PID_Typedef UPPID; // 上台阶履带PID
CCMRAM static Vision_Recv_s *vision_recv_data; // 视觉接收数据

extern float HC_Measure; // 超声波测距值
extern USB_Control_t g_usb_dev; // 全局USB设备实例

/**
 * @brief 命令读取与发送FreeRTOS任务
 * @note DJI电机控制函数在此调用
 */
void CmdTask(void *argument) {
    taskENTER_CRITICAL();
    DWT_Init(168);
    Robot_Cmd_Init();
    MX_USB_DEVICE_Init();
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12, GPIO_PIN_RESET); //USB DP拉低，重新枚举
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

#ifdef MCU_CHASSIS
        DM_Motor_Control();
        Gimbal_Data.distance = HC_Measure;
        CANCommSend(CANCOM, (uint8_t *) &Gimbal_Data);
#elifdef MCU_GIMBAL // @todo 目前只是云台板发送，底盘板不发送，之后的数据交互可以再调整
        CANCommSend(CANCOM, (uint8_t *) &Chassis_Data);
        VisionSend();

        PubPushMessage(shoot_cmd_pub, (void *) &shoot_cmd_send);
        PubPushMessage(gimbal_cmd_pub, (void *) &gimbal_cmd_send);
#endif

        osDelay(4);
    }
}

/**
 * @brief 机甲命令初始化
 */
static void Robot_Cmd_Init(void) {
    RC_data = RemoteControlInit(&huart3);
    CANCOM = CANCommInit(&TMC_CANComm_Config);
    vision_recv_data = VisionInit();

    // 上台阶履带PID参数
    PID_Param(&UPPID, -6.0f, -3.0f, 0.0f,
              Integral_Limit | Derivative_On_Measurement,
              1.0f, 10, 10, 90);

    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    shoot_cmd_pub = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    shoot_feed_sub = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
}

/**
 * @brief 根据gimbal app传回的当前电机角度计算和零位的误差
 *        单圈绝对角度的范围是0~360,说明文档中有图示
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
 * @brief 命令解析
 */
static void Robot_Cmd_Serve(void) {
    // 左右两杆均拨下，紧急断电
    if (switch_is_down(RC_data[TEMP].rc.switch_left) && switch_is_down(RC_data[TEMP].rc.switch_right)) {
        Emergency_Stop();
        return;
    }
    // 射击指令
    chassis_cmd_send.chassis_last_mode = chassis_cmd_send.chassis_mode;
    shoot_cmd_send.shoot_mode = SHOOT_ON;
    LED_Red_Down;

    // 左右拨杆均拨上，键盘控制,否则遥控器控制
    if (switch_is_up(RC_data[TEMP].rc.switch_left) && switch_is_up(RC_data[TEMP].rc.switch_right))
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
 */
static void Keyboard_Cmd(void) {
    static float max_speed;
    //WASD方向平移
    if (RC_data[TEMP].key[KEY_PRESS].w)
        chassis_cmd_send.vy = (chassis_cmd_send.vy < max_speed) ? (chassis_cmd_send.vy += 2.0f) : max_speed;
    else if (RC_data[TEMP].key[KEY_PRESS].s)
        chassis_cmd_send.vy = (chassis_cmd_send.vy > -max_speed) ? (chassis_cmd_send.vy -= 2.0f) : -max_speed;
    else chassis_cmd_send.vy = 0;

    if (RC_data[TEMP].key[KEY_PRESS].d)
        chassis_cmd_send.vx = (chassis_cmd_send.vx > max_speed) ? (chassis_cmd_send.vx += 2.0f) : max_speed;
    else if (RC_data[TEMP].key[KEY_PRESS].a)
        chassis_cmd_send.vx = (chassis_cmd_send.vx > -max_speed) ? (chassis_cmd_send.vx -= 2.0f) : -max_speed;
    else
        chassis_cmd_send.vx = 0;

    if (RC_data[TEMP].key[KEY_PRESS].shift) {
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
    } else {
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
    }
    //	switch(RC_data[TEMP].key_count[KEY_PRESS][Key_C]%3)//C键切换底盘模式
    //	{
    //		case 0:
    //			chassis_cmd_send.chassis_mode=CHASSIS_INDEPENDENCE;
    //			chassis_cmd_send.wz = RC_data[TEMP].key[KEY_PRESS].q *15.0f  - RC_data[TEMP].key[KEY_PRESS].e * 15.0f;//QE左右旋
    //			break;
    //		case 1:
    //			chassis_cmd_send.chassis_mode=CHASSIS_FOLLOW_GIMBAL_YAW;
    //			break;
    //		default:
    //			chassis_cmd_send.chassis_mode=CHASSIS_ROTATE;
    //	}
    if (RC_data[TEMP].mouse.press_r) //长按鼠标右键进入自瞄模式
    {
        gimbal_cmd_send.gimbal_mode = GIMBAL_VISION; //只做头部跟随
        if (pTemp != vision->theta_pitch) {
            pTemp = vision->theta_pitch;
            gimbal_cmd_send.pitch = gimba_IMU_data->Pitch - vision->theta_pitch;
        }
        if (yTemp != vision->theta_yaw) {
            yTemp = vision->theta_yaw;
            gimbal_cmd_send.yaw = gimba_IMU_data->YawTotalAngle - vision->theta_yaw;
        }
    } else //不按右键云台自由移动
    {
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
        gimbal_cmd_send.yaw += (float) RC_data[TEMP].mouse.x / 660 * 2.4f; //3.5
        gimbal_cmd_send.pitch += -(float) RC_data[TEMP].mouse.y / 660 * 1.3f; //-2.5
    }
    //	switch(RC_data[TEMP].key_count[KEY_PRESS][Key_R]%2)//R键切换打弹模式 连\单
    //	{
    //		case 0:
    //			shoot_cmd_send.load_mode=LOAD_BURSTFIRE;
    //		break;
    //		default:
    //			shoot_cmd_send.load_mode=LOAD_1_BULLET;
    //		break;
    //	}
    switch (RC_data[TEMP].key_count[KEY_PRESS][Key_F] % 2) // F键开关摩擦轮
    {
        case 0:
            shoot_cmd_send.friction_mode = FRICTION_OFF;
            shoot_cmd_send.shoot_mode = SHOOT_OFF;
            break;
        default:
            shoot_cmd_send.friction_mode = FRICTION_ON;
            shoot_cmd_send.shoot_mode = SHOOT_ON;
            break;
    }
    switch (RC_data[TEMP].key_count[KEY_PRESS][Key_X] % 2) //唤起UI,UI任务在UITASK
    {
        case 0:
            KEY_X = 0;
            break;
        case 1:
            KEY_X = 1;
            break;
    }
    switch (RC_data[TEMP].key_count[KEY_PRESS][Key_V] % 4) //V键调整速度
    {
        case 0:
            max_speed = 100.0f;
            break;
        case 1:
            max_speed = 150.0f;
            break;
        case 2:
            max_speed = 200.0f;
            break;
        default:
            max_speed = 250.0f;
    }
    //	if(RC_data[TEMP].mouse.press_l)//按下按键才开启拨盘
    //	{
    //		return;
    //	}
    //	else
    //	{
    //		shoot_cmd_send.load_mode=LOAD_STOP;//不按关闭
    //	}
    if (RC_data[TEMP].mouse.press_l) //鼠标左键短按单发，长按连发
    {
        //		current_time=DWT_GetTimeline_ms();
        //		if(shoot_cmd_send.load_mode==LOAD_STOP)
        //		{
        //			mouse_press_time=current_time;//获取时间
        //			shoot_cmd_send.load_mode=LOAD_1_BULLET;
        //
        //		}
        //		else if(shoot_cmd_send.load_mode==LOAD_1_BULLET)
        //		{
        //			if(current_time-mouse_press_time>=Threshold_time)
        //			{
        shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
        shoot_cmd_send.shoot_rate = 1.9f;
        //			}
        //		}
    } else {
        shoot_cmd_send.load_mode = LOAD_STOP;
    }
}

/**
 * @brief 遥控器控制指令解析
 */
static void RemoteControl_Cmd(void) {
    abs(RC_data[TEMP].rc.dial) > 100 // 拨盘大于100 启动电机
        ? (shoot_cmd_send.friction_mode = FRICTION_ON)
        : (shoot_cmd_send.friction_mode = FRICTION_OFF);
    if (RC_data[TEMP].rc.dial > 500)
        shoot_cmd_send.load_mode = LOAD_BURSTFIRE; //连发
    else if (RC_data[TEMP].rc.dial < -500)
        shoot_cmd_send.load_mode = LOAD_1_BULLET; //单发
    else
        shoot_cmd_send.load_mode = LOAD_STOP; //停止

    static uint8_t up_count, down_count, flag = 0; //高度差计数，flag履带状态标志
    //左杆在下，右杆在中，底盘自由控制，云台不控制
    if (switch_is_down(RC_data[TEMP].rc.switch_left) && switch_is_mid(RC_data[TEMP].rc.switch_right)) {
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
        gimbal_cmd_send.gimbal_mode = GIMBAL_FREE_MODE;
        chassis_cmd_send.track = TRACK_NONE;
        flag = 0;
    }
    //左杆在下，右杆在上，底盘随云台控制
    else if (switch_is_down(RC_data[TEMP].rc.switch_left) && switch_is_up(RC_data[TEMP].rc.switch_right)) {
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
        chassis_cmd_send.track = TRACK_NONE;
        flag = 0;
    }
    // 左杆在中，右杆在上，履带为降下模式
    else if (switch_is_mid(RC_data[TEMP].rc.switch_left) && switch_is_up(RC_data[TEMP].rc.switch_right)) {
        chassis_cmd_send.chassis_mode = CHASSIS_INDEPENDENCE;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
        chassis_cmd_send.track = TRACK_EXTEND;
        flag = 0;
    }
    // 左杆在中，右杆在中，履带为上台阶模式
    else if (switch_is_mid(RC_data[TEMP].rc.switch_left) && switch_is_mid(RC_data[TEMP].rc.switch_right)) {
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
        gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
        chassis_cmd_send.track = TRACK_UP;
    }
    // 左杆在上，右杆在中，开启自瞄模式
    else if (switch_is_up(RC_data[TEMP].rc.switch_left) && switch_is_mid(RC_data[TEMP].rc.switch_right)) {
        chassis_cmd_send.chassis_mode = CHASSIS_INDEPENDENCE;
        gimbal_cmd_send.gimbal_mode = GIMBAL_VISION;
        chassis_cmd_send.track = TRACK_NONE;
        flag = 0;
    }
    if (flag == 2)
        chassis_cmd_send.track = TRACK_NONE;
    // @todo 履带控制指令，由于遥控器中云台和履带共用右拨杆，目前只能使用一个
    switch (chassis_cmd_send.track) {
        case TRACK_UP:
            up_count = HC_Measure > 20.0f && flag == 0 ? up_count + 1 : 0;
            down_count = HC_Measure < 10.0f && flag == 1 ? down_count + 1 : 0;
            flag = up_count > 20 ? 1 : flag;
            flag = down_count > 20 ? 2 : flag;

            chassis_cmd_send.a_track_head += (float) RC_data[TEMP].rc.rocker_r1 * 0.00034f;
            chassis_cmd_send.a_track_head = Angle_limit(chassis_cmd_send.a_track_head, 170.0f, 0.0f);
            chassis_cmd_send.a_track_back = 105 + PID_Calculate(&UPPID, 0.0f, gimbal_fetch_data.gimbal_imu_data.Roll);
            chassis_cmd_send.a_track_back = Angle_limit(chassis_cmd_send.a_track_back, 170.0f, 105.0f);
            break;
        case TRACK_EXTEND:
            chassis_cmd_send.a_track_head = 170.0f;
            chassis_cmd_send.a_track_back = 170.0f;
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
    // 跟随云台转动，右拨杆控制云台俯仰和偏航
    if (gimbal_cmd_send.gimbal_mode == GIMBAL_GYRO_MODE) {
        gimbal_cmd_send.yaw -= 0.00034f * (float) RC_data[TEMP].rc.rocker_r_;
        gimbal_cmd_send.pitch += 0.0001f * (float) RC_data[TEMP].rc.rocker_r1;
        gimbal_cmd_send.pitch = Angle_limit(gimbal_cmd_send.pitch, 20.0f, -35.0f);
    }
    // 视觉模式
    else if (gimbal_cmd_send.gimbal_mode == GIMBAL_VISION) {
        gimbal_cmd_send.yaw = vision_recv_data->yaw;
        gimbal_cmd_send.pitch = vision_recv_data->pitch;
        gimbal_cmd_send.pitch = Angle_limit(gimbal_cmd_send.pitch, 20.0f, -35.0f);
    }
    // 自由模式,右拨杆控制底盘旋转
    else if (gimbal_cmd_send.gimbal_mode == GIMBAL_FREE_MODE) {
        chassis_cmd_send.wz = (float) RC_data[TEMP].rc.rocker_r_ * 0.151f;
    }

    chassis_cmd_send.vy = (float) RC_data[TEMP].rc.rocker_l1 * 0.151f * 30.0f; // 最高3m/s
    chassis_cmd_send.chassis_mode == CHASSIS_INDEPENDENCE
        ? (chassis_cmd_send.wz = (float) RC_data[TEMP].rc.rocker_l_ * 0.00151f * 3.0f) // 最高3转/s
        : (chassis_cmd_send.vx = -(float) RC_data[TEMP].rc.rocker_l_ * 0.151f * 30.0f); // 最高3m/s
}
