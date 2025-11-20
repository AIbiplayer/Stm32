/**
* @file Robot_Cmd_Task.c
 * @brief 机甲控制程序
 * @author Shen FeiLin
 * @date 2025/10/29
 */

#include "bsp_dwt.h"
#include "main.h"
#include "bsp_usart.h"
#include "cmsis_os.h"
#include "remote_control.h"
#include "message_center.h"
#include "robot_def.h"
#include "Vofa_Debug.h"

RC_ctrl_t* RC_data; // 遥控器数据,初始化时返回

extern osSemaphoreId RC_Parse_FlagHandle;
extern USARTInstance* rc_usart_instance;
extern attitude_t* Gimbal_IMU_Data; ///< 云台IMU数据

/* cmd应用包含的模块实例指针和交互信息存储*/
static Publisher_t* chassis_cmd_pub; // 底盘控制消息发布者
static Subscriber_t* chassis_feed_sub; // 底盘反馈信息订阅者
static Chassis_Ctrl_Cmd_s chassis_cmd_send; // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Chassis_Upload_Data_s chassis_fetch_data; // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等

static Publisher_t* gimbal_cmd_pub; // 云台控制消息发布者
static Subscriber_t* gimbal_feed_sub; // 云台反馈信息订阅者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send; // 传递给云台的控制信息
static Gimbal_Upload_Data_s gimbal_fetch_data; // 从云台获取的反馈信息

static void Robot_Cmd_Init(void);
static void Emergency_Stop(void);
static void CalcOffsetAngle(void);
static void Remote_Control_Cmd_Serve(void);

/**
 * @brief 命令读取与发送FreeRTOS任务
 * @note DJI电机控制函数在此调用
 */
void CmdTask(void* argument)
{
    DWT_Init(168);
    Robot_Cmd_Init();

    taskENTER_CRITICAL();
    Gimbal_IMU_Data = INS_Init();
    taskEXIT_CRITICAL();
    HAL_GPIO_WritePin(LED_B_GPIO_Port,LED_B_Pin, GPIO_PIN_SET);

    for (;;)
    {
        SubGetMessage(chassis_feed_sub, (void*)&chassis_fetch_data);
        SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);

        CalcOffsetAngle();
        Remote_Control_Cmd_Serve();
        DJI_Motor_Control();

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

    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
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
    HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin, GPIO_PIN_RESET);
    //两杆为中间，小陀螺模式
    if (switch_is_mid(RC_data[TEMP].rc.switch_left) && switch_is_mid(RC_data[TEMP].rc.switch_right))
    {
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
        chassis_cmd_send.chassis_last_mode = CHASSIS_ROTATE;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    }
    // 两杆为上边，随云台转动
    else if (switch_is_up(RC_data[TEMP].rc.switch_left) && switch_is_up(RC_data[TEMP].rc.switch_right))
    {
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
        chassis_cmd_send.chassis_last_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    }
    if (gimbal_cmd_send.gimbal_mode == GIMBAL_GYRO_MODE)
    {
        gimbal_cmd_send.yaw -= 0.00034f * (float)RC_data[TEMP].rc.rocker_r_;
        // gimbal_cmd_send.pitch += 0.00078f * (float)RC_data[TEMP].rc.rocker_l1;
    }
    chassis_cmd_send.vy = (float)RC_data->rc.rocker_l1 * 2;
    chassis_cmd_send.chassis_mode == CHASSIS_INDEPENDENCE
        ? (chassis_cmd_send.wz = 0.3f * (float)RC_data[TEMP].rc.rocker_l_)
        : (chassis_cmd_send.vx = (float)RC_data[TEMP].rc.rocker_l_ * 2);
}

/**
 * @brief 紧急断电状态
 */
static void Emergency_Stop(void)
{
    chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
    gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
    HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
}
