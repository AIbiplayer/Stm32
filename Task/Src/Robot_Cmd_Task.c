/**
* @file Robot_Cmd_Task.c
 * @brief 机甲控制程序
 * @author Shen FeiLin
 * @date 2025/10/29
 */

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

/* cmd应用包含的模块实例指针和交互信息存储*/
static Publisher_t* chassis_cmd_pub; // 底盘控制消息发布者
static Subscriber_t* chassis_feed_sub; // 底盘反馈信息订阅者
static Chassis_Ctrl_Cmd_s chassis_cmd_send; // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Chassis_Upload_Data_s chassis_fetch_data; // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等

static void Robot_Cmd_Init(void);
static void Remote_Control_Cmd_Serve(void);

/**
 * @brief 命令读取与发送FreeRTOS任务
 */
void CmdTask(void* argument)
{
    Robot_Cmd_Init();
    for (;;)
    {
        Remote_Control_Cmd_Serve();
        SubGetMessage(chassis_feed_sub, (void*)&chassis_fetch_data);
        PubPushMessage(chassis_cmd_pub, (void*)&chassis_cmd_send);
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
}

/**
 * @brief 遥控器命令解析
 * @todo 目前只有对底盘的控制，后续增加对云台和射击等控制
 */
static void Remote_Control_Cmd_Serve(void)
{
    if (switch_is_down(RC_data[TEMP].rc.switch_left) && switch_is_down(RC_data[TEMP].rc.switch_right))
    {
    }
    else
}
