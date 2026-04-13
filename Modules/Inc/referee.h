#ifndef RM_REFEREE_H
#define RM_REFEREE_H

#include "usart.h"
#include "referee_protocol.h"
#include "robot_def.h"
#include "bsp_usart.h"
#include "stdbool.h"
#include "FreeRTOS.h"

#pragma pack(1)
typedef struct {
    uint8_t Robot_Color; // 机器人颜色
    uint16_t Robot_ID; // 本机器人ID
    uint16_t Cilent_ID; // 本机器人对应的客户端ID
    uint16_t Receiver_Robot_ID; // 机器人车间通信时接收者的ID，必须和本机器人同颜色
} referee_id_t;

// 此结构体包含裁判系统接收数据以及UI绘制与机器人车间通信的相关信息
typedef struct {
    referee_id_t referee_id;

    xFrameHeader FrameHeader; // 接收到的帧头信息
    uint16_t CmdID;
    ext_game_state_t GameState; /* ID: 0x0001  Byte: 11    比赛状态数据 */
    ext_game_result_t GameResult; /* ID: 0x0002  Byte:  1    比赛结果数据 */
    ext_game_robot_HP_t GameRobotHP; /* ID: 0x0003  Byte: 16    比赛机器人血量数据 */
    ext_event_data_t EventData; /* ID: 0x0101  Byte:  4    场地事件数据 */
    ext_referee_warning_t RefereeWarning; /* ID: 0x0104  Byte:  3    裁判警告数据，己方判罚/判负时触发发送，其余时间以1Hz频率发送 */
    ext_dart_info_t DartInfo; /* ID: 0x0105  Byte:  3    飞镖发射相关数据，固定以1Hz频率发送 */
    ext_game_robot_state_t GameRobotState; /* ID: 0X0201  Byte: 13    机器人状态数据 */
    ext_power_heat_data_t PowerHeatData; /* ID: 0X0202  Byte: 14    实时底盘缓冲能量和射击热量数据，固定以10Hz频率发送 */
    ext_game_robot_pos_t GameRobotPos; /* ID: 0x0203  Byte: 12    机器人位置数据 */
    ext_buff_t BuffMusk; /* ID: 0x0204  Byte:  8    机器人增益数据 */
    ext_hurt_data_t RobotHurt; /* ID: 0x0206  Byte:  1    伤害状态数据，伤害发生后发送 */
    ext_shoot_data_t ShootData; /* ID: 0x0207  Byte:  7    实时射击数据 */
    ext_projectile_allowance_t ProjectileAllowance; /* ID: 0x0208  Byte:  8    允许发弹量，固定以10Hz频率发送 */
    ext_map_command_t MapCommand; /* ID: 0x0303  Byte: 12    选手端小地图交互数据，频率上限为3Hz  */

    // 自定义交互数据的接收
    Communicate_ReceiveData_t ReceiveData;

    bool referee_trust_state; // 裁判系统是否可信
    bool referee_online_state; // 裁判系统是否在线

    uint8_t init_flag;
} referee_info_t;

// 模式是否切换标志位，0为未切换，1为切换，static定义默认为0
typedef struct {
    uint32_t chassis_flag: 1;
    uint32_t gimbal_flag: 1;
    uint32_t shoot_flag: 1;
    uint32_t lid_flag: 1;
    uint32_t friction_flag: 1;
    uint32_t Power_flag: 1;
} Referee_Interactive_Flag_t;

// 此结构体包含UI绘制与机器人车间通信的需要的其他非裁判系统数据
typedef struct {
    Referee_Interactive_Flag_t Referee_Interactive_Flag;
    // 为UI绘制以及交互数据所用
    chassis_mode_e chassis_mode; // 底盘模式
    gimbal_mode_e gimbal_mode; // 云台模式
    shoot_mode_e shoot_mode; // 发射模式设置
    friction_mode_e friction_mode; // 摩擦轮关闭
    Chassis_Power_Data_s Chassis_Power_Data; // 功率控制
    lid_mode_e lid_mode; // 盖子开关


    // 上一次的模式，用于flag判断
    chassis_mode_e chassis_last_mode;
    gimbal_mode_e gimbal_last_mode;
    shoot_mode_e shoot_last_mode;
    friction_mode_e friction_last_mode;
    lid_mode_e lid_last_mode;
    Chassis_Power_Data_s Chassis_last_Power_Data;
} Referee_Interactive_info_t;

#pragma pack()

/**
 * @brief 裁判系统通信初始化,该函数会初始化裁判系统串口,开启中断
 * @param referee_usart_handle 串口handle,C板一般用串口6
 * @return referee_info_t* 返回裁判系统反馈的数据,包括热量/血量/状态等
 */
referee_info_t *Referee_Init(UART_HandleTypeDef *referee_usart_handle);

/**
 * @brief UI绘制和交互数的发送接口,由UI绘制任务和多机通信函数调用
 * @note 内部包含了一个实时系统的延时函数,这是因为裁判系统接收CMD数据至高位10Hz
 * @param send 发送数据首地址
 * @param tx_len 发送长度
 */
void RefereeSend(uint8_t *send, uint16_t tx_len);

void RefereeLostCallback(void);

#endif // !REFEREE_H
