//#pragma once // 可以用#pragma once代替#ifndef ROBOT_DEF_H(header guard)
#ifndef ROBOT_DEF_H
#define ROBOT_DEF_H

#include "master_process.h"
#include "stdint.h"
#include "DJI_Motor.h"
#include "INS.h"

#define ONE_BOARD // 单板控制整车
#define VISION_USE_VCP  // 使用虚拟串口发送视觉数据
// #define VISION_USE_UART // 使用串口发送视觉数据

/* 机器人重要参数定义,注意根据不同机器人进行修改,浮点数需要以.0或f结尾,无符号以u结尾 */
#define PI               3.14159265358979f

// 超声波测距
#define MIN_DISTANCE 10.0f // 最小测距距离,单位cm
#define MAX_DISTANCE 20.0f // 最大测距距离,单位cm

// 云台参数
#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI) // 对齐时的角度,0-360
#define YAW_CHASSIS_ALIGN_ECD 1800// 云台和底盘对齐指向相同方向时的电机编码器值,若对云台有机械改动需要修改
#define YAW_ECD_GREATER_THAN_4096 0 // ALIGN_ECD值是否大于4096,是为1,否为0;用于计算云台偏转角度
#define PITCH_HORIZON_ECD 2304      // 云台处于水平位置时编码器值,若对云台有机械改动需要修改
#define PITCH_MAX_ANGLE 20.90f           // 云台竖直方向最大角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)
#define PITCH_MIN_ANGLE -23.00f           // 云台竖直方向最小角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)
// 发射参数
#define SHOOT_COMPENSATION_K 1.5f // 发射补偿系数,用于调节发射速度与距离的关系 todo 具体数值待调试
#define ONE_BULLET_DELTA_ANGLE  36.00f   // 发射一发弹丸拨盘转动的距离,由机械设计图纸给出
#define REDUCTION_SHOOT 2.5f // 齿轮减速比
#define  RADIUS_FRICTION 30.0f // 摩擦轮半径,单位mm
#define REDUCTION_RATIO_LOADER 36.0f // 拨盘电机的减速比,2006减速比36：1
#define NUM_PER_CIRCLE 9            // 拨盘一圈的装载量
// 机器人底盘修改的参数,单位为mm(毫米)
#define WHEEL_BASE 311              // 纵向轴距(前进后退方向)
#define TRACK_WIDTH 297             // 横向轮距(左右平移方向)
#define CENTER_GIMBAL_OFFSET_X 0    // 云台旋转中心距底盘几何中心的距离,前后方向,云台位于正中心时默认设为0
#define CENTER_GIMBAL_OFFSET_Y 0    // 云台旋转中心距底盘几何中心的距离,左右方向,云台位于正中心时默认设为0
#define RADIUS_WHEEL 76.2f             // 轮子半径
#define RADS_2_RPM 9.54929658551f      // 弧度每秒转化为转每分钟的系数
#define REDUCTION_RATIO_WHEEL 19.0f // 3508电机减速比,因为编码器量测的是转子的速度而不是输出轴的速度故需进行转换，这也是M3519减速比
#define REDUCTION_TRACK 2.9f     // 履带减速比
#define MAX_ANGLE_TRACK 180.0f        // 履带最大转动角度

#define GYRO2GIMBAL_DIR_YAW 1   // 陀螺仪数据相较于云台的yaw的方向,1为相同,-1为相反
#define GYRO2GIMBAL_DIR_PITCH -1 // 陀螺仪数据相较于云台的pitch的方向,1为相同,-1为相反
#define GYRO2GIMBAL_DIR_ROLL 1  // 陀螺仪数据相较于云台的roll的方向,1为相同,-1为相反

#define DEGREE_2_RAD 0.01745329252f // pi/180
#define HALF_WHEEL_BASE (WHEEL_BASE / 2.0f)     // 半轴距
#define HALF_TRACK_WIDTH (TRACK_WIDTH / 2.0f)   // 半轮距
#define PERIMETER_WHEEL (RADIUS_WHEEL * 2 * PI) // 轮子周长
#define LF_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * 6.28f)
#define RF_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * 6.28f)
#define LB_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * 6.28f)
#define RB_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * 6.28f)

#define Qwarn 100//热量预警阈值
#define Qsatu 40//热量饱和阈值
#define Qthrsh 10//热量门限阈值
#define HEAT_OF_PROJECTILE 10//单发弹丸的热量(由2026赛季RM规则决定)

// 爆发优先热量打表
static const uint16_t booster_burst_first_heat_max[10] = {
    170,
    180,
    190,
    200,
    210,
    220,
    230,
    240,
    250,
    260,
};

// 爆发优先冷却打表
static const uint8_t booster_burst_first_heat_cd[10] = {
    5,
    7,
    9,
    11,
    12,
    13,
    14,
    16,
    18,
    20,
};

// 冷却优先热量打表
static const uint8_t booster_cd_first_heat_max[10] = {
    40,
    48,
    56,
    64,
    72,
    80,
    88,
    96,
    114,
    120,
};

// 冷却优先冷却打表
static const uint8_t booster_cd_first_heat_cd[10] = {
    12,
    14,
    16,
    18,
    20,
    22,
    24,
    26,
    28,
    30,
};

typedef enum {
    SHOOT_DETECTION_STOP = 0, //停止检测
    SHOOT_DETECTION_READY, //准备检测
} shoot_detection_e; //射击检测状态

typedef enum {
    Robot_Booster_Type_BURST = 0, //爆发优先
    Robot_Booster_Type_CD, //冷却优先
} Shooter_Type_e; //射击方式

#pragma pack(1) // 压缩结构体,取消字节对齐,下面的数据都可能被传输

// 应用状态
typedef enum {
    APP_OFFLINE = 0,
    APP_ONLINE,
    APP_ERROR,
} App_Status_e;

typedef enum {
    CHASSIS_ZERO_FORCE = 0, // 电流零输入
    CHASSIS_ROTATE, // 小陀螺模式
    CHASSIS_NO_FOLLOW, // 不跟随，允许全向平移
    CHASSIS_FOLLOW_GIMBAL_YAW, // 跟随模式，底盘叠加角度环控制
    CHASSIS_INDEPENDENCE,
} chassis_mode_e;

// 云台模式设置
typedef enum {
    GIMBAL_ZERO_FORCE = 0, // 电流零输入
    GIMBAL_FREE_MODE, // 云台自由运动模式,即与底盘分离(底盘此时应为NO_FOLLOW)反馈值为电机total_angle;似乎可以改为全部用IMU数据?
    GIMBAL_GYRO_MODE, // 云台陀螺仪反馈模式,反馈值为陀螺仪pitch,total_yaw_angle,底盘可以为小陀螺和跟随模式
    GIMBAL_VISION, //自瞄模式
} gimbal_mode_e;

// 发射模式设置
typedef enum {
    SHOOT_OFF = 0,
    SHOOT_ON,
} shoot_mode_e;

typedef enum {
    FRICTION_OFF = 0, // 摩擦轮关闭
    FRICTION_ON, // 摩擦轮开启
} friction_mode_e;

typedef enum {
    LID_OPEN = 0, // 弹舱盖打开
    LID_CLOSE, // 弹舱盖关闭
} lid_mode_e;

typedef enum {
    LOAD_STOP = 2, // 停止发射
    LOAD_REVERSE = 3, // 反转
    LOAD_1_BULLET = 1, // 单发
    LOAD_BURSTFIRE = 0, // 连发
} loader_mode_e;

// 功率限制,从裁判系统获取,是否有必要保留?
typedef struct {
    // 功率控制
    float chassis_power_mx;
} Chassis_Power_Data_s;

/* ----------------CMD应用发布的控制数据,应当由gimbal/chassis/shoot订阅---------------- */
/**
 * @brief 对于双板情况,遥控器和pc在云台,裁判系统在底盘
 *
 */
// cmd发布的底盘控制数据,由chassis订阅
typedef struct {
    // 控制部分
    float vx; // 前进方向速度
    float vy; // 横移方向速度
    float wz; // 旋转速度
    float offset_angle; // 底盘和归中位置的夹角
    chassis_mode_e chassis_mode: 3; // 底盘模式
    chassis_mode_e chassis_last_mode: 3; // 底盘上一次模式
    float angle_offset_c; //云台与底盘的角度差,底盘用

    Track_Mode_e track: 3; // 履带模式
    float a_track_head; // 履带前轮
    float a_track_back; // 履带后轮
} Chassis_Ctrl_Cmd_s;

// cmd发布的云台控制数据,由gimbal订阅
typedef struct {
    // 云台角度控制
    float yaw;
    float pitch;

    gimbal_mode_e gimbal_mode: 2; // 云台模式
    float angle_offset_g; //云台与底盘角度差，云台用
} Gimbal_Ctrl_Cmd_s;

// cmd发布的发射控制数据,由shoot订阅
typedef struct {
    shoot_mode_e shoot_mode;
    loader_mode_e load_mode;
    lid_mode_e lid_mode;
    friction_mode_e friction_mode;
    Bullet_Speed_e bullet_speed; // 弹速枚举
    uint8_t rest_heat;
    float shoot_rate; // 连续发射的射频,unit per s,发/秒
    uint8_t ONE_SHOOT_FLAG;
} Shoot_Ctrl_Cmd_s;

/* ----------------gimbal/shoot/chassis发布的反馈数据----------------*/
/**
 * @brief 由cmd订阅,其他应用也可以根据需要获取.
 *
 */

typedef struct {
    // 后续增加底盘的真实速度
    // float real_vx;
    // float real_vy;
    // float real_wz;
    // uint8_t rest_heat; // 剩余枪口热量
    // Bullet_Speed_e bullet_speed; // 弹速限制
    // Enemy_Color_e enemy_color; // 0 for blue, 1 for red
} Chassis_Upload_Data_s;


typedef struct {
    INS_t gimbal_imu_data;
    uint16_t yaw_motor_single_round_angle;
} Gimbal_Upload_Data_s;

typedef struct {
    uint16_t heat; // 枪口热量
    uint16_t shooter_heat_limit; // 枪口热量上限
    uint8_t shooter_barrel_cooling_value; // 枪管冷却值
    uint8_t robot_level: 7; // 机器人等级
    uint8_t reference_online_state: 1; // 参考数据在线状态
} Shoot_Upload_Data_s;

#pragma pack() // 开启字节对齐,结束前面的#pragma pack(1)

#endif // !ROBOT_DEF_H
