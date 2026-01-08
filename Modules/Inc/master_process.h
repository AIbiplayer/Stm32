#ifndef MASTER_PROCESS_H
#define MASTER_PROCESS_H

#ifdef MASTER_PROCESS_H

#define VISION_RECV_SIZE 18u // 当前为固定值,36字节
#define VISION_SEND_SIZE 64u // 发送缓冲区最大64字节，可更改,足够大即可

#pragma pack(1)
typedef enum
{
    NO_FIRE = 0,
    AUTO_FIRE = 1,
    AUTO_AIM = 2
} Fire_Mode_e; // 开火模式枚举

typedef enum
{
    NO_TARGET = 0,
    TARGET_CONVERGING = 1,
    READY_TO_FIRE = 2
} Target_State_e; // 目标状态枚举

typedef enum
{
    NO_TARGET_NUM = 0,
    HERO1 = 1,
    ENGINEER2 = 2,
    INFANTRY3 = 3,
    INFANTRY4 = 4,
    INFANTRY5 = 5,
    OUTPOST = 6,
    SENTRY = 7,
    BASE = 8
} Target_Type_e; // 目标类型枚举

typedef struct
{
    // Fire_Mode_e fire_mode;
    // Target_State_e target_state;
    // Target_Type_e target_type;

    float pitch;
    float yaw;
} Vision_Recv_s; // 接收数据结构体

typedef enum
{
    COLOR_NONE = 0,
    COLOR_BLUE = 1,
    COLOR_RED = 2,
} Enemy_Color_e; // 敌方颜色枚举

typedef enum
{
    VISION_MODE_AIM = 0,
    VISION_MODE_SMALL_BUFF = 1,
    VISION_MODE_BIG_BUFF = 2
} Work_Mode_e; // 工作模式枚举

typedef enum
{
    BULLET_SPEED_NONE = 0,
    BIG_AMU_10 = 10,
    SMALL_AMU_15 = 15,
    BIG_AMU_16 = 16,
    SMALL_AMU_18 = 18,
    SMALL_AMU_24 = 24,
} Bullet_Speed_e; // 子弹速度枚举

typedef struct
{
    // Enemy_Color_e enemy_color;
    // Work_Mode_e work_mode;
    // Bullet_Speed_e bullet_speed;

    float yaw;
    float pitch;
    float roll;
} Vision_Send_s; // 发送数据结构体
#pragma pack()

/**
 * @brief 调用此函数初始化和视觉的串口通信
 */
Vision_Recv_s *VisionInit(void);

/**
 * @brief 发送视觉数据
 *
 */
void VisionSend(void);

/**
 * @brief 设置视觉发送标志位
 *
 * @param enemy_color
 * @param work_mode
 * @param bullet_speed
 */
void VisionSetFlag(Enemy_Color_e enemy_color, Work_Mode_e work_mode, Bullet_Speed_e bullet_speed);

/**
 * @brief 设置发送数据的姿态部分
 * @param yaw
 * @param pitch
 */
void VisionSetAltitude(float yaw, float pitch, float roll);

#endif

#endif // !MASTER_PROCESS_H
