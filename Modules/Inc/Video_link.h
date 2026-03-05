//
// Created by lenovo on 26-2-9.
//

#ifndef VIDEO_LINK_H
#define VIDEO_LINK_H

#include "main.h"

// 用于遥控器数据读取,遥控器数据是一个大小为2的数组
#define LAST 1
#define TEMP 0

// 获取按键操作
#define KEY_PRESS 0
#define KEY_STATE 1
#define KEY_PRESS_WITH_CTRL 1
#define KEY_PRESS_WITH_SHIFT 2

// 检查接收值是否出错
#define RC_CH_VALUE_MIN ((uint16_t)364)
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define RC_CH_VALUE_MAX ((uint16_t)1684)

/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP ((uint16_t)1)   // 开关向上时的值
#define RC_SW_MID ((uint16_t)3)  // 开关中间时的值
#define RC_SW_DOWN ((uint16_t)2) // 开关向下时的值


#define LONG_PRESS_TICK_THRESH  30  // 长按判定阈值 (约1秒, 假设30Hz频率)

typedef enum
{
    KEY_RELEASED = 0,   // 按键未按下 / 松开状态
    KEY_PRESS_DOWN,     // 按键刚刚按下 (上升沿，只维持一帧)
    KEY_PRESSING,       // 按键持续按下 (短按期间)
    KEY_LONG_PRESS,     // 按键长按 (持续按下时间超过阈值)
    KEY_PRESS_UP        // 按键刚刚松开 (下降沿，只维持一帧)
} KeyState_e;

typedef struct
{
    KeyState_e state;   // 当前状态
    uint16_t hold_tick; // 按下持续时间计数器
} KeyStateMachine_t;

#pragma pack(1)

typedef union
{
    struct // 用于访问键盘状态
    {
        uint16_t w : 1;
        uint16_t s : 1;
        uint16_t a : 1;
        uint16_t d : 1;
        uint16_t shift : 1;
        uint16_t ctrl : 1;
        uint16_t q : 1;
        uint16_t e : 1;
        uint16_t r : 1;
        uint16_t f : 1;
        uint16_t g : 1;
        uint16_t z : 1;
        uint16_t x : 1;
        uint16_t c : 1;
        uint16_t v : 1;
        uint16_t b : 1;
    };
    uint16_t keys; // 用于memcpy而不需要进行强制类型转换
}__attribute__((packed)) Video_Key_u;

typedef struct
{
    uint8_t sof_1;
    uint8_t sof_2;
    uint64_t ch_0:11;
    uint64_t ch_1:11;
    uint64_t ch_2:11;
    uint64_t ch_3:11;
    uint64_t mode_sw:2;
    uint64_t pause:1;
    uint64_t fn_1:1;
    uint64_t fn_2:1;
    uint64_t wheel:11;
    uint64_t trigger:1;

    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    uint8_t mouse_left:2;
    uint8_t mouse_right:2;
    uint8_t mouse_middle:2;
    Video_Key_u key;
    uint16_t crc16;
}__attribute__((packed)) vl_remote_data_t ;

typedef struct
{
    struct
    {
        int16_t rocker_l_;  // 左水平
        int16_t rocker_l1;  // 左竖直
        int16_t rocker_r_;  // 右水平
        int16_t rocker_r1;  // 右竖直
        int16_t wheel;      // 侧边拨轮
        uint8_t mode_sw;    // 模式开关
        uint8_t pause;      // 暂停开关
        uint8_t fn_1;       // 自定义功能键1
        uint8_t fn_2;       // 自定义功能键2
        uint8_t trigger;    // 扳机键
    } rc;
    struct
    {
        int16_t x; //向左为负
        int16_t y; //向下为负
        int16_t z; //向后为负
        uint8_t mouse_left;
        uint8_t mouse_right;
        uint8_t mouse_middle;
    } mouse;

    Video_Key_u key[3]; // 改为位域后的键盘索引,空间减少8倍,速度增加16~倍

    uint8_t key_count[3][16];

    // 16个按键独立的状态机 (对应 Video_Key_u 的16个位)
    KeyStateMachine_t key_fsm[16];
} VL_ctrl_t;

#pragma pack()

/**
 * @brief 初始化图传遥控器,该函数会将遥控器注册到串口
 * @param vl_rc_usart_handle
 * @return
 * @attention 注意分配正确的串口硬件,图传遥控器的波特率为921600,数据位8,停止位1,无校验
 */
VL_ctrl_t *VLRemoteControlInit(UART_HandleTypeDef *vl_rc_usart_handle);

#endif //VIDEO_LINK_H
