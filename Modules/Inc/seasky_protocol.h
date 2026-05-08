#ifndef __SEASKY_PROTOCOL_H
#define __SEASKY_PROTOCOL_H

#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>

#define PROTOCOL_CMD_ID 0X55
#define OFFSET_BYTE 6 // 出数据段外，其他部分所占字节数

#pragma pack(1)

typedef enum {
    RECEIVE_GIMBAL_POSITION = 0x10, // 100HZ
    RECEIVE_ENEMY_INFORMATION = 0x11, // 10HZ
} Rx_Data_type_e;

typedef enum {
    TX_GIMBAL_REALTIME = 0x60, // 400HZ
    TX_GIMBAL_N_REALTIME = 0X61 // 100HZ
} Tx_Data_type_e;

typedef enum {
    AIM_NORMAL = 0, //常规自瞄
    AIM_SMALL_ENERGY_BUFF, //小能量机关自瞄
    AIM_BIG_ENERGY_BUFF, //大能量机关自瞄
} Aimbot_Mode_e;

typedef struct {
    uint8_t reserved_2: 2; //保留位
    uint8_t loader_frequency: 5; //开火频率，单位为发/s
    uint8_t fire_flag: 1; //发射标志位，0为不发射，1为发射
    float yaw; //云台相对于车身的yaw角度，弧度制，范围为- pi ~ pi
    float pitch; //云台相对于车身的pitch角度，弧度制，范围为- pi ~ pi
} Rx_Gimbal_Data_s;

typedef struct {
    uint8_t reserved_1: 6; //保留位
    uint8_t distance_status: 1; //距离状态，0为距离正常
    uint8_t isfind_enemy: 1; //是否找到敌人,0为未找到，1为找到
} Enemy_Information_Data_s;

typedef struct {
    Rx_Data_type_e cmd_data_type; //指令类型
    Rx_Gimbal_Data_s gimbal_data; //上位机发送的云台数据
    Enemy_Information_Data_s enemy_data; //上位机发送的敌人信息数据
} Vision_Recv_s;

typedef struct {
    float yaw; //云台相对于车身的yaw角度,- pi ~ pi
    float little_pitch; //云台相对于车身的pitch角度，-pi ~ pi
    float big_pitch; //云台pitch和yaw的差值，-pi ~ pi
    float speedx; //云台x轴速度，单位为m/s
    float speedy; //云台y轴速度，单位为m/s
    float accelx; //云台x轴加速度，单位为m/s^2
    float accely; //云台y轴加速度，单位为m/s^2
} Tx_Gimbal_Data_s;

typedef struct {
    uint8_t heat: 4; //当前热量十分比
    uint8_t auto_find: 1; //自动扫描,0为不扫描，1为扫描
    uint8_t my_color: 1; //我方颜色,0为红色，1为蓝色
    uint8_t mode: 2; //当前模式,0为自瞄，1为前哨站自瞄，2为能量机关自瞄
    float bullet_speed; //子弹速度，单位为m/s
} Tx_NRT_Data_s; // 非实时数据

typedef struct {
    Tx_Data_type_e cmd_data_type; //命令码
    Tx_Gimbal_Data_s gimbal_data; //控制板发送的云台数据
    Tx_NRT_Data_s nrt_data; // 控制板发送的非实时数据
} Vision_Send_s;

typedef struct {
    struct {
        uint8_t sof;
        uint16_t data_length;
    } header; // 数据帧头
    // uint16_t cmd_id;	   // 数据ID
} protocol_rm_struct;

#pragma pack()

/*更新发送数据帧，并计算发送数据帧长度*/
void get_protocol_send_data(
    Vision_Send_s *tx_data, // 待发送的float数据
    uint8_t *tx_buf, // 待发送的数据帧
    uint16_t *tx_buf_len); // 待发送的数据帧长度

/*接收数据处理*/
uint16_t get_protocol_info(uint8_t *rx_buf, // 接收到的原始数据
                           Vision_Recv_s *rx_data); // 接收的float数据存储地址

#endif
