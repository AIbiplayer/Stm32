#ifndef __SEASKY_PROTOCOL_H
#define __SEASKY_PROTOCOL_H

#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>

#define PROTOCOL_CMD_ID 0X55
#define OFFSET_BYTE 6 // 出数据段外，其他部分所占字节数

#pragma pack(1)

typedef enum
{
	COLOR_BLUE = 0,
	COLOR_RED = 1,
} Enemy_Color_e;

typedef enum
{
	SMALL_AMU_22 = 0,
	SMALL_AMU_20 = 1,
	SMALL_AMU_18 = 2,
	SMALL_AMU_15 = 3,
} Bullet_Speed_e;

typedef enum
{
	RECEIVE_GIMBAL_POSITION = 0x10,
	RECEIVE_FIRE_CONTROL = 0x11,
}Rx_Data_type_e;

typedef enum
{
	TX_GIMBAL_POSITION = 0x60,
}Tx_Data_type_e;

typedef enum
{
	NO_FIRE = 0,
	AUTO_FIRE = 1,
	AUTO_AIM = 2
} Fire_Mode_e;

typedef enum
{
	NO_TARGET = 0,
	TARGET_CONVERGING = 1,
	READY_TO_FIRE = 2
} Target_State_e;

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
} Target_Type_e;

typedef enum
{
	AIM_OFF = 0,
	AIM_NORMAL = 1, //常规自瞄
	AIM_SMALL_ENERGY_BUFF = 2, //小能量机关自瞄
	AIM_BIG_ENERGY_BUFF = 3, //大能量机关自瞄

}Aimbot_Mode_e;

typedef struct
{
	float yaw; //云台相对于车身的yaw角度，弧度制，范围为- pi ~ pi
	float pitch; //云台相对于车身的pitch角度，弧度制，范围为- pi ~ pi
}Rx_Gimbal_Data_s;

typedef struct
{
	uint8_t reserved_2 : 4; //保留位
	uint8_t fire_flag : 1; //发射标志位，0为不发射，1为发射
	uint8_t find_target : 1; //发现目标标志位，0为未发现目标，1为发现目标
	uint8_t fire_mode_flag : 1; //发射模式标志位，0 为连发，1为单发
	uint8_t distance_limit_inside : 1; //目标是否在距离内标志位
	uint8_t loader_frequency; //开火频率，单位为发/s
}Fire_Control_Data_s;

typedef struct
{
	// Fire_Mode_e fire_mode;
	// Target_State_e target_state;
	// Target_Type_e target_type;

	Rx_Data_type_e cmd_data_type; //指令类型
	Rx_Gimbal_Data_s gimbal_data; //上位机发送的云台数据
	Fire_Control_Data_s fire_control_data; //上位机发送的发射控制数据

	//上次接收到的数据，用来对比
	Rx_Gimbal_Data_s last_gimbal_data; //上次上位机发送的云台数据

} Vision_Recv_s;


typedef struct
{
	float yaw; //云台相对于车身的yaw角度,- pi ~ pi
	float pitch; //云台相对于车身的pitch角度，-pi ~ pi
	uint8_t reserved : 4; //保留位
	bool enable_aimbot : 1; //是否开启自瞄
	Enemy_Color_e enemy_color : 1; //敌方颜色，0为蓝色，1为红色
	Aimbot_Mode_e aimbot_mode : 2; //自瞄模式

}Tx_Gimbal_Data_s;

typedef struct
{
	Tx_Data_type_e cmd_data_type; //命令码
	Tx_Gimbal_Data_s gimbal_data; //控制板发送的云台数据
} Vision_Send_s;

typedef struct
{
	struct
	{
		uint8_t sof;
		uint16_t data_length;
	} header;			   // 数据帧头
	// uint16_t cmd_id;	   // 数据ID
} protocol_rm_struct;

#pragma pack()

/*更新发送数据帧，并计算发送数据帧长度*/
void get_protocol_send_data(
							Vision_Send_s *tx_data,			 // 待发送的float数据
							uint8_t *tx_buf,		 // 待发送的数据帧
							uint16_t *tx_buf_len);	 // 待发送的数据帧长度

/*接收数据处理*/
uint16_t get_protocol_info(uint8_t *rx_buf,			 // 接收到的原始数据
						   Vision_Recv_s *rx_data);			 // 接收的float数据存储地址

#endif
