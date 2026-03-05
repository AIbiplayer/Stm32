#include <stdbool.h>
#include <stdint.h>
#include "video_link.h"
#include "bsp_usart.h"
#include "main.h"
#include "stdlib.h"
#include "string.h"

#define VL_RC_RECV_SIZE 21 // 遥控器数据接收长度,根据协议设定

static VL_ctrl_t vl_rc_ctrl[2]; //[0]:当前数据TEMP,[1]:上一次的数据LAST.用于按键持续按下和切换的判断
static uint8_t vl_rc_init_flag = 0; // 遥控器数据初始化标志位,0为未初始化,1为已初始化
static vl_remote_data_t vl_rc_data; // 遥控器数据

static USARTInstance *vl_rc_usart_instance;

static uint16_t get_crc16_check_sum(uint8_t *p_msg, uint16_t len, uint16_t crc16);

static uint16_t crc16_init = 0xffff;
static const uint16_t crc16_tab[256] =
{
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
    0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
    0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
    0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
	0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
	0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
	0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
	0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
	0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
	0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
	0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
	0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
	0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
	0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
	0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
	0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
	0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
	0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
	0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
	0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
	0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
	0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
	0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
	0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
	0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
	0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
	0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
	0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
	0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
	0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
	0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

/**
 * @brief Get the crc16 checksum
 *
 * @param p_msg Data to check
 * @param len Datalength
 * @param crc16 Crc16 initialized checksum
 * @return crc16 Crc16 checksum
 */
static uint16_t get_crc16_check_sum(uint8_t *p_msg, uint16_t len, uint16_t crc16)
{
    uint8_t data;
    if(p_msg == NULL)
    {
        return 0xffff;
    }
    while(len--)
    {
        data = *p_msg++;
        (crc16) = ((uint16_t)(crc16) >> 8) ^ crc16_tab[((uint16_t)(crc16) ^ (uint16_t)(data)) & 0x00ff];
    }
    return crc16;
}

/**
 * @brief crc16 verify function
 * @attention 这个函数会自己用数据的最后俩个字节，对比不包含最后两个字节校验位的整包的crc16校验结果，因此在调用此函数时，数据包的最后两个字节必须是crc16校验位
 * @param p_msg Data to verify
 * @param len Stream length=data+checksum
 * @return bool Crc16 check result
 */
bool verify_crc16_check_sum(uint8_t *p_msg, uint16_t len)
{
    uint16_t w_expected = 0;

    if((p_msg == NULL) || (len <= 2))
    {
        return false;
    }
    w_expected = get_crc16_check_sum(p_msg, len - 2, crc16_init);

    return ((w_expected & 0xff) == p_msg[len - 2] && ((w_expected >> 8) & 0xff) == p_msg[len - 1]);
}

/**
 * @brief 矫正遥控器摇杆的值,超过660或者小于-660的值都认为是无效值,置0
 *
 */
static void RectifyRCjoystick()
{
	for (uint8_t i = 0; i < 5; ++i)
		if (abs(*(&vl_rc_ctrl[TEMP].rc.rocker_l_ + i)) > 660)
			*(&vl_rc_ctrl[TEMP].rc.rocker_l_ + i) = 0;
}

/**
 * @brief 更新按键状态机
 * @note  根据当前原始按键值和上一次的状态机状态来更新
 */
static void UpdateKeyStateMachine()
{
	// 获取当前所有按键的位掩码 (16位)
	uint16_t keys_now_bits = vl_rc_ctrl[TEMP].key[KEY_PRESS].keys;

	for (int i = 0; i < 16; i++)
	{
		// 1. 获取当前该位是否按下 (bool)
		uint8_t is_down_now = (keys_now_bits >> i) & 0x01;

		// 2. 获取指针对(方便操作)
		// 注意：逻辑基于上一次的状态(LAST)来计算当前状态(TEMP)
		KeyStateMachine_t *fsm_last = &vl_rc_ctrl[LAST].key_fsm[i];
		KeyStateMachine_t *fsm_curr = &vl_rc_ctrl[TEMP].key_fsm[i];

		// 3. 状态机流转逻辑
		if (is_down_now)
		{
			// --- 如果当前物理按键是按下的 ---

			if (fsm_last->state == KEY_RELEASED || fsm_last->state == KEY_PRESS_UP)
			{
				// 上一次是松开，现在是按下 -> 触发【按下瞬间】
				fsm_curr->state = KEY_PRESS_DOWN;
				fsm_curr->hold_tick = 0; // 重置计数
			}
			else
			{
				// 上一次是按下，现在还是按下 -> 持续按下
				fsm_curr->hold_tick = fsm_last->hold_tick + 1; // 累加计数

				if (fsm_curr->hold_tick > LONG_PRESS_TICK_THRESH)
				{
					// 超过阈值 -> 判定为【长按】
					fsm_curr->state = KEY_LONG_PRESS;
				}
				else
				{
					// 还没超过阈值 -> 判定为【持续按下】
					fsm_curr->state = KEY_PRESSING;
				}
			}
		}
		else
		{
			// --- 如果当前物理按键是松开的 ---

			if (fsm_last->state != KEY_RELEASED && fsm_last->state != KEY_PRESS_UP)
			{
				// 上一次是各种按下状态，现在松开了 -> 触发【松开瞬间】
				fsm_curr->state = KEY_PRESS_UP;
				fsm_curr->hold_tick = fsm_last->hold_tick; // 保持最后一次的计数供查询
			}
			else
			{
				// 一直没按 -> 保持【松开状态】
				fsm_curr->state = KEY_RELEASED;
				fsm_curr->hold_tick = 0;
			}
		}
	}
}

/**
 * @brief 遥控器数据解析函数,在接收回调函数中被调用
 *
 * @param buffer 接收到的数据buffer
 */
static void DecodeVLRC(const uint8_t *buffer)
{
	if(buffer[0] != 0xA9 || buffer[1] != 0x53) // 帧头校验
		return;
	if (verify_crc16_check_sum(buffer, 21) == false) // CRC校验
		return;
	memcpy(&vl_rc_data, buffer, sizeof(vl_remote_data_t)); // 数据复制
	vl_rc_ctrl[TEMP].rc.rocker_r_ = vl_rc_data.ch_0 - RC_CH_VALUE_OFFSET;
	vl_rc_ctrl[TEMP].rc.rocker_r1 = vl_rc_data.ch_1 - RC_CH_VALUE_OFFSET;
	vl_rc_ctrl[TEMP].rc.rocker_l1 = vl_rc_data.ch_2 - RC_CH_VALUE_OFFSET;
	vl_rc_ctrl[TEMP].rc.rocker_l_ = vl_rc_data.ch_3 - RC_CH_VALUE_OFFSET;
	vl_rc_ctrl[TEMP].rc.wheel = vl_rc_data.wheel - RC_CH_VALUE_OFFSET;
	RectifyRCjoystick();

	vl_rc_ctrl[TEMP].rc.mode_sw = vl_rc_data.mode_sw;
	vl_rc_ctrl[TEMP].rc.pause = vl_rc_data.pause;
	vl_rc_ctrl[TEMP].rc.fn_1 = vl_rc_data.fn_1;
	vl_rc_ctrl[TEMP].rc.fn_2 = vl_rc_data.fn_2;

	vl_rc_ctrl[TEMP].rc.trigger = vl_rc_data.trigger;

	vl_rc_ctrl[TEMP].mouse.x = vl_rc_data.mouse_x;
	vl_rc_ctrl[TEMP].mouse.y = vl_rc_data.mouse_y;
	vl_rc_ctrl[TEMP].mouse.z = vl_rc_data.mouse_z;
	vl_rc_ctrl[TEMP].mouse.mouse_left = vl_rc_data.mouse_left;
	vl_rc_ctrl[TEMP].mouse.mouse_right = vl_rc_data.mouse_right;
	vl_rc_ctrl[TEMP].mouse.mouse_middle = vl_rc_data.mouse_middle;

	vl_rc_ctrl[TEMP].key[KEY_PRESS] = vl_rc_data.key;
	if (vl_rc_ctrl[TEMP].key[KEY_PRESS].ctrl) // ctrl键按下
		vl_rc_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL] = vl_rc_ctrl[TEMP].key[KEY_PRESS];
	else
		memset(&vl_rc_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL], 0, sizeof(Video_Key_u));
	if (vl_rc_ctrl[TEMP].key[KEY_PRESS].shift) // shift键按下
		vl_rc_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT] = vl_rc_ctrl[TEMP].key[KEY_PRESS];
	else
		memset(&vl_rc_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT], 0, sizeof(Video_Key_u));

	uint16_t key_now = vl_rc_ctrl[TEMP].key[KEY_PRESS].keys,                   // 当前按键是否按下
	key_last = vl_rc_ctrl[LAST].key[KEY_PRESS].keys,                       // 上一次按键是否按下
	key_with_ctrl = vl_rc_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL].keys,        // 当前ctrl组合键是否按下
	key_with_shift = vl_rc_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT].keys,      //  当前shift组合键是否按下
	key_last_with_ctrl = vl_rc_ctrl[LAST].key[KEY_PRESS_WITH_CTRL].keys,   // 上一次ctrl组合键是否按下
	key_last_with_shift = vl_rc_ctrl[LAST].key[KEY_PRESS_WITH_SHIFT].keys; // 上一次shift组合键是否按下

	for (uint16_t i = 0, j = 0x1; i < 16; j <<= 1, i++)
	{
		if (i == 4 || i == 5) // 4,5位为ctrl和shift,直接跳过
			continue;
		// 如果当前按键按下,上一次按键没有按下,且ctrl和shift组合键没有按下,则按键按下计数加1(检测到上升沿)
		if ((key_now & j) && !(key_last & j) && !(key_with_ctrl & j) && !(key_with_shift & j))
			vl_rc_ctrl[TEMP].key_count[KEY_PRESS][i]++;
		// 当前ctrl组合键按下,上一次ctrl组合键没有按下,则ctrl组合键按下计数加1(检测到上升沿)
		if ((key_with_ctrl & j) && !(key_last_with_ctrl & j))
			vl_rc_ctrl[TEMP].key_count[KEY_PRESS_WITH_CTRL][i]++;
		// 当前shift组合键按下,上一次shift组合键没有按下,则shift组合键按下计数加1(检测到上升沿)
		if ((key_with_shift & j) && !(key_last_with_shift & j))
			vl_rc_ctrl[TEMP].key_count[KEY_PRESS_WITH_SHIFT][i]++;
	}

	vl_rc_ctrl[TEMP].rc.rocker_l_ = -vl_rc_ctrl[TEMP].rc.rocker_l_; // 左摇杆反向,确保向左接收到的数据为正，符合右手系
	vl_rc_ctrl[TEMP].rc.rocker_r_ = -vl_rc_ctrl[TEMP].rc.rocker_r_; // 右摇杆反向,确保向右接收到的数据为正，符合右手系

	//在保存到 LAST 之前，运行状态机
	UpdateKeyStateMachine();

	memcpy(&vl_rc_ctrl[LAST], &vl_rc_ctrl[TEMP], sizeof(VL_ctrl_t)); // 保存上一次的数据,用于按键持续按下和切换的判断

}

static void VLRemoteControlRxCallback()
{
	DecodeVLRC(vl_rc_usart_instance->recv_buff); // 进行协议解析
}

/**
 * @brief 遥控器离线的回调函数,注册到守护进程中,串口掉线时调用
 */
static void VLRCLostCallback()
{
	memset(vl_rc_ctrl, 0, sizeof(vl_rc_ctrl)); // 清空遥控器数据
	USARTServiceInit(vl_rc_usart_instance); // 尝试重新启动接收
}

/**
 * @brief 初始化图传遥控器,该函数会将遥控器注册到串口
 * @param vl_rc_usart_handle
 * @return
 * @attention 注意分配正确的串口硬件,图传遥控器的波特率为921600,数据位8,停止位1,无校验
 */
VL_ctrl_t *VLRemoteControlInit(UART_HandleTypeDef *vl_rc_usart_handle)
{
	USART_Init_Config_s conf;
	conf.module_callback = VLRemoteControlRxCallback;
	conf.usart_handle = vl_rc_usart_handle;
	conf.recv_buff_size = VL_RC_RECV_SIZE;
	vl_rc_usart_instance = USARTRegister(&conf);

	vl_rc_init_flag = 1;
	return vl_rc_ctrl;
}
