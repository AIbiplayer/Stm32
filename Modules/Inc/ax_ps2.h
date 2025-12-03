/**
____                    _____ _______ _____       XTARK@塔克创新
		  / __ \                  / ____|__   __|  __ \
		 | |  | |_ __   ___ _ __ | |       | |  | |__) |
		 | |  | | '_ \ / _ \ '_ \| |       | |  |  _  /
		 | |__| | |_) |  __/ | | | |____   | |  | | \ \
		  \____/| .__/ \___|_| |_|\_____|  |_|  |_|  \_\
				| |
				|_|                OpenCTR   机器人控制器

  ******************************************************************************
  *
  * 版权所有： XTARK@塔克创新  版权所有，盗版必究
  * 公司网站： www.xtark.cn   www.tarkbot.com
  * 淘宝店铺： https://xtark.taobao.com
  * 塔克微信： 塔克创新（关注公众号，获取最新更新资讯）
  *
  ******************************************************************************
  * @作  者  Musk Han@XTARK
  * @内  容  PS2无线手柄函数文件
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AX_PS2_H
#define __AX_PS2_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

#define DI()     PAin(4)      //数据输入引脚
#define CMD_H()   PAout(5)=1   //命令位高
#define CMD_L()   PAout(5)=0   //命令位低
#define CS_H()   PAout(6)=1    //CS拉高(别名ATT)
#define CS_L()   PAout(6)=0    //CS拉低(别名ATT)
#define CLK_H()  PAout(7)=1    //时钟拉高
#define CLK_L()  PAout(7)=0    //时钟拉低

typedef struct
{
    uint8_t select : 1; /* B0:SLCT B3:STRT B4:UP B5:R B6:DOWN B7:L   */
    uint8_t select_last : 1;
    uint8_t select_mode : 1;
    uint8_t select_mode_last : 1;
    uint8_t button_L : 1;
    uint8_t button_R : 1;
    uint8_t button_R_Last : 1;
    uint8_t Vision_Mode : 2;
    uint8_t start : 1;
    uint8_t up : 1;
    uint8_t down : 1;
    uint8_t left : 1;
    uint8_t right : 1;
    uint8_t L1 : 1; /* B0:L2 B1:R2 B2:L1 B3:R1 B4:/\ B5:O B6:X B7:口 */
    uint8_t L2 : 1;
    uint8_t R1 : 1;
    uint8_t R2 : 1;
    uint8_t Triangle : 1;
    uint8_t Triangle_Last : 1;
    uint8_t Control_Mode : 1;
    uint8_t Circle : 1;
    uint8_t Cross : 1;
    uint8_t Square : 1;
    int16_t RJoy_LR; /* 右边摇杆  0x00 = 左    0xff = 右   */
    int16_t RJoy_UD; /* 右边摇杆  0x00 = 上    0xff = 下   */
    int16_t LJoy_LR; /* 左边摇杆  0x00 = 左    0xff = 右   */
    int16_t LJoy_UD; /* 左边摇杆  0x00 = 上    0xff = 下   */
} JOYSTICK_TypeDef;

/*** PS2无线手柄操作函数 **********/
void AX_PS2_ScanKey(void); //PS2获取按键及摇杆数值

#endif

/******************* (C) 版权 2023 XTARK **************************************/
