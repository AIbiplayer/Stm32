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
  * @版  本  V1.0
  * @日  期  2022-7-26
  * @内  容  PS2无线手柄函数文件
  *
  ******************************************************************************
  * @说  明
  *
  *   PS2数据定义
  *   BYTE   DATA   解释
  *   01     idle
  *   02     0x73   手柄工作模式
  *   03     0x5A   Bit0  Bit1  Bit2  Bit3  Bit4  Bit5  Bit6  Bit7
  *   04     data   SLCT  JOYR  JOYL  STRT   UP   RGIHT  DOWN   L
  *   05     data   L2     R2     L1    R1   /\     O     X    口
  *   06     data   右边摇杆  0x00 = 左    0xff = 右
  *   07     data   右边摇杆  0x00 = 上    0xff = 下
  *   08     data   左边摇杆  0x00 = 左    0xff = 右
  *   09     data   左边摇杆  0x00 = 上    0xff = 下
  *
  ******************************************************************************
  */

#include "ax_ps2.h"
#include "ax_delay.h"
#include "ax_sys.h"
#include "Debug_Tool.h"

JOYSTICK_TypeDef JoystickStruct = {0};

const uint8_t PS2_cmnd[9] = {0x01, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; //请求获取数据命令
static uint8_t PS2_data[9] = {0}; //接收的数据

/**
  * @简  述  PS2数据读写函数
  * @参  数  data:要写入的数据
  * @返回值  读出数据
  */
static uint8_t PS2_ReadWriteData(uint8_t data)
{
    uint8_t ref, res = 0;
    for (ref = 0x01; ref > 0x00; ref <<= 1)
    {
        CLK_L();
        if (ref & data)
            CMD_H();
        else
            CMD_L();
        AX_Delayus(16);
        CLK_H();
        if (DI())
            res |= ref;
        AX_Delayus(16);
    }
    CMD_H();
    //返回读出数据
    return res;
}

/**
  * @简  述  PS2获取按键及摇杆数值。
  * @参  数  *JoystickStruct 手柄键值结构体
  * @返回值  无
  */
void AX_PS2_ScanKey(void)
{
    uint8_t i;
    //使能手柄
    CS_L();
    //读取PS2数据
    for (i = 0; i < 9; i++)
    {
        PS2_data[i] = PS2_ReadWriteData(PS2_cmnd[i]);
        AX_Delayus(16);
    }
    //关闭使能
    CS_H();
    //数值传递
    JoystickStruct.select_last = JoystickStruct.select;
    JoystickStruct.select_mode_last = JoystickStruct.select_mode;
    JoystickStruct.select = ~PS2_data[3] >> 0;
    JoystickStruct.select_mode = JoystickStruct.select && !JoystickStruct.select_last
                                     ? !JoystickStruct.select_mode
                                     : JoystickStruct.select_mode;

    JoystickStruct.button_R_Last = JoystickStruct.button_R;
    JoystickStruct.button_R = ~PS2_data[3] >> 2;
    JoystickStruct.Vision_Mode = JoystickStruct.button_R && !JoystickStruct.button_R_Last
                                     ? (JoystickStruct.Vision_Mode + 1) % 4
                                     : JoystickStruct.Vision_Mode;

    JoystickStruct.Triangle_Last = JoystickStruct.Triangle;
    JoystickStruct.Triangle = ~PS2_data[4] >> 4;
    JoystickStruct.Control_Mode = JoystickStruct.Triangle && !JoystickStruct.Triangle_Last
                                      ? (JoystickStruct.Control_Mode + 1) % 4
                                      : JoystickStruct.Control_Mode;

    JoystickStruct.button_L = ~PS2_data[3] >> 1;
    JoystickStruct.start = ~PS2_data[3] >> 3;
    JoystickStruct.up = ~PS2_data[3] >> 4;
    JoystickStruct.down = ~PS2_data[3] >> 6;
    JoystickStruct.left = ~PS2_data[3] >> 7;
    JoystickStruct.right = ~PS2_data[3] >> 5;
    JoystickStruct.L1 = ~PS2_data[4] >> 2;
    JoystickStruct.L2 = ~PS2_data[4] >> 0;
    JoystickStruct.R1 = ~PS2_data[4] >> 3;
    JoystickStruct.R2 = ~PS2_data[4] >> 1;
    JoystickStruct.Circle = ~PS2_data[4] >> 5;
    JoystickStruct.Cross = ~PS2_data[4] >> 6;
    JoystickStruct.Square = ~PS2_data[4] >> 7;
    JoystickStruct.RJoy_LR = -(int16_t)(PS2_data[5] - 128);
    JoystickStruct.RJoy_UD = -(int16_t)(PS2_data[6] - 127);
    JoystickStruct.LJoy_LR = -(int16_t)(PS2_data[7] - 128);
    JoystickStruct.LJoy_UD = -(int16_t)(PS2_data[8] - 127);
}

/******************* (C) 版权 2022 XTARK **************************************/
