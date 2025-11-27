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
  * @内  容  系统配置相关
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AX_SYS_H
#define __AX_SYS_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

///************** 位带操作，实现类51GPIO控制*********************************/

////IO口操作宏定义
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2))
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr))
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum))

//IO口地址映射
#define GPIOA_ODR_Addr    (GPIOA_BASE+20) //0x40020014
#define GPIOA_IDR_Addr    (GPIOA_BASE+16) //0x40020010

//IO口操作
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //输出
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //输入

//X-SOFT 接口函数
void AX_NC_Set(void);  //占位函数

#endif

/******************* (C) 版权 2022 XTARK **************************************/