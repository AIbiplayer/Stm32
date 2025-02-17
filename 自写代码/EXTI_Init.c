/**
*@file EXTI_Init.c
*@author 申飞麟
*@brief EXTI外部中断初始化
*@date 2025/2/10 
*/

#include "stm32f10x.h"                  

/*
*@brief EXTI初始化 
*@param EXTI_TriggerX 外部中断是上沿、下沿还是双沿触发 

	EXTI_Trigger_Rising = 0x08,
  	EXTI_Trigger_Falling = 0x0C,  
  	EXTI_Trigger_Rising_Falling = 0x10

*@param EXTI_LineX 需要外部中断的引脚 

	EXTI_Line(1~19) 

*@param EXTI_ModeX 模式 

 	EXTI_Mode_Interrupt = 0x00,
	EXTI_Mode_Event = 0x04
  
*/ 
void EXT_Init(uint32_t EXTI_LineX,EXTIMode_TypeDef EXTI_ModeX,EXTITrigger_TypeDef EXTI_TriggerX) 
{
	EXTI_InitTypeDef EInit; // EXTI无需手动使能，直接配置。初始化EXTI结构体
	EInit.EXTI_Line=EXTI_LineX;
	EInit.EXTI_LineCmd=ENABLE;
	EInit.EXTI_Mode=EXTI_ModeX;
	EInit.EXTI_Trigger=EXTI_TriggerX;
	EXTI_Init(&EInit);
}




