/**
*@file EXTI_Init.h
*@author 申飞麟
*@brief EXTI外部中断初始化头文件 
*@date 2025/2/10 
*/

#ifndef EXTI_INIT_H
#define EXTI_INIT_H
 
void EXT_Init(uint32_t EXTI_LineX,EXTIMode_TypeDef EXTI_ModeX,EXTITrigger_TypeDef EXTI_TriggerX);

#endif

