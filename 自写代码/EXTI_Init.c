/**
*@file EXTI_Init.c
*@author �����
*@brief EXTI�ⲿ�жϳ�ʼ��
*@date 2025/2/10 
*/

#include "stm32f10x.h"                  

/*
*@brief EXTI��ʼ�� 
*@param EXTI_TriggerX �ⲿ�ж������ء����ػ���˫�ش��� 

	EXTI_Trigger_Rising = 0x08,
  	EXTI_Trigger_Falling = 0x0C,  
  	EXTI_Trigger_Rising_Falling = 0x10

*@param EXTI_LineX ��Ҫ�ⲿ�жϵ����� 

	EXTI_Line(1~19) 

*@param EXTI_ModeX ģʽ 

 	EXTI_Mode_Interrupt = 0x00,
	EXTI_Mode_Event = 0x04
  
*/ 
void EXT_Init(uint32_t EXTI_LineX,EXTIMode_TypeDef EXTI_ModeX,EXTITrigger_TypeDef EXTI_TriggerX) 
{
	EXTI_InitTypeDef EInit; // EXTI�����ֶ�ʹ�ܣ�ֱ�����á���ʼ��EXTI�ṹ��
	EInit.EXTI_Line=EXTI_LineX;
	EInit.EXTI_LineCmd=ENABLE;
	EInit.EXTI_Mode=EXTI_ModeX;
	EInit.EXTI_Trigger=EXTI_TriggerX;
	EXTI_Init(&EInit);
}




