/**
*@file NVIC_Init.c
*@author 申飞麟
*@brief NVIC初始化
*@date 2025/2/10 
*/

#include "stm32f10x.h"  

/*
*@brief NVIC初始化 
*@param NVIC_IRQChannelX 要中断的端口 ，参考 stm32F10x.h 中 STM32F10X_MD 文件 
*@param NVIC_IRQChannelPreemptionPriorityX 抢占优先级，参考 misc.h 文件 
*@param NVIC_IRQChannelSubPriorityX 响应优先级，参考 misc.h 文件
*@param NVIC_PriorityGroup 对优先级分组 

 	NVIC_PriorityGroup_0~4

*/ 
void NV_Init(uint8_t NVIC_IRQChannelX,uint8_t NVIC_IRQChannelPreemptionPriorityX,uint8_t NVIC_IRQChannelSubPriorityX,uint32_t NVIC_PriorityGroup)        
  {
  	NVIC_PriorityGroupConfig(NVIC_PriorityGroup);//NVIC已经使能，直接配置。对NVIC进行分组
	NVIC_InitTypeDef NInit;//构建NVIC初始化结构体
	NInit.NVIC_IRQChannel=NVIC_IRQChannelX;
	NInit.NVIC_IRQChannelCmd=ENABLE;
	NInit.NVIC_IRQChannelPreemptionPriority=NVIC_IRQChannelPreemptionPriorityX;
	NInit.NVIC_IRQChannelSubPriority=NVIC_IRQChannelSubPriorityX;
	NVIC_Init(&NInit);
  }
  
/*void EXTI15_10_IRQHandler(void)//10~15寮曡剼涓柇鍑芥暟
{
	if(EXTI_GetFlagStatus(EXTI_Line11)==SET)
	{

		EXTI_ClearFlag(EXTI_Line11);
	}
}*/

  
