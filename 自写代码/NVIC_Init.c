/**
*@file NVIC_Init.c
*@author �����
*@brief NVIC��ʼ��
*@date 2025/2/10 
*/

#include "stm32f10x.h"  

/*
*@brief NVIC��ʼ�� 
*@param NVIC_IRQChannelX Ҫ�жϵĶ˿� ���ο� stm32F10x.h �� STM32F10X_MD �ļ� 
*@param NVIC_IRQChannelPreemptionPriorityX ��ռ���ȼ����ο� misc.h �ļ� 
*@param NVIC_IRQChannelSubPriorityX ��Ӧ���ȼ����ο� misc.h �ļ�
*@param NVIC_PriorityGroup �����ȼ����� 

 	NVIC_PriorityGroup_0~4

*/ 
void NV_Init(uint8_t NVIC_IRQChannelX,uint8_t NVIC_IRQChannelPreemptionPriorityX,uint8_t NVIC_IRQChannelSubPriorityX,uint32_t NVIC_PriorityGroup)        
  {
  	NVIC_PriorityGroupConfig(NVIC_PriorityGroup);//NVIC�Ѿ�ʹ�ܣ�ֱ�����á���NVIC���з���
	NVIC_InitTypeDef NInit;//����NVIC��ʼ���ṹ��
	NInit.NVIC_IRQChannel=NVIC_IRQChannelX;
	NInit.NVIC_IRQChannelCmd=ENABLE;
	NInit.NVIC_IRQChannelPreemptionPriority=NVIC_IRQChannelPreemptionPriorityX;
	NInit.NVIC_IRQChannelSubPriority=NVIC_IRQChannelSubPriorityX;
	NVIC_Init(&NInit);
  }
  
/*void EXTI15_10_IRQHandler(void)//10~15引脚中断函数
{
	if(EXTI_GetFlagStatus(EXTI_Line11)==SET)
	{

		EXTI_ClearFlag(EXTI_Line11);
	}
}*/

  
