/**
*@file NVIC_Init.h
*@author �����
*@brief NVIC��ʼ��
*@date 2025/2/10 
*/

#ifndef NVIC_INIT_H
#define NVIC_INIT_H
 
void NV_Init(uint8_t NVIC_IRQChannelX,uint8_t NVIC_IRQChannelPreemptionPriorityX,uint8_t NVIC_IRQChannelSubPriorityX,uint32_t NVIC_PriorityGroup);      
/*void EXTI15_10_IRQHandler(void); //中断函数，默认11引脚*/

#endif
