/**
*@file GPIO_Init.h
*@author �����
*@brief GPIO��ʼ��ͷ�ļ� 
*@date 2025/2/10 
*/

#ifndef GPIO_INIT_H
#define GPIO_INIT_H
 
void GPIOX_Init(GPIO_TypeDef* GPIOX,uint32_t RCC_APB2Periph,GPIOSpeed_TypeDef GPIO_SpeedX,GPIOMode_TypeDef GPIO_ModeX,uint16_t GPIO_PinX); 

#endif

