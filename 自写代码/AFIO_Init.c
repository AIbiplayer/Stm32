/**
*@file AFIO_Init.c
*@author �����
*@brief AFIOʹ�ܲ���ʼ��
*@date 2025/2/10 
*/

#include "stm32f10x.h" 

/*
*@brief AFIO��ʼ�� 
*@param GPIO_PortSource ѡ��˿� 

	GPIO_PortSourceGPIOA~G

*@param GPIO_PinSource ѡ������ 

	GPIO_PinSource1~15

*/ 
void AFIO_Init(uint8_t GPIO_PortSource, uint8_t GPIO_PinSource) 
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);//AFIOʹ�ܣ������ⲿ�ж�
	GPIO_EXTILineConfig(GPIO_PortSource,GPIO_PinSource);//��X��������Ϊ�ⲿ�ж�
}

