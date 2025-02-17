/**
*@file AFIO_Init.c
*@author 申飞麟
*@brief AFIO使能并初始化
*@date 2025/2/10 
*/

#include "stm32f10x.h" 

/*
*@brief AFIO初始化 
*@param GPIO_PortSource 选择端口 

	GPIO_PortSourceGPIOA~G

*@param GPIO_PinSource 选择引脚 

	GPIO_PinSource1~15

*/ 
void AFIO_Init(uint8_t GPIO_PortSource, uint8_t GPIO_PinSource) 
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);//AFIO使能，用于外部中断
	GPIO_EXTILineConfig(GPIO_PortSource,GPIO_PinSource);//将X引脚配置为外部中断
}

