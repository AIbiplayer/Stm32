/**
*@file PWM_Init.c
*@author 申飞麟
*@brief PWM初始化
*@date 2025/2/12 
*/

#include "stm32f10x.h" 

/*
*@brief PWM初始化 ,默认TIM2,CH2通道 
*@param Angle 舵机角度，范围为0~180 
*/ 
void PWM_Init(uint16_t Angle)
{
	TIM_TimeBaseInitTypeDef TXInit;//定时器数组 
	TXInit.TIM_Prescaler=3600-1;  //分频值 
	TXInit.TIM_CounterMode=TIM_CounterMode_Up;//模式 
	TXInit.TIM_Period=400-1; //重装值 
	TXInit.TIM_ClockDivision=TIM_CKD_DIV1;//取样模式，用处不大    
	TXInit.TIM_RepetitionCounter=0;//高级定时器专用 
	TIM_TimeBaseInit(TIM2,&TXInit);
	
	TIM_OCInitTypeDef T2P;
	
	TIM_OCStructInit(&T2P);
	T2P.TIM_OCMode=TIM_OCMode_PWM1;
	T2P.TIM_OCPolarity=TIM_OCPolarity_High;
	T2P.TIM_OutputState=TIM_OutputState_Enable;
	T2P.TIM_Pulse=(Angle/90.0f+0.5f)*20;//角度控制 
	TIM_OC1Init(TIM2,&T2P);
	TIM_OC2Init(TIM2,&T2P);
	TIM_OC3Init(TIM2,&T2P);
	TIM_OC4Init(TIM2,&T2P);
	
	
	TIM_Cmd(TIM2,ENABLE);
}

