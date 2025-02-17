/**
*@file PWM_Init.c
*@author �����
*@brief PWM��ʼ��
*@date 2025/2/12 
*/

#include "stm32f10x.h" 

/*
*@brief PWM��ʼ�� ,Ĭ��TIM2,CH2ͨ�� 
*@param Angle ����Ƕȣ���ΧΪ0~180 
*/ 
void PWM_Init(uint16_t Angle)
{
	TIM_TimeBaseInitTypeDef TXInit;//��ʱ������ 
	TXInit.TIM_Prescaler=3600-1;  //��Ƶֵ 
	TXInit.TIM_CounterMode=TIM_CounterMode_Up;//ģʽ 
	TXInit.TIM_Period=400-1; //��װֵ 
	TXInit.TIM_ClockDivision=TIM_CKD_DIV1;//ȡ��ģʽ���ô�����    
	TXInit.TIM_RepetitionCounter=0;//�߼���ʱ��ר�� 
	TIM_TimeBaseInit(TIM2,&TXInit);
	
	TIM_OCInitTypeDef T2P;
	
	TIM_OCStructInit(&T2P);
	T2P.TIM_OCMode=TIM_OCMode_PWM1;
	T2P.TIM_OCPolarity=TIM_OCPolarity_High;
	T2P.TIM_OutputState=TIM_OutputState_Enable;
	T2P.TIM_Pulse=(Angle/90.0f+0.5f)*20;//�Ƕȿ��� 
	TIM_OC1Init(TIM2,&T2P);
	TIM_OC2Init(TIM2,&T2P);
	TIM_OC3Init(TIM2,&T2P);
	TIM_OC4Init(TIM2,&T2P);
	
	
	TIM_Cmd(TIM2,ENABLE);
}

