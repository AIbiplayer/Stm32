/**
*@file TIMX_Init.c
*@author 申飞麟
*@brief 初级时钟初始化
*@date 2025/2/11
*/

#include "stm32f10x.h"                  

/*
*@brief 初级时钟初始化 
*@param TIM_PrescalerX 分频器值，0~65536 
*@param TIM_CounterModeX 时钟模式 

	TIM_CounterMode_Up                 
	IM_CounterMode_Down               
	TIM_CounterMode_CenterAligned1     
	TIM_CounterMode_CenterAligned2     
	TIM_CounterMode_CenterAligned3     

*@param TIM_PeriodX 取样次数，0~65536 
*@param RCC_APB1PeriphX APB1对应端口 
  
	RCC_APB1Periph_TIM2              ((uint32_t)0x00000001)
	RCC_APB1Periph_TIM3              ((uint32_t)0x00000002)
	RCC_APB1Periph_TIM4              ((uint32_t)0x00000004)
	RCC_APB1Periph_TIM5              ((uint32_t)0x00000008)
	RCC_APB1Periph_TIM6              ((uint32_t)0x00000010)
	RCC_APB1Periph_TIM7              ((uint32_t)0x00000020)
	RCC_APB1Periph_TIM12             ((uint32_t)0x00000040)
	RCC_APB1Periph_TIM13             ((uint32_t)0x00000080)
	RCC_APB1Periph_TIM14             ((uint32_t)0x00000100)
	
*/ 
void TIMX_Init(uint32_t RCC_APB1PeriphX,TIM_TypeDef* TIMX,uint16_t TIM_PrescalerX,uint16_t TIM_CounterModeX,uint16_t TIM_PeriodX) 
{
	RCC_APB1PeriphClockCmd(RCC_APB1PeriphX,ENABLE);//配置TIMX的APB1时钟
	
	TIM_TimeBaseInitTypeDef TXInit;
	TXInit.TIM_Prescaler=TIM_PrescalerX;  //分频器值-1，溢出值 
	TXInit.TIM_CounterMode=TIM_CounterModeX;//模式  
	TXInit.TIM_Period=TIM_PeriodX; //取样次数-1， 溢出次数的最大值 
	TXInit.TIM_ClockDivision=TIM_CKD_DIV2;//滤波取样频率，几乎不影响    
	TXInit.TIM_RepetitionCounter=0;//高级计数器专用
	TIM_TimeBaseInit(TIMX,&TXInit);
	
	TIM_ITConfig(TIMX,TIM_IT_Update,ENABLE);//TIMX中断使能
	
	TIM_Cmd(TIMX,ENABLE);//TIMX使能
}



/*void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2,TIM_IT_Update)==SET)
	{
		
		TIM_ClearFlag(TIM2,TIM_FLAG_Update);
}

	GPIOX_Init(GPIOA,RCC_APB2Periph_GPIOA,GPIO_Speed_10MHz,GPIO_Mode_IPU,GPIO_Pin_0);//TIM2默认PA0口 

	TIM_ETRClockMode2Config(TIM2, TIM_ExtTRGPSC_OFF,TIM_ExtTRGPolarity_Inverted, 0x0F);//TIM设置为外部中断 



}*/
