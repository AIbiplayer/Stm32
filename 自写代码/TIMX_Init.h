/**
*@file TIMX_Init.h
*@author 申飞麟
*@brief 初级时钟初始化头文件 
*@date 2025/2/11
*/

#ifndef TIMX_INIT_H
#define TIMX_INIT_H

void TIMX_Init(uint32_t RCC_APB1PeriphX,TIM_TypeDef* TIMX,uint16_t TIM_PrescalerX,uint16_t TIM_CounterModeX,uint16_t TIM_PeriodX); 

#endif

