/**
*@file GPIOX_Init.c
*@author ����� 
*@brief GPIO��ʼ�� 
*@date 2025/2/10 
*/

#include "stm32f10x.h" 
                 
/*
*@brief GPIO��ʼ�� 
*@param GPIOX �ӿ�X 

	GPIOA/GPIOB/GPIOC 

*@param RCC_APB2Periph APB2�Ľӿ� 

	RCC_APB2Periph_AFIO, RCC_APB2Periph_GPIOA, RCC_APB2Periph_GPIOB,
	RCC_APB2Periph_GPIOC, RCC_APB2Periph_GPIOD, RCC_APB2Periph_GPIOE,
	RCC_APB2Periph_GPIOF, RCC_APB2Periph_GPIOG, RCC_APB2Periph_ADC1,
	RCC_APB2Periph_ADC2, RCC_APB2Periph_TIM1, RCC_APB2Periph_SPI1,
	RCC_APB2Periph_TIM8, RCC_APB2Periph_USART1, RCC_APB2Periph_ADC3,
	RCC_APB2Periph_TIM15, RCC_APB2Periph_TIM16, RCC_APB2Periph_TIM17,
	RCC_APB2Periph_TIM9, RCC_APB2Periph_TIM10, RCC_APB2Periph_TIM11     

*@param GPIO_SpeedX �ٶ� 

	GPIO_Speed_10MHz = 1,
	GPIO_Speed_2MHz, 
	GPIO_Speed_50MHz

*@param GPIO_ModeX ģʽ 

	GPIO_Mode_AIN = 0x0,
	GPIO_Mode_IN_FLOATING = 0x04,
	GPIO_Mode_IPD = 0x28,
	GPIO_Mode_IPU = 0x48,
	GPIO_Mode_Out_OD = 0x14,
	GPIO_Mode_Out_PP = 0x10,
	GPIO_Mode_AF_OD = 0x1C,
	GPIO_Mode_AF_PP = 0x18

*@param GPIO_PinX ���� 

	GPIO_Pin1~15 

*/ 
void GPIOX_Init(GPIO_TypeDef* GPIOX,uint32_t RCC_APB2Periph,GPIOSpeed_TypeDef GPIO_SpeedX,GPIOMode_TypeDef GPIO_ModeX,uint16_t GPIO_PinX) 
{
	GPIO_InitTypeDef GPIOX_Init;//����GPIO��ʼ���ṹ�� 
	GPIOX_Init.GPIO_Pin=GPIO_PinX;
	GPIOX_Init.GPIO_Speed= GPIO_SpeedX;
	GPIOX_Init.GPIO_Mode=GPIO_ModeX;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph,ENABLE);//ʹ�� 
	GPIO_Init(GPIOX,&GPIOX_Init);
}

/**
  * @brief  Enables or disables the High Speed APB (APB2) peripheral clock.
  * @param  RCC_APB2Periph: specifies the APB2 peripheral to gates its clock.
  *   This parameter can be any combination of the following values:
  *     @arg RCC_APB2Periph_AFIO, RCC_APB2Periph_GPIOA, RCC_APB2Periph_GPIOB,
  *          RCC_APB2Periph_GPIOC, RCC_APB2Periph_GPIOD, RCC_APB2Periph_GPIOE,
  *          RCC_APB2Periph_GPIOF, RCC_APB2Periph_GPIOG, RCC_APB2Periph_ADC1,
  *          RCC_APB2Periph_ADC2, RCC_APB2Periph_TIM1, RCC_APB2Periph_SPI1,
  *          RCC_APB2Periph_TIM8, RCC_APB2Periph_USART1, RCC_APB2Periph_ADC3,
  *          RCC_APB2Periph_TIM15, RCC_APB2Periph_TIM16, RCC_APB2Periph_TIM17,
  *          RCC_APB2Periph_TIM9, RCC_APB2Periph_TIM10, RCC_APB2Periph_TIM11     
  * @param  NewState: new state of the specified peripheral clock.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None

	GPIO_Speed_10MHz = 1,
	GPIO_Speed_2MHz, 
	GPIO_Speed_50MHz

	GPIO_Mode_AIN = 0x0,
	GPIO_Mode_IN_FLOATING = 0x04,
	GPIO_Mode_IPD = 0x28,
	GPIO_Mode_IPU = 0x48,
	GPIO_Mode_Out_OD = 0x14,
	GPIO_Mode_Out_PP = 0x10,
	GPIO_Mode_AF_OD = 0x1C,
	GPIO_Mode_AF_PP = 0x18
	
	GPIO_Pin_X
  */


