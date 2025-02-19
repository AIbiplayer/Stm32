#include "stm32f10x.h"                 
#include "Delay.h"
#include "OLED.h"
#include "GPIOX_Init.h"
#include "PWM_Init.h"
#include "DogEmotions.h"
#include "OLED_Data.h"
#include "NVIC_Init.h"
#include <string.h>
#include "DogActive.h"
#include "AFIO_Init.h"
#include "EXTI_Init.h"

static char RXInformation[20];//用于接收数据

int main(void)
{
	OLED_Init();
	//使用USART1
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	//触摸引脚PB12初始化
	GPIOX_Init(GPIOB,RCC_APB2Periph_GPIOB,GPIO_Speed_50MHz,GPIO_Mode_IPU,GPIO_Pin_12);
	//TX写引脚PA9初始化
	GPIOX_Init(GPIOA,RCC_APB2Periph_GPIOA,GPIO_Speed_50MHz,GPIO_Mode_AF_PP,GPIO_Pin_9);
	//RX读引脚PA10初始化
	GPIOX_Init(GPIOA,RCC_APB2Periph_GPIOA,GPIO_Speed_50MHz,GPIO_Mode_IPU,GPIO_Pin_10);
	//舵机引脚初始化
	GPIOX_Init(GPIOA,RCC_APB2Periph_GPIOA,GPIO_Speed_50MHz,GPIO_Mode_AF_PP,GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3);
	//OLED引脚初始化
	GPIOX_Init(GPIOB,RCC_APB2Periph_GPIOB,GPIO_Speed_50MHz,GPIO_Mode_Out_PP,GPIO_Pin_8|GPIO_Pin_9);
	//舵机初始化
	PWM_Init(0);
	
	USART_InitTypeDef USART_BlueTooth;//蓝牙连接初始化
	USART_BlueTooth.USART_BaudRate=9600;//波特率9600
	USART_BlueTooth.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	USART_BlueTooth.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;//发送/接收模式
	USART_BlueTooth.USART_Parity=USART_Parity_No;//无校验
	USART_BlueTooth.USART_StopBits=USART_StopBits_1;//1位停止位
	USART_BlueTooth.USART_WordLength=USART_WordLength_8b;//8位数据
	USART_Init(USART1,&USART_BlueTooth);
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);
	
	NV_Init(USART1_IRQn,1,1,NVIC_PriorityGroup_2);
	AFIO_Init(GPIO_PortSourceGPIOB,GPIO_PinSource12);//触发外部中断
	NV_Init(EXTI15_10_IRQn,2,2,NVIC_PriorityGroup_2);
	EXT_Init(EXTI_Line12,EXTI_Mode_Interrupt,EXTI_Trigger_Rising);//上升沿触发
	
	USART_Cmd(USART1,ENABLE);
	
	while(1)
	{

		if(!strcmp(RXInformation,"Back"))
		{
			Run_Emotion();
			BackRun();
		}
		else if(!strcmp(RXInformation,"Run"))
		{
			Run_Emotion();
			Run();
		}
		else if(!strcmp(RXInformation,"RightRun"))
		{
			RightRun_Emotion();
			RightRun();
		}
		else if(!strcmp(RXInformation,"LeftRun"))
		{
			LeftRun_Emotion();
			LeftRun();
		}
		else if(!strcmp(RXInformation,"SitDown"))
		{
			Blink_Emotion();
			SitDown();
		}
		else if(!strcmp(RXInformation,"Dance"))
		{
			Dance_Emotion();
			Dance();
		}
		else if(!strcmp(RXInformation,"Sleep"))
		{
			Lay_Emotion();
			Laydown();
		}
		else if(!strcmp(RXInformation,"Wave"))
		{
			SitDown();
			Wave_Emotion();
			Wave();
		}
		else if(!strcmp(RXInformation,"Pet"))
		{
			SitDown();
			Pet_Emotion();
		}
		else
		{
			StandUp();
			Blink_Emotion();
			Delays(1);
		}
	}
}

void USART1_IRQHandler(void)//USART引脚中断函数
{
	static uint8_t Sum=0;
	if(USART_GetFlagStatus(USART1,USART_FLAG_RXNE)==SET)
	{
		char Data=USART_ReceiveData(USART1);//把数据存入Data中，避免出错
		RXInformation[Sum]=Data;
		if (Data == '\n' || Data == '\r' || Sum >= sizeof(RXInformation) - 1)
        {
            RXInformation[Sum] = '\0'; // 添加字符串结束符
            Sum = 0; // 重置计数器
        }
		else if(Data == '@')
		{
			strcpy(RXInformation, ""); // 清空字符串
		}
        else
        {
            RXInformation[Sum] = Data; // 存储数据
            Sum++;
        }
		USART_ClearFlag(USART1,USART_FLAG_RXNE);
	}
	USART_ClearITPendingBit(USART1,USART_FLAG_RXNE);
}

void EXTI15_10_IRQHandler(void)//10~15引脚中断函数
{
	if(EXTI_GetFlagStatus(EXTI_Line12)==SET)
	{
		strcpy(RXInformation,"Pet");
		EXTI_ClearFlag(EXTI_Line12);
	}
}

