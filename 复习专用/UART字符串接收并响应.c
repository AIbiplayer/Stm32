#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "GPIOX_Init.h"
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

char String[50];
bool RxFlag=false;//数据接收成功标志

int main(void)
{
	OLED_Init();
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	GPIOX_Init(GPIOA,RCC_APB2Periph_GPIOA,GPIO_Speed_50MHz,GPIO_Mode_Out_PP,GPIO_Pin_0);//TX
	GPIOX_Init(GPIOA,RCC_APB2Periph_GPIOA,GPIO_Speed_50MHz,GPIO_Mode_AF_PP,GPIO_Pin_9);//TX
	GPIOX_Init(GPIOA,RCC_APB2Periph_GPIOA,GPIO_Speed_50MHz,GPIO_Mode_IPU,GPIO_Pin_10);//RX

	USART_InitTypeDef UInit1;
	UInit1.USART_BaudRate=9600;
	UInit1.USART_HardwareFlowControl=USART_HardwareFlowControl_None;//无硬件流控
	UInit1.USART_Mode=USART_Mode_Tx|USART_Mode_Rx;//发送|接收模式
	UInit1.USART_Parity=USART_Parity_No;//不校验
	UInit1.USART_StopBits=USART_StopBits_1;//1停止位
	UInit1.USART_WordLength=USART_WordLength_8b;//8位长度
	USART_Init(USART1,&UInit1);
	
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);//USART1中断，当收到数据时启动中断
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_InitTypeDef NInit;
	NInit.NVIC_IRQChannel=USART1_IRQn;//USART1中断
	NInit.NVIC_IRQChannelCmd=ENABLE;
	NInit.NVIC_IRQChannelPreemptionPriority=1;
	NInit.NVIC_IRQChannelSubPriority=1;
	NVIC_Init(&NInit);
	
	USART_Cmd(USART1,ENABLE);
	
	GPIO_SetBits(GPIOA,GPIO_Pin_0);

	while(1)
	{
		if(!RxFlag)continue;

		OLED_Clear();
		if(!strcmp(String,"Barking"))
		{
			GPIO_ResetBits(GPIOA,GPIO_Pin_0);
			OLED_ShowString(1,1,String);
		}
		if(!strcmp(String,"NOBarking"))
		{
			GPIO_SetBits(GPIOA,GPIO_Pin_0);
			OLED_ShowString(1,1,String);
		}
		RxFlag=false;//读出数据后及时清零标志，减少数据冲突	
	}
}

void USART1_IRQHandler(void)//数据包读取，注意不要调用OLED，否则出错
{
	static uint8_t Status=0,Sum=0;
	
	if(USART_GetFlagStatus(USART1,USART_FLAG_RXNE)==SET)
	{
		char Data=USART_ReceiveData(USART1);//把数据存入Data中，避免出错
		switch(Status)
		{
			case 0:
				if(Data =='M'&& !RxFlag)//字符串开头为M，否则不读取
				{
					Status++;
				}
				break;
			case 1:
				if(Data!='\r')//字符串结尾为回车(\r + \n)，否则不读取
				{
					String[Sum]=Data;
					Sum++;
					break;
				}
				if(Data!='\n')
				{
					String[Sum]='\0';
					Status=0;
					Sum=0;
					RxFlag=true;
				}
				break;
		}
		USART_ClearFlag(USART1,USART_FLAG_RXNE);
	}
	USART_ClearITPendingBit(USART1,USART_FLAG_RXNE);
}


