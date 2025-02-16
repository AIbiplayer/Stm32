#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "GPIOX_Init.h"
#include <stdio.h>
#include <stdbool.h>

uint8_t Array1[]={0,0,0,0};
bool RxFlag=0;//���ݽ��ճɹ���־

/*void SendArray(uint16_t *a,uint8_t Length);
void SendPacket(uint16_t *a,uint8_t Length);*/

int main(void)
{
	OLED_Init();
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	GPIOX_Init(GPIOA,RCC_APB2Periph_GPIOA,GPIO_Speed_50MHz,GPIO_Mode_AF_PP,GPIO_Pin_9);//TX
	GPIOX_Init(GPIOA,RCC_APB2Periph_GPIOA,GPIO_Speed_50MHz,GPIO_Mode_IPU,GPIO_Pin_10);//RX

	USART_InitTypeDef UInit1;
	UInit1.USART_BaudRate=9600;
	UInit1.USART_HardwareFlowControl=USART_HardwareFlowControl_None;//��Ӳ������
	UInit1.USART_Mode=USART_Mode_Tx|USART_Mode_Rx;//����|����ģʽ
	UInit1.USART_Parity=USART_Parity_No;//��У��
	UInit1.USART_StopBits=USART_StopBits_1;//1ֹͣλ
	UInit1.USART_WordLength=USART_WordLength_8b;//8λ����
	USART_Init(USART1,&UInit1);
	
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);//USART1�жϣ����յ�����ʱ�����ж�
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_InitTypeDef NInit;
	NInit.NVIC_IRQChannel=USART1_IRQn;//USART1�ж�
	NInit.NVIC_IRQChannelCmd=ENABLE;
	NInit.NVIC_IRQChannelPreemptionPriority=1;
	NInit.NVIC_IRQChannelSubPriority=1;
	NVIC_Init(&NInit);
	
	USART_Cmd(USART1,ENABLE);
	
	OLED_ShowString(1,1,"Status:");

	while(1)
	{
		if(RxFlag==true)
		{
			OLED_ShowHexNum(2,1,Array1[0],2);
			OLED_ShowHexNum(2,4,Array1[1],2);
			OLED_ShowHexNum(2,7,Array1[2],2);
			OLED_ShowHexNum(2,10,Array1[3],2);
			Delayms(50);
			RxFlag=false;
		}
	}
}

void USART1_IRQHandler(void)//���ݰ���ȡ��ע�ⲻҪ����OLED���������
{
	static uint8_t Status=0,Sum=0;
	
	if(USART_GetFlagStatus(USART1,USART_FLAG_RXNE)==SET)
	{
		uint8_t Data=USART_ReceiveData(USART1);//�����ݴ���Data�У��������
		switch(Status)
		{
			case 0:
				if(Data==0xAA)
				{
					Status++;
					Sum=0;
				}
				break;
			case 1:
				if(Sum<3)//���ݸ���Ϊ4
				{
					Array1[Sum]=Data;
					Sum++;
					break;
				}
				Array1[Sum]=Data;
				Status++;
				Sum=0;
				break;
			case 2:
				if(Data==0xAB)
				{
					Status=0;
					RxFlag=true;
				}
				break;
		}
		USART_ClearFlag(USART1,USART_FLAG_RXNE);
	}
	USART_ClearITPendingBit(USART1,USART_FLAG_RXNE);
}

/*void SendPacket(uint16_t *a,uint8_t Length)
{
	USART_SendData(USART1,0xAA);
	Delayms(5);
	SendArray(a,4);
	USART_SendData(USART1,0xAB);
}*/

