/**
*@file DogActive.c
*@author �����
*@brief ���������� 
*@date 2025/2/16
*/

#include "stm32f10x.h" 

uint8_t ReverseAngle(uint8_t Angle,uint8_t Count)//��ת��
{
	if(!(Count%2))
	{
		return Angle;
	}
	return 180-Angle;
}

void SitDown(void)//����
{
	Front_LeftLeg(90);
	Front_RightLeg(90);
	Back_LeftLeg(45);
	Back_RightLeg(135);
}

void StandUp(void)//����
{
	Front_LeftLeg(90);
	Front_RightLeg(90);
	Back_LeftLeg(90);
	Back_RightLeg(90);
}

void Laydown(void)//ſ��
{
	Front_LeftLeg(15);
	Front_RightLeg(165);
	Back_LeftLeg(15);
	Back_RightLeg(160);
}

void Wave(void)//����
{
	SitDown();
	Delayms(500);
	for(uint8_t i=0;i<5;i++)
	{
		Front_LeftLeg(0);
		Delayms(300);
		Front_LeftLeg(40);
		Delayms(300);
	}
}

void LeftRun(void)//��ת
{
	StandUp();
	Delayms(500);
	for(uint8_t i=0;i<10;i++)
	{
		Front_LeftLeg(40);
		Back_RightLeg(40);
		Delayms(250);
		Front_RightLeg(40);
		Back_LeftLeg(40);
		Delayms(250);
		Front_LeftLeg(90);
		Back_RightLeg(90);
		Delayms(250);
		Front_RightLeg(90);
		Back_LeftLeg(90);
		Delayms(250);
	}
}

void RightRun(void)//��ת
{
	StandUp();
	Delayms(500);
	for(uint8_t i=0;i<10;i++)
	{
		Front_LeftLeg(140);
		Back_RightLeg(140);
		Delayms(250);
		Front_RightLeg(140);
		Back_LeftLeg(140);
		Delayms(250);
		Front_LeftLeg(90);
		Back_RightLeg(90);
		Delayms(250);
		Front_RightLeg(90);
		Back_LeftLeg(90);
		Delayms(250);
	}
}

void Run(void)//ǰ��
{
	StandUp();
	Delayms(500);
	for(uint8_t i=0;i<10;i++)
	{
		Front_LeftLeg(40);
		Back_RightLeg(140);
		Delayms(250);
		Front_RightLeg(140);
		Back_LeftLeg(40);
		Delayms(250);
		Front_LeftLeg(90);
		Back_RightLeg(90);
		Delayms(250);
		Front_RightLeg(90);
		Back_LeftLeg(90);
		Delayms(250);
		
		Front_RightLeg(40);
		Back_LeftLeg(140);
		Delayms(250);
		Front_LeftLeg(140);
		Back_RightLeg(40);
		Delayms(250);
		Front_RightLeg(90);
		Back_LeftLeg(90);
		Delayms(250);
		Front_LeftLeg(90);
		Back_RightLeg(90);
		Delayms(250);
	}
}
	
void Dance(void)//����
{
	StandUp();
	Delayms(500);
	for(uint8_t i=0;i<10;i++)
	{
		Front_LeftLeg(0);
		Back_LeftLeg(180);
		Delayms(300);
		Front_RightLeg(180);
		Back_RightLeg(0);
		Delayms(300);
		Front_LeftLeg(90);
		Back_LeftLeg(90);
		Delayms(300);
		Front_RightLeg(90);
		Back_RightLeg(90);
		Delayms(300);
		for(uint8_t k=0;k<3;k++)
		{
			Front_LeftLeg(ReverseAngle(45,k));
			Front_RightLeg(ReverseAngle(135,k));
			Back_LeftLeg(ReverseAngle(45,k));
			Back_RightLeg(ReverseAngle(135,k));
			Delayms(300);
		}
		StandUp();
		Delayms(300);
	}
}

void Front_LeftLeg(uint8_t Angle)//��ǰ��,0~180��
{
	TIM_SetCompare2(TIM2,(Angle/90.0f+0.5f)*20);
}
void Front_RightLeg(uint8_t Angle)//��ǰ��,0~180��
{
	TIM_SetCompare1(TIM2,(Angle/90.0f+0.5f)*20);
}
void Back_LeftLeg(uint8_t Angle)//�����,0~180��
{
	TIM_SetCompare3(TIM2,(Angle/90.0f+0.5f)*20);
}
void Back_RightLeg(uint8_t Angle)//�Һ���,0~180��
{
	TIM_SetCompare4(TIM2,(Angle/90.0f+0.5f)*20);
}

