/**
*@file DogActive.c
*@author 申飞麟
*@brief 机器狗动作 
*@date 2025/2/16
*/

#include "stm32f10x.h" 
#include "Delay.h" 

void Front_LeftLeg(uint8_t Angle)//左前腿,0~180度
{
	TIM_SetCompare2(TIM2,(Angle/90.0f+0.5f)*20);
}
void Front_RightLeg(uint8_t Angle)//右前腿,0~180度
{
	TIM_SetCompare1(TIM2,(Angle/90.0f+0.5f)*20);
}
void Back_LeftLeg(uint8_t Angle)//左后腿,0~180度
{
	TIM_SetCompare3(TIM2,(Angle/90.0f+0.5f)*20);
}
void Back_RightLeg(uint8_t Angle)//右后腿,0~180度
{
	TIM_SetCompare4(TIM2,(Angle/90.0f+0.5f)*20);
}

uint8_t ReverseAngle(uint8_t Angle,uint8_t Count)//翻转角
{
	if(!(Count%2))
	{
		return Angle;
	}
	return 180-Angle;
}

void SitDown(void)//坐下
{
	Front_LeftLeg(90);
	Front_RightLeg(90);
	Back_LeftLeg(45);
	Back_RightLeg(135);
}

void StandUp(void)//起立
{
	Front_LeftLeg(90);
	Front_RightLeg(90);
	Back_LeftLeg(90);
	Back_RightLeg(90);
}

void Laydown(void)//趴下
{
	Front_LeftLeg(15);
	Front_RightLeg(165);
	Back_LeftLeg(15);
	Back_RightLeg(160);
}

void Wave(void)//挥手
{
	for(uint8_t i=0;i<2;i++)
	{
		Front_LeftLeg(0);
		Delayms(300);
		Front_LeftLeg(40);
		Delayms(300);
	}
}

void LeftRun(void)//左转
{
	for(uint8_t i=0;i<2;i++)
	{
		Front_LeftLeg(140);
		Back_RightLeg(140);
		Delayms(200);
		Front_RightLeg(40);
		Back_LeftLeg(40);
		Delayms(200);
		Front_LeftLeg(90);
		Back_RightLeg(90);
		Delayms(200);
		Front_RightLeg(90);
		Back_LeftLeg(90);
		Delayms(200);
	}
}

void RightRun(void)//右转
{
	for(uint8_t i=0;i<2;i++)
	{
		Front_RightLeg(40);
		Back_LeftLeg(40);
		Delayms(200);
		Front_LeftLeg(140);
		Back_RightLeg(140);
		Delayms(200);
		Front_RightLeg(90);
		Back_LeftLeg(90);
		Delayms(200);
		Front_LeftLeg(90);
		Back_RightLeg(90);
		Delayms(200);
	}
}

void Run(void)//前进
{
	for(uint8_t i=0;i<2;i++)
	{
		Front_LeftLeg(40);
		Back_RightLeg(140);
		Delayms(250);
		Front_RightLeg(40);
		Back_LeftLeg(140);
		Delayms(250);
		Front_LeftLeg(90);
		Back_RightLeg(90);
		Delayms(250);
		Front_RightLeg(90);
		Back_LeftLeg(90);
		Delayms(250);
		
		Front_RightLeg(140);
		Back_LeftLeg(40);
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

void BackRun(void)//后退
{
	for(uint8_t i=0;i<2;i++)
	{
		Front_LeftLeg(140);
		Back_RightLeg(40);
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
		Front_LeftLeg(40);
		Back_RightLeg(140);
		Delayms(250);
		Front_RightLeg(90);
		Back_LeftLeg(90);
		Delayms(250);
		Front_LeftLeg(90);
		Back_RightLeg(90);
		Delayms(250);
	}
}
	
void Dance(void)//跳舞
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


