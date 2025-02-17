/**
*@file DogActive.h
*@author 申飞麟
*@brief 机器狗动作 
*@date 2025/2/17
*/

#ifndef DOGACTIVE_H
#define DOGACTIVE_H

void Front_LeftLeg(uint8_t Angle);//左前腿,0~180度
void Front_RightLeg(uint8_t Angle);//右前腿,0~180度
void Back_LeftLeg(uint8_t Angle);//左后腿,0~180度
void Back_RightLeg(uint8_t Angle);//右后腿,0~180度
void SitDown(void);//坐下
void Wave(void);//挥手
void StandUp(void);//起立
void Laydown(void);//趴下
void Dance(void);//跳舞
void Run(void);//前进
void LeftRun(void);//左转
void RightRun(void);//右转

#endif

