/**
 *@file HC_SR04.h
 *@brief 超声波测距
 *@author 申飞麟
 *@date 2025/8/13
 **/

#ifndef HC_SR04_H
#define HC_SR04_H

#include "main.h"

void Delay_Us(uint32_t us);
void Delay_Us_Init(void);
void HC_SR04_Send_Trig(void);

#endif //HC_SR04_H
