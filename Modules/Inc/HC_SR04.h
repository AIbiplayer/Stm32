/**
* @file HC_SR04.h
 * @brief 超声波控制
 * @author Shen FeiLin
 * @date 2025/12/12
 */

#ifndef HC_SR04_H
#define HC_SR04_H

#include "main.h"

void HC_Init(void);
void HC_Send_Trig(void);
float HC_Get_Measure(void);

#endif //HC_SR04_H
