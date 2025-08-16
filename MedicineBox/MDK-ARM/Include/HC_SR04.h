/**
 * @brief 超声波控制(TIM3)
 * @date 2025/5/25
 */

#ifndef HC_SR04_H
#define HC_SR04_H

void HC_Send_Trig(void);
float HC_Get_Measure(uint8_t Temperature);

#endif
