/**
* @file Vofa_Debug.h
 * @brief VOFA调试程序
 * @author Shen FeiLin
 * @date 2025/11/1
 */

#ifndef VOFA_DEBUG_H
#define VOFA_DEBUG_H

#include "main.h"

#define LED_Red_Up HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
#define LED_Red_Down HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
#define LED_Green_Up HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
#define LED_Green_Down HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
#define LED_Blue_Up HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);
#define LED_Blue_Down HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
#define LED_Clear_All \
    HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET); \
    HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET); \
    HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);

void VOFA_Print_RC(void);
void VOFA_Print_INS(void);

#endif //VOFA_DEBUG_H
