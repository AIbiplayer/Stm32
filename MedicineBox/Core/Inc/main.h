/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32u5xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Key_Line_3_Pin GPIO_PIN_2
#define Key_Line_3_GPIO_Port GPIOE
#define HC_SR04_Receive_Pin GPIO_PIN_3
#define HC_SR04_Receive_GPIO_Port GPIOE
#define Key_Line_2_Pin GPIO_PIN_4
#define Key_Line_2_GPIO_Port GPIOE
#define Key_Line_1_Pin GPIO_PIN_5
#define Key_Line_1_GPIO_Port GPIOE
#define HC_SR04_Send_Pin GPIO_PIN_6
#define HC_SR04_Send_GPIO_Port GPIOE
#define DHT11_Normal_Pin GPIO_PIN_3
#define DHT11_Normal_GPIO_Port GPIOC
#define Motor_Rotate_Pin GPIO_PIN_0
#define Motor_Rotate_GPIO_Port GPIOA
#define Motor_UD_Pin GPIO_PIN_1
#define Motor_UD_GPIO_Port GPIOA
#define DHT11_Cold_Pin GPIO_PIN_0
#define DHT11_Cold_GPIO_Port GPIOB
#define SPI_RST_Pin GPIO_PIN_10
#define SPI_RST_GPIO_Port GPIOE
#define SPI_DC_Pin GPIO_PIN_12
#define SPI_DC_GPIO_Port GPIOE
#define SPI_CS_Pin GPIO_PIN_14
#define SPI_CS_GPIO_Port GPIOE
#define Cold_In_Pin GPIO_PIN_11
#define Cold_In_GPIO_Port GPIOB
#define Red_Light_Pin GPIO_PIN_2
#define Red_Light_GPIO_Port GPIOG
#define Green_Light_Pin GPIO_PIN_7
#define Green_Light_GPIO_Port GPIOC
#define Key_Line_4_Pin GPIO_PIN_3
#define Key_Line_4_GPIO_Port GPIOD
#define Key_Column_1_Pin GPIO_PIN_4
#define Key_Column_1_GPIO_Port GPIOD
#define Key_Column_2_Pin GPIO_PIN_5
#define Key_Column_2_GPIO_Port GPIOD
#define Key_Column_3_Pin GPIO_PIN_6
#define Key_Column_3_GPIO_Port GPIOD
#define Key_Column_4_Pin GPIO_PIN_7
#define Key_Column_4_GPIO_Port GPIOD
#define Blue_Light_Pin GPIO_PIN_7
#define Blue_Light_GPIO_Port GPIOB
#define Light_On_Off_Pin GPIO_PIN_8
#define Light_On_Off_GPIO_Port GPIOB
#define Light_On_Off_EXTI_IRQn EXTI8_IRQn
#define Cold_Out_Pin GPIO_PIN_0
#define Cold_Out_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
