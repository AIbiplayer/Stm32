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
#include "stm32f1xx_hal.h"

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
#define LED2_Pin GPIO_PIN_13
#define LED2_GPIO_Port GPIOC
#define Steer1_Pin GPIO_PIN_0
#define Steer1_GPIO_Port GPIOA
#define Steer2_Pin GPIO_PIN_1
#define Steer2_GPIO_Port GPIOA
#define LED7_Pin GPIO_PIN_12
#define LED7_GPIO_Port GPIOB
#define LED8_Pin GPIO_PIN_13
#define LED8_GPIO_Port GPIOB
#define OLED_SCL_Pin GPIO_PIN_14
#define OLED_SCL_GPIO_Port GPIOB
#define OLED_SDA_Pin GPIO_PIN_15
#define OLED_SDA_GPIO_Port GPIOB
#define BlueTooth_TX_Pin GPIO_PIN_9
#define BlueTooth_TX_GPIO_Port GPIOA
#define BlueTooth_RX_Pin GPIO_PIN_10
#define BlueTooth_RX_GPIO_Port GPIOA
#define LED5_Pin GPIO_PIN_11
#define LED5_GPIO_Port GPIOA
#define LED6_Pin GPIO_PIN_12
#define LED6_GPIO_Port GPIOA
#define Buzz_Pin GPIO_PIN_15
#define Buzz_GPIO_Port GPIOA
#define Infrare_Pin GPIO_PIN_3
#define Infrare_GPIO_Port GPIOB
#define Infrare_EXTI_IRQn EXTI3_IRQn
#define SW2_Pin GPIO_PIN_4
#define SW2_GPIO_Port GPIOB
#define SW3_Pin GPIO_PIN_5
#define SW3_GPIO_Port GPIOB
#define MPU_SCL_Pin GPIO_PIN_6
#define MPU_SCL_GPIO_Port GPIOB
#define MPU_SDA_Pin GPIO_PIN_7
#define MPU_SDA_GPIO_Port GPIOB
#define SW4_Pin GPIO_PIN_8
#define SW4_GPIO_Port GPIOB
#define SW5_Pin GPIO_PIN_9
#define SW5_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
