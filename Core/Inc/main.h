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
#include "stm32f4xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define STEER_1_Pin GPIO_PIN_0
#define STEER_1_GPIO_Port GPIOA
#define STEER_2_Pin GPIO_PIN_1
#define STEER_2_GPIO_Port GPIOA
#define BlueTooth_TX_Pin GPIO_PIN_2
#define BlueTooth_TX_GPIO_Port GPIOA
#define BlueTooth_RX_Pin GPIO_PIN_3
#define BlueTooth_RX_GPIO_Port GPIOA
#define PS2_DAT_Pin GPIO_PIN_4
#define PS2_DAT_GPIO_Port GPIOA
#define PS2_CMD_Pin GPIO_PIN_5
#define PS2_CMD_GPIO_Port GPIOA
#define PS2_CS_Pin GPIO_PIN_6
#define PS2_CS_GPIO_Port GPIOA
#define PS2_CLK_Pin GPIO_PIN_7
#define PS2_CLK_GPIO_Port GPIOA
#define BUZZ_Pin GPIO_PIN_0
#define BUZZ_GPIO_Port GPIOB
#define OLED_SDA_Pin GPIO_PIN_9
#define OLED_SDA_GPIO_Port GPIOC
#define OLED_SCL_Pin GPIO_PIN_8
#define OLED_SCL_GPIO_Port GPIOA
#define K210_TX_Pin GPIO_PIN_12
#define K210_TX_GPIO_Port GPIOC
#define K210_RX_Pin GPIO_PIN_2
#define K210_RX_GPIO_Port GPIOD
#define MPU_SCL_Pin GPIO_PIN_6
#define MPU_SCL_GPIO_Port GPIOB
#define MPU_SDA_Pin GPIO_PIN_7
#define MPU_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
