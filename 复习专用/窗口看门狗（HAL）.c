/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "OLED.h"
#include "stm32f1xx_hal_pwr.h"
#include "stm32f1xx_hal_rtc.h"
#include "stm32f1xx_hal_wwdg.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

WWDG_HandleTypeDef hwwdg;

/* USER CODE BEGIN PV */
RTC_TimeTypeDef RTime = {0};
RTC_AlarmTypeDef RAlarm = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_WWDG_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */
	
	/* USER CODE END 1 */
	
	/* MCU Configuration--------------------------------------------------------*/
	
	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();
	
	/* USER CODE BEGIN Init */
	OLED_Init();
	/* USER CODE END Init */
	
	/* Configure the system clock */
	SystemClock_Config();
	
	/* USER CODE BEGIN SysInit */
	
	/* USER CODE END SysInit */
	
	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_RTC_Init();
	/* USER CODE BEGIN 2 */
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST) != SET)
	{
		OLED_ShowString(2, 1, "WWDG");
		HAL_Delay(100);
		OLED_ShowString(2, 1, "    ");
	}
	else
	{
		OLED_ShowString(1, 1, "RESET");
		HAL_Delay(100);
		OLED_ShowString(1, 1, "     ");
	}
	
	/* USER CODE END 2 */
	
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	MX_WWDG_Init(); // 初始化放在最后，Cube每生成一次项目，上面就会出现一样的WWDG_Init，记得删除，不然时间不对
	
	/*
	  计算公式：（单位：ms）
	  T最晚 = f(PCLK1按毫秒计) x 4096 x 分频 x (T[6:0] -63) 
	  T窗口 = f(PCLK1按毫秒计) x 4096 x 分频 x (T[6:0] - W[6:0])
	  
	  比如 PCLK1=8MHz，8分频，我想让看门狗窗口为 30ms，最晚为 50ms，那么 50ms = (1/8000) x 4096 x 8 x (T[6:0] -63) 
	  可解出 (T[6:0] -63) = 12，T[6:0] = 75
	  30ms = (1/8000) x 4096 x 8 x (T[6:0] - W[6:0])
	  (T[6:0] - W[6:0]) = 7， W[6:0] = 68
	  
	  将 W[6:0] 和  T[6:0] 带入Cube中即可
	  
	 */
	
	while (1)
	{
		HAL_Delay(60);
		HAL_WWDG_Refresh(&hwwdg);
		
		/* USER CODE END WHILE */
		
		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
	
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSE;
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	
	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
	{
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
	PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief RTC Initialization Function
 * @param None
 * @retval None
 */
static void MX_RTC_Init(void)
{
	
	/* USER CODE BEGIN RTC_Init 0 */
	
	/* USER CODE END RTC_Init 0 */
	
	/* USER CODE BEGIN RTC_Init 1 */
	
	/* USER CODE END RTC_Init 1 */
	
	/** Initialize RTC Only
	 */
	hrtc.Instance = RTC;
	hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
	hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
	if (HAL_RTC_Init(&hrtc) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN RTC_Init 2 */
	
	/* USER CODE END RTC_Init 2 */
}

/**
 * @brief WWDG Initialization Function
 * @param None
 * @retval None
 */
static void MX_WWDG_Init(void)
{
	
	/* USER CODE BEGIN WWDG_Init 0 */
	
	/* USER CODE END WWDG_Init 0 */
	
	/* USER CODE BEGIN WWDG_Init 1 */
	
	/* USER CODE END WWDG_Init 1 */
	hwwdg.Instance = WWDG;
	hwwdg.Init.Prescaler = WWDG_PRESCALER_8;
	hwwdg.Init.Window = 75;
	hwwdg.Init.Counter = 87;
	hwwdg.Init.EWIMode = WWDG_EWI_DISABLE;
	if (HAL_WWDG_Init(&hwwdg) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN WWDG_Init 2 */
	
	/* USER CODE END WWDG_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	
	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

