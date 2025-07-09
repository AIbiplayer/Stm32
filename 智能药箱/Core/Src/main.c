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
#include "adc.h"
#include "fmac.h"
#include "gpdma.h"
#include "i2c.h"
#include "icache.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "UART.h"
#include "oled.h"
#include "font.h"
#include "Flash.h"
#include "Motor.h"
#include "DHT11.h"
#include "GRB.h"
#include "HC_SR04.h"
#include "Key.h"
#include "RTC_Medicine.h"
#include "stm32u5xx_hal_rtc.h"
#include "stm32u5xx_hal_uart.h"
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

/* USER CODE BEGIN PV */

RTC_TimeTypeDef sTime = {0};
RTC_DateTypeDef sDate = {0};
RTC_AlarmTypeDef sAlarm = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void SystemPower_Config(void);
/* USER CODE BEGIN PFP */

/**
 * @brief 延迟函数修改
 */
void HAL_IncTick(void)
{
    if (uwTickFreq == 0)
    {
        uwTickFreq = HAL_TICK_FREQ_DEFAULT;
    }
    uwTick += uwTickFreq;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

extern uint8_t Fan_Status;
extern uint8_t Temp, Humid;            ///< 温湿�???????????????????
extern uint8_t Light_Status;           ///< 夜间�???????????????????关灯状�??
extern uint32_t ADC_Light_Data;        ///< 光敏值，范围�???????????????????0~4095
extern uint8_t Day_Night_Status;       ///< 白天和夜晚状态，夜晚�???????????????????1
extern uint8_t Medicine_Remind_Status; ///< 服药提醒状�?�位�??????????????1为到点提�??????????????
extern const char Voice_Reminder[];
extern const char Voice_MedicineOK[]; ///< 网络连接成功语音
extern uint8_t Key_Status;            // 键盘从左�????右为1~4，从上往下为1�????5�????9�????13，对�????0~F标记

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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the System Power */
  SystemPower_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_GPDMA1_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_RTC_Init();
  MX_ICACHE_Init();
  MX_FMAC_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_ADC4_Init();
  MX_TIM5_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

    All_Connect();

    DHT11_Delay_Init();                                                                                       // DWT计时器开�???????????????????
    HAL_TIM_Base_Start_IT(&htim5);                                                                            // �???????????????????启温度测�???????????????????
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);                                                               // �???????????????????启超声波测量
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);                                                              // �???????????????????启ESP01
    HAL_TIM_PWM_Start_DMA(&htim4, TIM_CHANNEL_1, (uint32_t *)LED_Array, sizeof(LED_Array) / sizeof(uint8_t)); // �???????????????????启灯�???????????????????

    HAL_ADCEx_Calibration_Start(&hadc4, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED); // �???????????????????启光敏传�???????????????????
    HAL_Delay(1);
    HAL_ADC_Start_DMA(&hadc4, &ADC_Light_Data, 1);
    __HAL_ADC_ENABLE_IT(&hadc4, ADC_IT_AWD1);

    // HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_SET); // 亮红�??????????
    // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET); // 亮蓝�??????????
    // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET); // 亮绿�??????????

    HSV Yellow_Color = {0}; ///< 黄色夜灯
    Yellow_Color.H = 40;
    Yellow_Color.S = 1.0f;
    Yellow_Color.V = 0.01f;

    HSV Orange_Color = {0}; ///< 橙色提醒
    Orange_Color.H = 15;
    Orange_Color.S = 1.0f;
    Orange_Color.V = 0.5f;

    HSV Blue_Color = {0}; ///< 蓝色制冷�??????????
    Blue_Color.H = 240;
    Blue_Color.S = 0.8f;
    Blue_Color.V = 0.5f;
    GRB GRB_Blue = {0};
    HSV_To_GRB(&Blue_Color, &GRB_Blue);

    GRB GRB_Yellow = {0};
    HSV_To_GRB(&Yellow_Color, &GRB_Yellow);

    float Measure = 0;
    uint8_t Night_Breath_Status = 0; ///< 判断白天是否关闭夜灯�???????????????????1为开启夜灯，使关灯只执行�???????????????????�???????????????????
    uint8_t Night_Light_Init = 0;    ///< 夜晚�??????????????�??????????????始亮�??????????????暗的灯，后面不变，亮灯为1

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
        HAL_Delay(200);
        if (Light_Status)
        {
            if (Medicine_Remind_Status == 2) // 服药成功提示
            {
                GRB_Medicine_Successful();
                Voice_Remind(Voice_MedicineOK);
            }
            else if (ADC_Light_Data > 3600) // 夜间状�??
            {
                if (!Night_Light_Init) // 亮黄夜灯
                {
                    for (uint8_t i = 0; i < 60; i++)
                    {
                        GRB_Write(i, GRB_Yellow);
                    }
                    GRB_Send();
                    Night_Light_Init = 1;
                }
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
                HC_Send_Trig();
                HAL_Delay(50);
                Measure = HC_Get_Measure(Temp);
                GRB_Distance_Breath(&Yellow_Color, Measure);
                Night_Breath_Status = 1;
            }
            else if (Medicine_Remind_Status == 1) // 服药提醒�??????????
            {
                GRB_Breath(&Orange_Color);
                Voice_Remind(Voice_Reminder);
            }
            else if (Fan_Status == 1) // 制冷�??????????
            {
                for (uint8_t i = 0; i < 60; i++)
                {
                    GRB_Write(i, GRB_Blue);
                }
                GRB_Send();
            }
            else if (ADC_Light_Data < 3550) // 白天关夜�??????????
            {
                Night_Light_Init = 0;
                GRB_Close();
                Night_Breath_Status = 0;
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
            }
        }
        else if (Night_Breath_Status)
        {
            GRB_Close();
            Night_Breath_Status = 0;
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
        }
        HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
        HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
        HAL_RTC_GetAlarm(&hrtc, &sAlarm, RTC_ALARM_A, RTC_FORMAT_BIN);
        Motor_Operation(&Cabinet_List);
        Uart_printf(&huart1, "Time:%d/%d/%d %d:%d:%d\nAlarm:%d %d:%d", sDate.Year, sDate.Month, sDate.Date, sTime.Hours, sTime.Minutes, sTime.Seconds, sAlarm.AlarmDateWeekDay, sAlarm.AlarmTime.Hours, sAlarm.AlarmTime.Minutes);

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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMBOOST = RCC_PLLMBOOST_DIV1;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 32;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 1;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLLVCIRANGE_0;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Power Configuration
  * @retval None
  */
static void SystemPower_Config(void)
{
  HAL_PWREx_EnableVddIO2();

  /*
   * Disable the internal Pull-Up in Dead Battery pins of UCPD peripheral
   */
  HAL_PWREx_DisableUCPDDeadBattery();

  /*
   * Switch to SMPS regulator instead of LDO
   */
  if (HAL_PWREx_ConfigSupply(PWR_SMPS_SUPPLY) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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

#ifdef  USE_FULL_ASSERT
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
