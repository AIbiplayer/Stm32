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
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "CAN_Func.h"
#include "PID.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

float Angle;
float Temp;
float Speed;
float I;
char Receive_Buffer[10];
uint16_t Receive_Angle = 0;

extern float Target;

void Uart_printf(UART_HandleTypeDef *huart, char *format, ...) // 串口打印函数
{
    static char buf[1024]; // 定义临时数组，根据实际发送大小微调
    va_list args;
    va_start(args, format);
    uint32_t len = vsnprintf((char *)buf, sizeof(buf), (char *)format, args);
    va_end(args);
    HAL_UART_Transmit(huart, (uint8_t *)buf, len, 100);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef CAN_Rx; // 接收句柄
    uint8_t Data[8] = {0};      // 数据
    HAL_CAN_GetRxMessage(hcan, CAN_FIFO_0, &CAN_Rx, Data);

    Angle = ((Data[0] << 8) | Data[1]) / 8192.0 * 360.0;
    Speed = ((Data[2] << 8) | Data[3]);
    I = ((Data[4] << 8) | Data[5]) / 65535.0 * 3.0;
    Temp = Data[6];

    Uart_printf(&huart1, "%.1f\r\n", Angle);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) // 串口接收回调函数
{
    if (huart->Instance == USART2)
    {
        sscanf(Receive_Buffer, "%d", &Receive_Angle); // 解析接收到的角度值
        if (Receive_Angle > 0 && Receive_Angle < 360) // 检查角度范围
        {
            Target = (float)Receive_Angle;
        }
        memset(Receive_Buffer, 0, sizeof(Receive_Buffer)); // 清空接收缓冲区
        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t *)Receive_Buffer, sizeof(Receive_Buffer));
        __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT); // 禁用DMA半传输中断，避免进入两次回调
    }
}

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

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

    HAL_TIM_Base_Start_IT(&htim2);
    Can_Init(&hcan);
    Can_Filter_Config(&hcan,
                      CAN_FILTER(0) | CAN_FIFO_0 | CANSTDID | CANRTR_DATA, // Filter 0, FIFO 0, Standard ID, Data Frame
                      0x208,                                               // ID to filter
                      0x7FF);                                              // Mask ID (standard frame mask)

    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t *)Receive_Buffer, sizeof(Receive_Buffer));
    __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT); // 禁用DMA半传输中断，避免进入两次回调
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
