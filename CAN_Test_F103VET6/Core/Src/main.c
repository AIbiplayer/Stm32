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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "stdarg.h"
#include "stdio.h"
#include "stm32f1xx_hal_can.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define CAN_FILTER(x) ((x) << 3)

#define CAN_FIFO_0 (0 << 2)
#define CAN_FIFO_1 (1 << 2)

#define CANSTDID (0 << 1) // 标准ID
#define CANEXTID (1 << 1) // 扩展ID

#define CANRTR_DATA (0 << 0)   // 数据帧
#define CANRTR_REMOTE (1 << 0) // 远程帧

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

void Uart_printf(UART_HandleTypeDef *huart, char *format, ...) // 串口打印函数
{
    static char buf[1024]; // 定义临时数组，根据实际发送大小微�??
    va_list args;
    va_start(args, format);
    uint32_t len = vsnprintf((char *)buf, sizeof(buf), (char *)format, args);
    va_end(args);
    HAL_UART_Transmit_DMA(huart, (uint8_t *)buf, len);
}

void Can_Init(CAN_HandleTypeDef *hcan)
{
    HAL_CAN_Start(hcan);
    __HAL_CAN_ENABLE_IT(hcan, CAN_IT_RX_FIFO0_MSG_PENDING); // 使能接收中断
    __HAL_CAN_ENABLE_IT(hcan, CAN_IT_RX_FIFO1_MSG_PENDING); // 使能接收中断
}

HAL_StatusTypeDef Can_SendData(CAN_HandleTypeDef *hcan, uint16_t ID, uint8_t *Data, uint16_t Length)
{
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;

    TxHeader.StdId = ID; // 设置标准ID
    TxHeader.ExtId = 0;
    TxHeader.IDE = 0;
    TxHeader.RTR = 0;
    TxHeader.DLC = Length;

    return HAL_CAN_AddTxMessage(hcan, &TxHeader, Data, &TxMailbox);
}

void Can_Filter_Config(CAN_HandleTypeDef *hcan, uint8_t Object_Para, uint32_t ID, uint32_t Mask_ID)
{
    CAN_FilterTypeDef Can_Filter;

    if ((Object_Para & 0x02)) // 标准帧
    {
        Can_Filter.FilterIdHigh = ID << 3 >> 16;
        Can_Filter.FilterIdLow = ID << 3 | ((Object_Para & 0x03) << 1);
        Can_Filter.FilterMaskIdHigh = Mask_ID << 3 >> 16;
        Can_Filter.FilterMaskIdLow = Mask_ID << 3 | ((Object_Para & 0x03) << 1);
    }
    else // 遥控帧
    {
        Can_Filter.FilterIdHigh = ID << 5;
        Can_Filter.FilterIdLow = ((Object_Para & 0x03) << 1);
        Can_Filter.FilterMaskIdHigh = Mask_ID << 5;
        Can_Filter.FilterMaskIdLow = ((Object_Para & 0x03) << 1);
    }

    Can_Filter.FilterBank = Object_Para >> 3;
    Can_Filter.FilterFIFOAssignment = (Object_Para >> 2) & 0x01;
    Can_Filter.FilterActivation = ENABLE;
    Can_Filter.FilterMode = CAN_FILTERMODE_IDMASK;
    Can_Filter.FilterScale = CAN_FILTERSCALE_32BIT;
    Can_Filter.SlaveStartFilterBank = 14;
    HAL_CAN_ConfigFilter(hcan, &Can_Filter);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef CAN_Rx;
    uint8_t Data[8] = {0};
    HAL_CAN_GetRxMessage(hcan, CAN_FIFO_0, &CAN_Rx, Data);

    if (Data[0] == 0x01 && Data[1] == 0x02 && Data[2] == 0x03 && Data[3] == 0x04 && Data[4] == 0x05 && Data[5] == 0x06 && Data[6] == 0x07 && Data[7] == 0x08)
    {
        HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_5);
    }
}

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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */

    Can_Init(&hcan);
    Can_Filter_Config(&hcan, CAN_FILTER(13) | CAN_FIFO_0 | CANSTDID | CANRTR_DATA, 0x124, 0x7FE);

    // HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET);
    // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
    uint8_t Data[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
        if (Can_SendData(&hcan, 0x124, Data, 8) == HAL_ERROR)
        {
            HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_5);
        }
        HAL_Delay(1000); // 发送间隔1秒
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
