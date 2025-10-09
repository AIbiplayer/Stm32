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
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "BlueTooth.h"
#include "Chassis.h"
#include "OLED.h"
#include "ShowShape.h"
#include "Key.h"
#include  "Infrare.h"
#include "mpu6050.h"

extern uint8_t Receive_Gimbal_Mode;
extern Control_Mode Chassis_Mode;
extern bool Mode_Change_Flag;
extern bool Inf_ReceiveFlag;
extern Chassis Chassis_Control;
extern double pitch_, roll_, yaw;

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
    MX_I2C1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();
    MX_USART3_UART_Init();
    /* USER CODE BEGIN 2 */

    OLED_Init();
    MPU6050_Init();
    Uart_Init();
    Inf_Stop();
    BlueTooth_Stop();
    HAL_TIM_Base_Start_IT(&htim3);

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        Mode_Change_Flag ? (Mode_Change_Flag = false, Uart_printf(&huart1, "M%d\n", (int8_t)Chassis_Mode)) : 0;
        //向蓝牙传输模式

        if (Chassis_Mode == INFRARE_MODE && Inf_ReceiveFlag) //解析红外线
        {
            Inf_ReceiveFlag = false;
            Inf_Server();
        }

        if (Chassis_Mode == BLUETOOTH_MODE)
            switch (Receive_Gimbal_Mode) //云台模式
            {
            case 0:
                Yaw_SetAngle(90.0f); // 设置偏航舵机初始位置
                Pitch_SetAngle(90.0f); // 设置俯仰舵机初始位置
                break;
            case 1:
                ShowSin();
                break;
            case 2:
                ShowTriangle();
                break;
            case 3:
                ShowCircal();
                break;
            }

        Chassis_Behavior();

        OLED_ShowString(1, 1, "Pit:");
        OLED_ShowString(2, 1, "Rol:");
        OLED_ShowString(3, 1, "Yaw:");
        OLED_ShowSignedNum(1, 5, (int32_t)pitch_, 2);
        OLED_ShowSignedNum(2, 5, (int32_t)roll_, 2);
        OLED_ShowSignedNum(3, 5, (int32_t)yaw, 2);
        OLED_ShowString(1, 10, "CM:");
        switch (Chassis_Mode)
        {
        case 0:
            OLED_ShowString(1, 14, "BT");
            break;
        case 1:
            OLED_ShowString(1, 14, "IR");
            break;
        case 2:
            OLED_ShowString(1, 14, "NO");
            break;
        default:
            break;
        }
        OLED_ShowString(2, 10, "CH:");
        switch (Chassis_Control.Status)
        {
        case 0:
            OLED_ShowNum(2, 14, 3, 1);
            break;
        case 1:
            OLED_ShowNum(2, 14, 4, 1);
            break;
        default:
            break;
        }

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
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
        | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
