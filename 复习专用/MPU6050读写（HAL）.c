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
#include "stm32f1xx_hal_i2c.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct SensorData
{
	int16_t GYRO_X;  ///< 陀螺仪X轴
	int16_t GYRO_Y;  ///< 陀螺仪Y轴
	int16_t GYRO_Z;  ///< 陀螺仪Z轴
	int16_t ACCEL_X; ///< 加速度X轴
	int16_t ACCEL_Y; ///< 加速度Y轴
	int16_t ACCEL_Z; ///< 加速度Z轴
} SensorData;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MPU6050_SMPLRT_DIV 0x19   // 数据输出的快慢，值越小越快
#define MPU6050_CONFIG 0x1A       // 配置寄存器设置
#define MPU6050_GYRO_CONFIG 0x1B  // 陀螺仪设置
#define MPU6050_ACCEL_CONFIG 0x1C // 加速度器设置
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_ACCEL_XOUT_L 0x3C
#define MPU6050_ACCEL_YOUT_H 0x3D
#define MPU6050_ACCEL_YOUT_L 0x3E
#define MPU6050_ACCEL_ZOUT_H 0x3F
#define MPU6050_ACCEL_ZOUT_L 0x40
#define MPU6050_TEMP_OUT_H 0x41 // 温度测量
#define MPU6050_TEMP_OUT_L 0x42
#define MPU6050_GYRO_XOUT_H 0x43
#define MPU6050_GYRO_XOUT_L 0x44
#define MPU6050_GYRO_YOUT_H 0x45
#define MPU6050_GYRO_YOUT_L 0x46
#define MPU6050_GYRO_ZOUT_H 0x47
#define MPU6050_GYRO_ZOUT_L 0x48
#define MPU6050_PWR_MGMT_1 0x6B // 电源设置
#define MPU6050_PWR_MGMT_2 0x6C
#define MPU6050_WHO_AM_I 0x75 // MPU地址
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
uint8_t MPU6050_Read(uint8_t ADDRESS)
{
	uint8_t Data;
	HAL_I2C_Mem_Read(&hi2c1, 0xD0, ADDRESS, I2C_MEMADD_SIZE_8BIT, &Data, 1, HAL_MAX_DELAY);
	return Data;
}

void MPU6050_Write(uint8_t ADDRESS, uint8_t Data)
{
	HAL_I2C_Mem_Write(&hi2c1, 0xD0, ADDRESS, I2C_MEMADD_SIZE_8BIT, &Data, 1, HAL_MAX_DELAY);
}

void MPU6050_GetData(SensorData *Sensor)
{
	int16_t High, Low; // 分别接收高位和低位
	High = MPU6050_Read(MPU6050_ACCEL_XOUT_H) << 8;
	Low = MPU6050_Read(MPU6050_ACCEL_XOUT_L);
	Sensor->ACCEL_X = High | Low;
	
	High = MPU6050_Read(MPU6050_ACCEL_YOUT_H) << 8;
	Low = MPU6050_Read(MPU6050_ACCEL_YOUT_L);
	Sensor->ACCEL_Y = High | Low;
	
	High = MPU6050_Read(MPU6050_ACCEL_ZOUT_H) << 8;
	Low = MPU6050_Read(MPU6050_ACCEL_ZOUT_L);
	Sensor->ACCEL_Z = High | Low;
	
	High = MPU6050_Read(MPU6050_GYRO_XOUT_H) << 8;
	Low = MPU6050_Read(MPU6050_GYRO_XOUT_L);
	Sensor->GYRO_X = High | Low;
	
	High = MPU6050_Read(MPU6050_GYRO_YOUT_H) << 8;
	Low = MPU6050_Read(MPU6050_GYRO_YOUT_L);
	Sensor->GYRO_Y = High | Low;
	
	High = MPU6050_Read(MPU6050_GYRO_ZOUT_H) << 8;
	Low = MPU6050_Read(MPU6050_GYRO_ZOUT_L);
	Sensor->GYRO_Z = High | Low;
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
	MPU6050_Write(MPU6050_PWR_MGMT_1, 0x01);   // 配置电源1并使用陀螺仪时钟
	MPU6050_Write(MPU6050_PWR_MGMT_2, 0x00);   // 配置电源2并唤醒
	MPU6050_Write(MPU6050_SMPLRT_DIV, 0x09);   // 分频，可随意配
	MPU6050_Write(MPU6050_CONFIG, 0x06);       // 滤波
	MPU6050_Write(MPU6050_GYRO_CONFIG, 0x18);  // 不自测，最大量程
	MPU6050_Write(MPU6050_ACCEL_CONFIG, 0x18); // 不自测，最大量程
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
	MX_I2C1_Init();
	/* USER CODE BEGIN 2 */
	SensorData Sensor;
	/* USER CODE END 2 */
	
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		MPU6050_GetData(&Sensor);
		OLED_ShowSignedNum(1, 1, Sensor.ACCEL_X, 4);
		OLED_ShowSignedNum(2, 1, Sensor.ACCEL_Y, 4);
		OLED_ShowSignedNum(3, 1, Sensor.ACCEL_Z, 4);
		OLED_ShowSignedNum(1, 8, Sensor.GYRO_X, 4);
		OLED_ShowSignedNum(2, 8, Sensor.GYRO_Y, 4);
		OLED_ShowSignedNum(3, 8, Sensor.GYRO_Z, 4);
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
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
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
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{
	
	/* USER CODE BEGIN I2C1_Init 0 */
	
	/* USER CODE END I2C1_Init 0 */
	
	/* USER CODE BEGIN I2C1_Init 1 */
	
	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */
	
	/* USER CODE END I2C1_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	
	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
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

