/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "HC_SR04.h"
#include "inv_mpu.h"
#include "mpu6050.h"
#include "OLED.h"
#include "stdbool.h"
#include "stdio.h"
#include "stdarg.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern float Distance;
extern bool GetDistance_Status;

uint8_t Receive_Buffer[128]; //接收缓存
float Pitch, Roll, Yaw;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId BalanceHandle;
osThreadId BlueToothHandle;
osThreadId HC_SR04Handle;
osThreadId OLEDHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/**
 * @brief 串口打印函数
 * @param huart 串口
 * @param format 字符串
 */
void Uart_printf(UART_HandleTypeDef* huart, char* format, ...) // 串口打印函数
{
    static char buf[128]; // 定义临时数组，根据实际发送大小
    va_list args;
    va_start(args, format);
    uint32_t len = vsnprintf((char*)buf, sizeof(buf), (char*)format, args);
    va_end(args);
    HAL_UART_Transmit_DMA(huart, (uint8_t*)buf, len);
}

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void Balance_Task(void const * argument);
void BlueTooth_Task(void const * argument);
void HC_SR04_Task(void const * argument);
void OLED_Task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t** ppxIdleTaskTCBBuffer, StackType_t** ppxIdleTaskStackBuffer,
                                   uint32_t* pulIdleTaskStackSize)
{
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
    *ppxIdleTaskStackBuffer = &xIdleStack[0];
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
    /* place for user code */
}

/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of Balance */
  osThreadDef(Balance, Balance_Task, osPriorityHigh, 0, 256);
  BalanceHandle = osThreadCreate(osThread(Balance), NULL);

  /* definition and creation of BlueTooth */
  osThreadDef(BlueTooth, BlueTooth_Task, osPriorityAboveNormal, 0, 128);
  BlueToothHandle = osThreadCreate(osThread(BlueTooth), NULL);

  /* definition and creation of HC_SR04 */
  osThreadDef(HC_SR04, HC_SR04_Task, osPriorityNormal, 0, 128);
  HC_SR04Handle = osThreadCreate(osThread(HC_SR04), NULL);

  /* definition and creation of OLED */
  osThreadDef(OLED, OLED_Task, osPriorityBelowNormal, 0, 128);
  OLEDHandle = osThreadCreate(osThread(OLED), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */

    OLED_Init();
    MPU_Init();
    mpu_dmp_init();
    Delay_Us_Init();
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);

    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, Receive_Buffer, sizeof(Receive_Buffer));
    __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);

    /* Infinite loop */
    for (;;)
    {
        osDelay(1);
    }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_Balance_Task */
/**
* @brief Function implementing the Balance thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Balance_Task */
void Balance_Task(void const * argument)
{
  /* USER CODE BEGIN Balance_Task */

    TickType_t xLastWakeTime;

    /* Infinite loop */
    for (;;)
    {
        mpu_dmp_get_data(&Pitch, &Roll, &Yaw);

        vTaskDelayUntil(&xLastWakeTime, 5);
        osDelay(1);
    }
  /* USER CODE END Balance_Task */
}

/* USER CODE BEGIN Header_BlueTooth_Task */
/**
* @brief Function implementing the BlueTooth thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_BlueTooth_Task */
void BlueTooth_Task(void const * argument)
{
  /* USER CODE BEGIN BlueTooth_Task */
    /* Infinite loop */
    for (;;)
    {
        osDelay(1);
    }
  /* USER CODE END BlueTooth_Task */
}

/* USER CODE BEGIN Header_HC_SR04_Task */
/**
* @brief Function implementing the HC_SR04 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_HC_SR04_Task */
void HC_SR04_Task(void const * argument)
{
  /* USER CODE BEGIN HC_SR04_Task */
    TickType_t xLastWakeTime;
    /* Infinite loop */
    for (;;)
    {
        HC_SR04_Send_Trig();
        vTaskDelayUntil(&xLastWakeTime, 50);

        osDelay(1);
    }
  /* USER CODE END HC_SR04_Task */
}

/* USER CODE BEGIN Header_OLED_Task */
/**
* @brief Function implementing the OLED thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_OLED_Task */
void OLED_Task(void const * argument)
{
  /* USER CODE BEGIN OLED_Task */

    TickType_t xLastWakeTime;

    /* Infinite loop */
    for (;;)
    {
        OLED_ShowString(1, 1, "Distance:");
        OLED_ShowNum(1, 10, (uint16_t)Distance, 2);
        OLED_ShowString(1, 12, "cm");
        OLED_ShowString(2, 1, "Pitch:");
        OLED_ShowSignedNum(2, 7, (int16_t)Pitch, 2);
        OLED_ShowString(3, 1, "Yaw:");
        OLED_ShowSignedNum(3, 5, (int16_t)Yaw, 2);
        vTaskDelayUntil(&xLastWakeTime, 50);
        osDelay(1);
    }
  /* USER CODE END OLED_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/**
 * @brief 蓝牙接收回调函数
 * @param huart 串口
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
    if (huart->Instance == USART3)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, Receive_Buffer, sizeof(Receive_Buffer));
        __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
    }
}

/* USER CODE END Application */

