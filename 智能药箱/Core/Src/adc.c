/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    adc.c
 * @brief   This file provides code for the configuration
 *          of the ADC instances.
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
#include "adc.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

ADC_HandleTypeDef hadc4;
DMA_NodeTypeDef Node_GPDMA1_Channel5;
DMA_QListTypeDef List_GPDMA1_Channel5;
DMA_HandleTypeDef handle_GPDMA1_Channel5;
ADC_AnalogWDGConfTypeDef AnalogWDGConfig = {0};

/* ADC4 init function */
void MX_ADC4_Init(void)
{

    /* USER CODE BEGIN ADC4_Init 0 */

    /* USER CODE END ADC4_Init 0 */

    ADC_ChannelConfTypeDef sConfig = {0};

    /* USER CODE BEGIN ADC4_Init 1 */

    /* USER CODE END ADC4_Init 1 */

    /** Common config
     */
    hadc4.Instance = ADC4;
    hadc4.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV8;
    hadc4.Init.Resolution = ADC_RESOLUTION_12B;
    hadc4.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc4.Init.ScanConvMode = ADC4_SCAN_DISABLE;
    hadc4.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc4.Init.LowPowerAutoPowerOff = ADC_LOW_POWER_NONE;
    hadc4.Init.LowPowerAutoWait = DISABLE;
    hadc4.Init.ContinuousConvMode = ENABLE;
    hadc4.Init.NbrOfConversion = 1;
    hadc4.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc4.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc4.Init.DMAContinuousRequests = ENABLE;
    hadc4.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_LOW;
    hadc4.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
    hadc4.Init.SamplingTimeCommon1 = ADC4_SAMPLETIME_814CYCLES_5;
    hadc4.Init.SamplingTimeCommon2 = ADC4_SAMPLETIME_814CYCLES_5;
    hadc4.Init.OversamplingMode = DISABLE;
    if (HAL_ADC_Init(&hadc4) != HAL_OK)
    {
        Error_Handler();
    }

    /** Configure Analog WatchDog 1
     */
    AnalogWDGConfig.WatchdogNumber = ADC_ANALOGWATCHDOG_1;
    AnalogWDGConfig.WatchdogMode = ADC_ANALOGWATCHDOG_SINGLE_REG;
    AnalogWDGConfig.Channel = ADC_CHANNEL_1;
    AnalogWDGConfig.ITMode = ENABLE;
    AnalogWDGConfig.HighThreshold = 3600;
    AnalogWDGConfig.LowThreshold = 3200;
    if (HAL_ADC_AnalogWDGConfig(&hadc4, &AnalogWDGConfig) != HAL_OK)
    {
        Error_Handler();
    }

    /** Configure Regular Channel
     */
    sConfig.Channel = ADC_CHANNEL_1;
    sConfig.Rank = ADC4_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC4_SAMPLINGTIME_COMMON_1;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    if (HAL_ADC_ConfigChannel(&hadc4, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN ADC4_Init 2 */

    /* USER CODE END ADC4_Init 2 */
}

void HAL_ADC_MspInit(ADC_HandleTypeDef *adcHandle)
{

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    DMA_NodeConfTypeDef NodeConfig = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
    if (adcHandle->Instance == ADC4)
    {
        /* USER CODE BEGIN ADC4_MspInit 0 */

        /* USER CODE END ADC4_MspInit 0 */

        /** Initializes the peripherals clock
         */
        PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADCDAC;
        PeriphClkInit.AdcDacClockSelection = RCC_ADCDACCLKSOURCE_HSI;
        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
        {
            Error_Handler();
        }

        /* ADC4 clock enable */
        __HAL_RCC_ADC4_CLK_ENABLE();

        __HAL_RCC_GPIOC_CLK_ENABLE();
        /**ADC4 GPIO Configuration
        PC0     ------> ADC4_IN1
        */
        GPIO_InitStruct.Pin = GPIO_PIN_0;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        /* ADC4 DMA Init */
        /* GPDMA1_REQUEST_ADC4 Init */
        NodeConfig.NodeType = DMA_GPDMA_LINEAR_NODE;
        NodeConfig.Init.Request = GPDMA1_REQUEST_ADC4;
        NodeConfig.Init.BlkHWRequest = DMA_BREQ_SINGLE_BURST;
        NodeConfig.Init.Direction = DMA_PERIPH_TO_MEMORY;
        NodeConfig.Init.SrcInc = DMA_SINC_FIXED;
        NodeConfig.Init.DestInc = DMA_DINC_FIXED;
        NodeConfig.Init.SrcDataWidth = DMA_SRC_DATAWIDTH_WORD;
        NodeConfig.Init.DestDataWidth = DMA_DEST_DATAWIDTH_WORD;
        NodeConfig.Init.SrcBurstLength = 1;
        NodeConfig.Init.DestBurstLength = 1;
        NodeConfig.Init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT0 | DMA_DEST_ALLOCATED_PORT0;
        NodeConfig.Init.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
        NodeConfig.Init.Mode = DMA_NORMAL;
        NodeConfig.TriggerConfig.TriggerPolarity = DMA_TRIG_POLARITY_MASKED;
        NodeConfig.DataHandlingConfig.DataExchange = DMA_EXCHANGE_NONE;
        NodeConfig.DataHandlingConfig.DataAlignment = DMA_DATA_RIGHTALIGN_ZEROPADDED;
        if (HAL_DMAEx_List_BuildNode(&NodeConfig, &Node_GPDMA1_Channel5) != HAL_OK)
        {
            Error_Handler();
        }

        if (HAL_DMAEx_List_InsertNode(&List_GPDMA1_Channel5, NULL, &Node_GPDMA1_Channel5) != HAL_OK)
        {
            Error_Handler();
        }

        if (HAL_DMAEx_List_SetCircularMode(&List_GPDMA1_Channel5) != HAL_OK)
        {
            Error_Handler();
        }

        handle_GPDMA1_Channel5.Instance = GPDMA1_Channel5;
        handle_GPDMA1_Channel5.InitLinkedList.Priority = DMA_LOW_PRIORITY_LOW_WEIGHT;
        handle_GPDMA1_Channel5.InitLinkedList.LinkStepMode = DMA_LSM_FULL_EXECUTION;
        handle_GPDMA1_Channel5.InitLinkedList.LinkAllocatedPort = DMA_LINK_ALLOCATED_PORT0;
        handle_GPDMA1_Channel5.InitLinkedList.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
        handle_GPDMA1_Channel5.InitLinkedList.LinkedListMode = DMA_LINKEDLIST_CIRCULAR;
        if (HAL_DMAEx_List_Init(&handle_GPDMA1_Channel5) != HAL_OK)
        {
            Error_Handler();
        }

        if (HAL_DMAEx_List_LinkQ(&handle_GPDMA1_Channel5, &List_GPDMA1_Channel5) != HAL_OK)
        {
            Error_Handler();
        }

        __HAL_LINKDMA(adcHandle, DMA_Handle, handle_GPDMA1_Channel5);

        if (HAL_DMA_ConfigChannelAttributes(&handle_GPDMA1_Channel5, DMA_CHANNEL_NPRIV) != HAL_OK)
        {
            Error_Handler();
        }

        /* ADC4 interrupt Init */
        HAL_NVIC_SetPriority(ADC4_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(ADC4_IRQn);
        /* USER CODE BEGIN ADC4_MspInit 1 */

        /* USER CODE END ADC4_MspInit 1 */
    }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef *adcHandle)
{

    if (adcHandle->Instance == ADC4)
    {
        /* USER CODE BEGIN ADC4_MspDeInit 0 */

        /* USER CODE END ADC4_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_ADC4_CLK_DISABLE();

        /**ADC4 GPIO Configuration
        PC0     ------> ADC4_IN1
        */
        HAL_GPIO_DeInit(GPIOC, GPIO_PIN_0);

        /* ADC4 DMA DeInit */
        HAL_DMA_DeInit(adcHandle->DMA_Handle);

        /* ADC4 interrupt Deinit */
        HAL_NVIC_DisableIRQ(ADC4_IRQn);
        /* USER CODE BEGIN ADC4_MspDeInit 1 */

        /* USER CODE END ADC4_MspDeInit 1 */
    }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
