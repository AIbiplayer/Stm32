/**
 * @brief CAN测试
 * @date 2025/7/26
 */

#ifndef CAN_FUNC_H
#define CAN_FUNC_H

#include "main.h"

void Can_Init(CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef Can_SendData(CAN_HandleTypeDef *hcan, uint16_t ID, uint8_t *Data, uint16_t Length);
void Can_Filter_Config(CAN_HandleTypeDef *hcan, uint8_t Object_Para, uint32_t ID, uint32_t Mask_ID);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);

#endif
