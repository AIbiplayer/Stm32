/**
 * @brief CAN测试
 * @date 2025/7/26
 */

#ifndef CAN_FUNC_H
#define CAN_FUNC_H

#include "main.h"

#define CAN_FILTER(x) ((x) << 3) // 过滤器配置，F103系列只有一个CAN接口，14个过滤器，范围0~13

#define CAN_FIFO_0 (0 << 2) // FIFO 0模式
#define CAN_FIFO_1 (1 << 2) // FIFO 1模式

#define CANSTDID (0 << 1) // 标准ID
#define CANEXTID (1 << 1) // 扩展ID

#define CANRTR_DATA (0 << 0)   // 数据帧
#define CANRTR_REMOTE (1 << 0) // 遥控帧

void Can_Init(CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef Can_SendData(CAN_HandleTypeDef *hcan, uint16_t ID, uint8_t *Data, uint16_t Length);
void Can_Filter_Config(CAN_HandleTypeDef *hcan, uint8_t Object_Para, uint32_t ID, uint32_t Mask_ID);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);

#endif
