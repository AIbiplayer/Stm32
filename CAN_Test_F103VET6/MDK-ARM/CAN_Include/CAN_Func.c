/**
 * @brief CAN测试
 * @date 2025/7/26
 */

/**
 * 由于高速CAN最高速率也只有1兆，所以CubeMX配置分频使速度不大于1兆
 *
 * 以下define的内容需要位移是根据后面的Object_Para来设置的，就是把这些参数整合到一个变量然后控制
 */

#include "main.h"

#define CAN_FILTER(x) ((x) << 3) // 过滤器配置，F103系列只有一个CAN接口，14个过滤器，范围0~13

#define CAN_FIFO_0 (0 << 2) // FIFO 0模式
#define CAN_FIFO_1 (1 << 2) // FIFO 1模式

#define CANSTDID (0 << 1) // 标准ID
#define CANEXTID (1 << 1) // 扩展ID

#define CANRTR_DATA (0 << 0)   // 数据帧
#define CANRTR_REMOTE (1 << 0) // 遥控帧

/**
 * @brief CAN初始化
 */
void Can_Init(CAN_HandleTypeDef *hcan)
{
    HAL_CAN_Start(hcan);                                    // 启动CAN
    __HAL_CAN_ENABLE_IT(hcan, CAN_IT_RX_FIFO0_MSG_PENDING); // 使能接收中断
    __HAL_CAN_ENABLE_IT(hcan, CAN_IT_RX_FIFO1_MSG_PENDING); // 使能接收中断
}

/**
 * @brief CAN发送数据
 * @param hcan CAN句柄
 * @param ID 发送的ID
 * @param Data 发送的数据指针
 * @param Length 数据长度
 * @return 状态
 */
HAL_StatusTypeDef Can_SendData(CAN_HandleTypeDef *hcan, uint16_t ID, uint8_t *Data, uint16_t Length)
{
    CAN_TxHeaderTypeDef TxHeader; // 发送头部结构体
    uint32_t TxMailbox;           // 邮箱

    TxHeader.StdId = ID;   // 设置标准ID
    TxHeader.ExtId = 0;    // 扩展ID为0，不扩展
    TxHeader.IDE = 0;      // 设置为标准帧，1表示扩展帧
    TxHeader.RTR = 0;      // 设置为数据帧，1表示远程帧
    TxHeader.DLC = Length; // 数据长度

    return HAL_CAN_AddTxMessage(hcan, &TxHeader, Data, &TxMailbox); // 发送CAN消息
}
/**
 * @brief CAN过滤器配置
 * @param hcan CAN句柄
 * @param Object_Para 过滤器参数，包含过滤器编号、FIFO选择、ID类型和帧类型
 * @param ID 过滤器ID，标准帧为11位，扩展帧为29位
 * @param Mask_ID 过滤器掩码ID
 */
void Can_Filter_Config(CAN_HandleTypeDef *hcan, uint8_t Object_Para, uint32_t ID, uint32_t Mask_ID)
{
    CAN_FilterTypeDef Can_Filter; // 过滤器结构体

    if ((Object_Para & 0x02)) // 扩展帧，即ID为29位，& 0x02 的原因是define中设置了第二位判断标准或扩展
    {
        // 设置过滤器ID高位，由于是29位，29位左移3位代表左对齐，再右移16位表示取高位，可以画图感受下
        Can_Filter.FilterIdHigh = ID << 3 >> 16;

        /*
         * 左移3位的原因和上面一样，(Object_Para & 0x03)是把（标准/扩展）位和（标准/遥控）位单独提出
         * 左移1位并或上ID的原因是32位过滤器规定了这两位必须在第1,2位
         * 这些内容在手册的过滤器匹配序号中可以找到
         */
        Can_Filter.FilterIdLow = ID << 3 | ((Object_Para & 0x03) << 1);

        Can_Filter.FilterMaskIdHigh = Mask_ID << 3 >> 16;                        // 左右移的原因同上
        Can_Filter.FilterMaskIdLow = Mask_ID << 3 | ((Object_Para & 0x03) << 1); // 左右移的原因同上
    }
    else // 标准帧，即ID为11位
    {
        Can_Filter.FilterIdHigh = ID << 5;                        // 11位左移5位就是左对齐
        Can_Filter.FilterIdLow = ((Object_Para & 0x03) << 1);     // 位移的原因同扩展帧
        Can_Filter.FilterMaskIdHigh = Mask_ID << 5;               // 位移的原因同扩展帧
        Can_Filter.FilterMaskIdLow = ((Object_Para & 0x03) << 1); // 位移的原因同扩展帧
    }

    Can_Filter.FilterBank = Object_Para >> 3;                    // 右移3位提出过滤器编号
    Can_Filter.FilterFIFOAssignment = (Object_Para >> 2) & 0x01; // 提取出F0/F1设置位
    Can_Filter.FilterActivation = ENABLE;                        // 使能过滤器
    Can_Filter.FilterMode = CAN_FILTERMODE_IDMASK;               // 掩码设置
    Can_Filter.FilterScale = CAN_FILTERSCALE_32BIT;              // 32位过滤器
    Can_Filter.SlaveStartFilterBank = 14;                        // 单个CAN而言，没什么意义
    HAL_CAN_ConfigFilter(hcan, &Can_Filter);                     // 配置
}

/**
 * @brief 接收回调函数
 * @param hcan 句柄
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef CAN_Rx; // 接收句柄
    uint8_t Data;               // 数据

    HAL_CAN_GetRxMessage(hcan, CAN_FIFO_0, &CAN_Rx, &Data); // 接收数据
}
