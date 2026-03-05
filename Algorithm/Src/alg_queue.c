#include "alg_queue.h"
#include <string.h> // for memset

/**
 * @brief 初始化队列
 */
void Queue_Init(Queue_t *q) {
    q->front = 0;
    q->rear = 0;
    q->size = 0;
    // 清空内存
    memset(q->buffer, 0, sizeof(q->buffer));
}

/**
 * @brief 入队 (Push)
 * @return true 成功, false 失败(满了)
 */
bool Queue_Push(Queue_t *q, Queue_Elem_t val) {
    if (Queue_Is_Full(q)) {
        return false; // 队列已满
    }

    q->buffer[q->rear] = val;
    // 循环指针逻辑：如果到了数组末尾，回到0
    q->rear = (q->rear + 1) % QUEUE_MAX_CAPACITY;
    q->size++;
    return true;
}

/**
 * @brief 出队 (Pop)
 * @param val [out] 弹出的值的容器指针
 * @return true 成功, false 失败(空的)
 */
bool Queue_Pop(Queue_t *q, Queue_Elem_t *val) {
    if (Queue_Is_Empty(q)) {
        return false; // 队列为空
    }

    if (val != NULL) {
        *val = q->buffer[q->front];
    }
    
    // 循环指针逻辑
    q->front = (q->front + 1) % QUEUE_MAX_CAPACITY;
    q->size--;
    return true;
}

/**
 * @brief 查看队首元素 (不弹出)
 */
Queue_Elem_t Queue_Peek(Queue_t *q) {
    if (Queue_Is_Empty(q)) {
        return 0.0f; // 错误处理：返回0或特定错误码
    }
    return q->buffer[q->front];
}

/**
 * @brief 清空队列
 */
void Queue_Clear(Queue_t *q) {
    q->front = 0;
    q->rear = 0;
    q->size = 0;
}

/**
 * @brief 判断满
 */
bool Queue_Is_Full(Queue_t *q) {
    return (q->size >= QUEUE_MAX_CAPACITY);
}

/**
 * @brief 判断空
 */
bool Queue_Is_Empty(Queue_t *q) {
    return (q->size == 0);
}

/**
 * @brief 获取当前长度
 */
uint16_t Queue_Get_Length(Queue_t *q) {
    return q->size;
}

/**
 * @brief 计算队列总和 (用于热量检测积分)
 * @note 此函数会遍历队列，时间复杂度为 O(N)
 */
float Queue_Get_Sum(Queue_t *q) {
    float sum = 0.0f;
    uint16_t idx = q->front;
    
    for (uint16_t i = 0; i < q->size; i++) {
        sum += q->buffer[idx];
        idx = (idx + 1) % QUEUE_MAX_CAPACITY;
    }
    return sum;
}