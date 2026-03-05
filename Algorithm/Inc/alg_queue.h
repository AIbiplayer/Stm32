#ifndef __QUEUE_H__
#define __QUEUE_H__

#include <stdint.h>
#include <stdbool.h>

// 定义队列最大容量
#define QUEUE_MAX_CAPACITY 100 

// 定义数据类型，这里配合电流检测设为 float
typedef float Queue_Elem_t;

typedef struct {
    Queue_Elem_t buffer[QUEUE_MAX_CAPACITY]; // 数据存储区
    uint16_t front;  // 头指针 (弹出位置)
    uint16_t rear;   // 尾指针 (推入位置)
    uint16_t size;   // 当前元素个数
} Queue_t;

// 函数声明
void Queue_Init(Queue_t *q);
bool Queue_Push(Queue_t *q, Queue_Elem_t val);
bool Queue_Pop(Queue_t *q, Queue_Elem_t *val);
Queue_Elem_t Queue_Peek(Queue_t *q);
void Queue_Clear(Queue_t *q);
bool Queue_Is_Full(Queue_t *q);
bool Queue_Is_Empty(Queue_t *q);
uint16_t Queue_Get_Length(Queue_t *q);

// 针对你上一个问题的特殊功能：计算队列内总和
float Queue_Get_Sum(Queue_t *q);

#endif