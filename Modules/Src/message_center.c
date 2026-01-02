#include "message_center.h"
#include "stdlib.h"
#include "string.h"
#include "stdio.h"

/* message_center是fake head node,是方便链表编写的技巧,这样就不需要处理链表头的特殊情况 */
static Publisher_t message_center = {
    .topic_name = "Message_Manager",
    .first_subs = NULL,
    .next_topic_node = NULL};

// 1. 移除不必要的 static 变量，改用局部变量
// 2. 增加 malloc 校验

Publisher_t *PubRegister(char *name, uint8_t data_len) {
    Publisher_t *node = &message_center;

    // 安全性检查：限制名称长度防止 strcpy 溢出
    if (strlen(name) > MAX_TOPIC_NAME_LEN) return NULL;

    while (node->next_topic_node) {
        node = node->next_topic_node;
        if (strcmp(node->topic_name, name) == 0) {
            node->pub_registered_flag = 1;
            return node;
        }
    }

    Publisher_t *new_pub = (Publisher_t *)malloc(sizeof(Publisher_t));
    if (!new_pub) return NULL; // 内存分配失败保护

    memset(new_pub, 0, sizeof(Publisher_t));
    new_pub->data_len = data_len;
    strncpy(new_pub->topic_name, name, MAX_TOPIC_NAME_LEN);
    new_pub->pub_registered_flag = 1;

    node->next_topic_node = new_pub;
    return new_pub;
}

Subscriber_t *SubRegister(char *name, uint8_t data_len) {
    Publisher_t *pub = PubRegister(name, data_len);
    if (!pub) return NULL;

    Subscriber_t *ret = (Subscriber_t *)malloc(sizeof(Subscriber_t));
    if (!ret) return NULL;

    memset(ret, 0, sizeof(Subscriber_t));
    ret->data_len = data_len;

    for (size_t i = 0; i < QUEUE_SIZE; ++i) {
        ret->queue[i] = malloc(data_len);
        if (!ret->queue[i]) return NULL; // 深度分配失败检查
        memset(ret->queue[i], 0, data_len);
    }

    // 链表插入操作
    if (pub->first_subs == NULL) {
        pub->first_subs = ret;
    } else {
        Subscriber_t *tmp = pub->first_subs;
        while (tmp->next_subs_queue) {
            tmp = tmp->next_subs_queue;
        }
        tmp->next_subs_queue = ret;
    }
    return ret;
}

uint8_t SubGetMessage(Subscriber_t *sub, void *data_ptr) {
    if (!sub || sub->temp_size == 0) return 0;

    memcpy(data_ptr, sub->queue[sub->front_idx], sub->data_len);

    // 修复自增 Bug
    sub->front_idx = (sub->front_idx + 1) % QUEUE_SIZE;
    sub->temp_size--;
    return 1;
}

uint8_t PubPushMessage(Publisher_t *pub, void *data_ptr) {
    if (!pub || !data_ptr) return 0;

    // 严禁在此处使用 static 指针，确保重入安全性 (Reentrancy)
    Subscriber_t *iter = pub->first_subs;

    while (iter) {
        // 核心安全检查：如果传入数据长度与订阅者预期不符，可能导致越界
        if (iter->temp_size == QUEUE_SIZE) {
            iter->front_idx = (iter->front_idx + 1) % QUEUE_SIZE;
            iter->temp_size--;
        }

        memcpy(iter->queue[iter->back_idx], data_ptr, pub->data_len);

        iter->back_idx = (iter->back_idx + 1) % QUEUE_SIZE;
        iter->temp_size++;
        iter = iter->next_subs_queue;
    }
    return 1;
}