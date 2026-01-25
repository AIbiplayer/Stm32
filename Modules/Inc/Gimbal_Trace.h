#ifndef GIMBAL_TRACE_H_
#define GIMBAL_TRACE_H_

#include <stdint.h>
typedef struct trace
{
    float x_angle;
    float y_angle;
    float x_distence;
    float y_distence;
}trace_t;

void distance_to_angle(trace_t *trace_);
void trace_data_process(uint16_t *buffer);
void auto_tracing(uint8_t channel);
void DMA_Init(void);

#endif
