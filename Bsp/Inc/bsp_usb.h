#ifndef __BSP_USB_H
#define __BSP_USB_H

#include "stm32f4xx_hal.h"
#include "usbd_cdc_if.h"

// 定义接收缓冲区大小
#define USB_RX_BUF_SIZE 64

typedef struct
{
    uint8_t rx_buffer[USB_RX_BUF_SIZE]; // 接收缓存
    uint16_t rx_len; // 接收长度
    uint8_t rx_flag; // 接收完成标志位
    void (* rx_callback)(uint16_t len); // 接收回调函数指针
} USB_Control_t;

uint8_t bsp_usb_transmit(uint8_t* buf, uint16_t len);
void bsp_usb_receive_callback(uint8_t* buf, uint32_t len);

#endif
