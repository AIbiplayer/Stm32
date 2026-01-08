#include "bsp_usb.h"
#include "main.h"
#include <string.h>

CCMRAM USB_Control_t g_usb_dev; // 全局USB设备实例

/**
 * @brief USB 发送封装
 * @param buf: 待发送数据首地址
 * @param len: 发送长度
 * @return 0-成功, 其他-失败
 */
uint8_t bsp_usb_transmit(uint8_t* buf, uint16_t len)
{
    uint8_t result = CDC_Transmit_FS(buf, len);
    return result;
}

/**
 * @brief USB 接收回调底层映射 (在 usbd_cdc_if.c 中被调用)
 */
void bsp_usb_receive_callback(uint8_t* buf, uint32_t len)
{
    // 简单处理：将数据拷贝至全局缓冲区
    if (len < USB_RX_BUF_SIZE)
    {
        LED_Green_Down;
        memcpy(g_usb_dev.rx_buffer, buf, len);
        g_usb_dev.rx_len = len;
        g_usb_dev.rx_callback();
        g_usb_dev.rx_flag = 1; // 触发应用层逻辑
    }

    // 重点：必须继续准备下一次接收，否则 USB 传输会停止
    // CubeMX 生成的代码通常在 usbd_cdc_if.c 中自动处理了重新设置接收地址的动作
}
