/**
 * @file master_process.c
 * @author neozng
 * @brief  module for recv&send vision data
 * @version beta
 * @date 2022-11-03
 * @todo 增加对串口调试助手协议的支持,包括vofa和serial debug
 * @copyright Copyright (c) 2022
 *
 */

#include "seasky_protocol.h"
#include "master_process.h"
#include "bsp_usb.h"
#include "robot_def.h"
#include "main.h"

CCMRAM static Vision_Recv_s recv_data; // 接收到的数据
CCMRAM static Vision_Send_s send_data; // 待发送的数据
CCMRAM static uint8_t send_buff[VISION_SEND_SIZE]; // 发送缓冲区

extern USB_Control_t g_usb_dev; // 全局USB设备实例

void VisionSetAltitude(float yaw, float pitch, float roll) {
    send_data.yaw = yaw;
    send_data.pitch = pitch;
    send_data.roll = roll;
}

#ifdef VISION_USE_UART

#include "bsp_usart.h"

static USARTInstance *vision_usart_instance;

/**
 * @brief 接收解包回调函数,将在bsp_usart.c中被usart rx callback调用
 * @todo  1.提高可读性,将get_protocol_info的第四个参数增加一个float类型buffer
 *        2.添加标志位解码
 */
static void DecodeVision()
{
    uint16_t flag_register;
    DaemonReload(vision_daemon_instance); // 喂狗
    get_protocol_info(vision_usart_instance->recv_buff, &flag_register, (uint8_t *)&recv_data.pitch);
    // TODO: code to resolve flag_register;
}

Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle)
{
    USART_Init_Config_s conf;
    conf.module_callback = DecodeVision;
    conf.recv_buff_size = VISION_RECV_SIZE;
    conf.usart_handle = _handle;
    vision_usart_instance = USARTRegister(&conf);

    // 为master process注册daemon,用于判断视觉通信是否离线
    Daemon_Init_Config_s daemon_conf = {
        .callback = VisionOfflineCallback, // 离线时调用的回调函数,会重启串口接收
        .owner_id = vision_usart_instance,
        .reload_count = 10,
    };
    vision_daemon_instance = DaemonRegister(&daemon_conf);

    return &recv_data;
}

/**
 * @brief 发送函数
 *
 * @param send 待发送数据
 *
 */
void VisionSend()
{
    // buff和txlen必须为static,才能保证在函数退出后不被释放,使得DMA正确完成发送
    // 析构后的陷阱需要特别注意!
    static uint16_t flag_register;
    static uint8_t send_buff[VISION_SEND_SIZE];
    static uint16_t tx_len;
    // TODO: code to set flag_register
    flag_register = 30 << 8 | 0b00000001;
    // 将数据转化为seasky协议的数据包
    get_protocol_send_data(0x02, flag_register, &send_data.yaw, 3, send_buff, &tx_len);
    USARTSend(vision_usart_instance, send_buff, tx_len, USART_TRANSFER_DMA); // 和视觉通信使用IT,防止和接收使用的DMA冲突
    // 此处为HAL设计的缺陷,DMASTOP会停止发送和接收,导致再也无法进入接收中断.
    // 也可在发送完成中断中重新启动DMA接收,但较为复杂.因此,此处使用IT发送.
    // 若使用了daemon,则也可以使用DMA发送.
}

#endif // VISION_USE_UART

#ifdef VISION_USE_VCP

static uint8_t *vis_recv_buff;

static void DecodeVision(void) {
    get_protocol_info(vis_recv_buff, (uint8_t *) &recv_data);
}

/* 视觉通信初始化 */
Vision_Recv_s *VisionInit(void) {
    g_usb_dev.rx_callback = DecodeVision;
    vis_recv_buff = g_usb_dev.rx_buffer;
    return &recv_data;
}

void VisionSend(void) {
    static uint16_t tx_len;
    // 将数据转化为seasky协议的数据包
    get_protocol_send_data((float *) &send_data, send_buff, &tx_len);
    bsp_usb_transmit(send_buff, tx_len);
    memset(send_buff, 0, sizeof(send_buff));
}

#endif // VISION_USE_VCP
