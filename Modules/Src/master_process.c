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
#include "master_process.h"
#include "robot_def.h"
#include "bsp_usb.h"
#include "daemon.h"
#include <math.h>
#include <string.h>

static Vision_Recv_s recv_data; // 接收到的数据
static Vision_Send_s send_data; // 待发送的数据
extern USB_Control_t g_usb_dev; // 全局USB设备实例

static DaemonInstance *vision_daemon_instance;
static volatile uint8_t vision_auto_find = 0;

static float ClampFloat(float value, float min_value, float max_value) {
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static float DecodeHeatLimit(uint16_t shooter_heat_limit) {
    return shooter_heat_limit >= 1024u ? (float) shooter_heat_limit / 1024.0f : (float) shooter_heat_limit;
}

void VisionSetAltitude(float yaw, float pitch) {
    send_data.gimbal_data.yaw = yaw * DEGREE_2_RAD;
    send_data.gimbal_data.little_pitch = pitch * DEGREE_2_RAD;
}

void VisionSetControlState(uint8_t auto_find, uint8_t mode) {
    vision_auto_find = auto_find ? 1u : 0u;
    (void) mode;
}

uint8_t VisionIsOnline(void) {
    return vision_daemon_instance != NULL ? DaemonIsOnline(vision_daemon_instance) : 0u;
}

void VisionUpdateRealtimeData(const INS_t *imu, float pitch_up_total_angle, float pitch_down_total_angle) {
    if (imu == NULL) {
        return;
    }

    send_data.gimbal_data.yaw = imu->Yaw * DEGREE_2_RAD;
    send_data.gimbal_data.little_pitch = (pitch_up_total_angle - pitch_down_total_angle) * DEGREE_2_RAD;
    send_data.gimbal_data.big_pitch = (imu->Yaw - pitch_down_total_angle) * DEGREE_2_RAD;

    send_data.gimbal_data.speedx = 0.0f;
    send_data.gimbal_data.speedy = 0.0f;
    send_data.gimbal_data.accelx = 0.0f;
    send_data.gimbal_data.accely = 0.0f;
}

void VisionUpdateNRTData(uint16_t heat, uint16_t shooter_heat_limit, uint8_t robot_color, float bullet_speed) {
    float heat_ratio = 0.0f;
    float actual_heat_limit = DecodeHeatLimit(shooter_heat_limit);

    send_data.nrt_data.my_color = robot_color ? 1u : 0u;
    if (actual_heat_limit > 0.0f) {
        heat_ratio = (float) heat / actual_heat_limit;
    }

    send_data.nrt_data.heat = (uint8_t) ClampFloat(heat_ratio * 10.0f, 0.0f, 15.0f);
    send_data.nrt_data.auto_find = vision_auto_find;
    send_data.nrt_data.mode = 0u;
    send_data.nrt_data.bullet_speed = bullet_speed;
}

#ifdef VISION_USE_UART

#include "bsp_usart.h"

static USARTInstance *vision_usart_instance;

/**
 * @brief 离线回调函数,将在daemon.c中被daemon task调用
 * @attention 由于HAL库的设计问题,串口开启DMA接收之后同时发送有概率出现__HAL_LOCK()导致的死锁,使得无法
 *            进入接收中断.通过daemon判断数据更新,重新调用服务启动函数以解决此问题.
 *
 * @param id vision_usart_instance的地址,此处没用.
 */
static void VisionOfflineCallback(void *id) {
    (void) id;
    USARTServiceInit(vision_usart_instance);
}

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

#include "bsp_usb.h"
static uint8_t *vis_recv_buff;

static void DecodeVision(uint16_t recv_len) {
    (void) recv_len;
    if (vision_daemon_instance != NULL) {
        DaemonReload(vision_daemon_instance);
    }
    get_protocol_info(vis_recv_buff, &recv_data);
}

/* 视觉通信初始化 */
Vision_Recv_s *VisionInit(void) {
    Daemon_Init_Config_s daemon_conf = {
        .reload_count = 500u,
        .init_count = 3000u,
        .callback = NULL,
        .owner_id = NULL
    };

    vision_daemon_instance = DaemonRegister(&daemon_conf);
    vis_recv_buff = g_usb_dev.rx_buffer;
    g_usb_dev.rx_callback = DecodeVision;
    return &recv_data;
}

void VisionSend(void) {
    static uint8_t usb_send_buff[2][VISION_SEND_SIZE];
    static uint8_t usb_send_index = 0u;
    uint16_t gimbal_tx_len = 0u;
    uint16_t nrt_tx_len = 0u;
    uint8_t next_index = usb_send_index ^ 1u;
    uint8_t *tx_buf = usb_send_buff[next_index];

    send_data.cmd_data_type = TX_GIMBAL_REALTIME;
    get_protocol_send_data(&send_data, tx_buf, &gimbal_tx_len);

    send_data.cmd_data_type = TX_GIMBAL_N_REALTIME;
    get_protocol_send_data(&send_data, tx_buf + gimbal_tx_len, &nrt_tx_len);

    if ((uint16_t) (gimbal_tx_len + nrt_tx_len) > VISION_SEND_SIZE) {
        return;
    }

    if (bsp_usb_transmit(tx_buf, (uint16_t) (gimbal_tx_len + nrt_tx_len)) == USBD_OK) {
        usb_send_index = next_index;
    }
}

#endif // VISION_USE_VCP
