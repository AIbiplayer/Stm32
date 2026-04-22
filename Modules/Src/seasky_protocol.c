/**
 * @file seasky_protocol.c
 * @brief 重庆理工大学RoBoMatster步兵串口通信协议
 * @version 2.1
 * @date 2026-3-09
 *
 */

#include "seasky_protocol.h"
#include "crc8.h"
#include "crc16.h"
#include "memory.h"

static uint8_t head[100] = {0};
/*获取CRC8校验码*/
uint8_t Get_CRC8_Check(uint8_t *pchMessage, uint16_t dwLength) {
    return crc_8(pchMessage, dwLength);
}

/*检验CRC8数据段*/
// static uint8_t CRC8_Check_Sum(uint8_t *pchMessage, uint16_t dwLength)
// {
//     uint8_t ucExpected = 0;
//     if ((pchMessage == 0) || (dwLength <= 2))
//         return 0;
//     ucExpected = crc_8(pchMessage, dwLength - 1);
//     return (ucExpected == pchMessage[dwLength - 1]);
// }

/*获取CRC16校验码*/
uint16_t Get_CRC16_Check(uint8_t *pchMessage, uint32_t dwLength) {
    return crc_16(pchMessage, dwLength);
}

/*检验CRC16数据段*/
// static uint16_t CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength)
// {
//     uint16_t wExpected = 0;
//     if ((pchMessage == 0) || (dwLength <= 2))
//     {
//         return 0;
//     }
//     wExpected = crc_16(pchMessage, dwLength - 2);
//     return (((wExpected & 0xff) == pchMessage[dwLength - 2]) && (((wExpected >> 8) & 0xff) == pchMessage[dwLength - 1]));
// }

//和校验
static uint8_t Sum_Check_Sum(uint8_t *pchMessage, uint16_t Length) {
    uint8_t sum = 0;
    for (uint16_t i = 0; i < Length; i++) {
        sum += pchMessage[i];
    }
    return sum & 0xFF;
}

/*检验数据帧头*/
static uint8_t protocol_heade_Check(protocol_rm_struct *pro, uint8_t *rx_buf) {
    if (rx_buf[0] == PROTOCOL_CMD_ID) {
        pro->header.sof = rx_buf[0];
        return 1;
    }
    return 0;
}

/*
    此函数根据待发送的数据更新数据帧格式以及内容，实现数据的打包操作
    后续调用通信接口的发送函数发送tx_buf中的对应数据
*/
/**
 *
 * @param tx_data 待发送的数据
 * @param tx_buf 待发送的数据帧，包括包头之类的
 * @param tx_buf_len 待发送的数据帧长度
 */
void get_protocol_send_data(Vision_Send_s *tx_data, // 待发送的数据
                            uint8_t *tx_buf, // 待发送的数据帧
                            uint16_t *tx_buf_len) // 待发送的数据帧长度
{
    static uint16_t data_len;

    /*帧头部分*/
    tx_buf[0] = PROTOCOL_CMD_ID;
    tx_buf[1] = tx_data->cmd_data_type; //数据类型id,比如云台位置数据,开火控制数据等等
    switch (tx_data->cmd_data_type) {
        case TX_GIMBAL_REALTIME:
            memcpy(tx_buf + 2, &tx_data->gimbal_data, sizeof(tx_data->gimbal_data));
            data_len = 2 + sizeof(tx_data->gimbal_data);
            break;
        case TX_GIMBAL_N_REALTIME:
            memcpy(tx_buf + 2, &tx_data->nrt_data, sizeof(tx_data->nrt_data));
            data_len = 2 + sizeof(tx_data->nrt_data);
        default:
            return;
    }
    tx_buf[data_len] = Sum_Check_Sum(tx_buf, data_len); //和校验,校验范围为包头+数据段
    *tx_buf_len = data_len + 1; //数据帧长度为包头+数据段+校验码
}

/*
    此函数用于处理接收数据，
    可以返回数据内容的id
*/
uint16_t get_protocol_info(uint8_t *rx_buf, // 接收到的原始数据
                           Vision_Recv_s *rx_data) // 接收的float数据存储地址
{
    // 放在静态区,避免反复申请栈上空间
    static protocol_rm_struct pro;
    static Rx_Data_type_e command_data;

    memcpy(head, rx_buf, 20);
    if (protocol_heade_Check(&pro, rx_buf)) {
        command_data = rx_buf[1];
        switch (command_data) {
            case RECEIVE_GIMBAL_POSITION:
                rx_data->cmd_data_type = command_data;
                if (rx_buf[2 + sizeof(rx_data->gimbal_data)] ==
                    Sum_Check_Sum(rx_buf, 2 + sizeof(rx_data->gimbal_data))) {
                    memcpy(&rx_data->gimbal_data, rx_buf + 2, sizeof(rx_data->gimbal_data));
                    rx_data->gimbal_data.pitch = rx_data->gimbal_data.pitch * 180 / 3.1415926f;
                    rx_data->gimbal_data.yaw = rx_data->gimbal_data.yaw * 180 / 3.1415926f;
                    get_protocol_info(&rx_buf[3 + sizeof(rx_data->gimbal_data)], rx_data);
                }

                break;
            case RECEIVE_ENEMY_INFORMATION:
                rx_data->cmd_data_type = command_data;
                if (rx_buf[2 + sizeof(rx_data->enemy_data)] == Sum_Check_Sum(
                        rx_buf, 2 + sizeof(rx_data->enemy_data))) {
                    memcpy(&rx_data->enemy_data, rx_buf + 2, sizeof(rx_data->enemy_data));
                    get_protocol_info(&rx_buf[3 + sizeof(rx_data->enemy_data)], rx_data);
                }
                break;
            default:
                // return 0;



        }
    }
    return 0;
}
