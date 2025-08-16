/**
 * @file MCP2515.c
 * @brief MCP2515驱动，波特率125Kbps
 * @author 申飞麟
 * @date 2025/8/5
 */

#include "main.h"
#include "SPI_Soft.h"

/**
 * @brief 复位指令
 * @note 会进入配置模式，无法传输数据
 */
void MCP2515_Reset(void)
{
    CS(0);
    SPI_Soft_Swap(0xC0); // 复位指令
    CS(1);
}

/**
 * @brief 读取寄存器数据
 * @param Address 寄存器地址
 * @param ReceiveData 接收的数组
 * @param Length 数组长度
 * @note MCP2515支持寄存器连续读取，读取完寄存器后内部指针自动指向下一个寄存器
 */
void MCP2515_Read_Register_Data(uint8_t Address, uint8_t *ReceiveData, uint8_t Length)
{
    CS(0);

    SPI_Soft_Swap(0x03);
    SPI_Soft_Swap(Address);
    for (uint8_t i = 0; i < Length; i++)
    {
        ReceiveData[i] = SPI_Soft_Swap(0xFF);
    }

    CS(1);
}

/**
 * @brief 向寄存器写入数据
 * @param Address 寄存器地址
 * @param WriteData 发送的数组
 * @param Length 数组长度
 * @note MCP2515支持寄存器连续写入，读取完寄存器后内部指针自动指向下一个寄存器
 */
void MCP2515_Write_Register_Data(uint8_t Address, uint8_t *WriteData, uint8_t Length)
{
    CS(0);

    SPI_Soft_Swap(0x02);
    SPI_Soft_Swap(Address);

    for (uint8_t i = 0; i < Length; i++)
    {
        SPI_Soft_Swap(WriteData[i]);
    }

    CS(1);
}

/**
 * @brief 设置MCP2515工作模式为正常模式
 */
void MCP2515_Set_Normal_Mode(void)
{
    CS(0);

    SPI_Soft_Swap(0x05);
    SPI_Soft_Swap(0xFF);
    SPI_Soft_Swap(0xE0);
    SPI_Soft_Swap(0x00);

    CS(1);
}

/**
 * @brief 设置MCP2515工作模式为环回模式
 */
void MCP2515_Set_Loopback_Mode(void)
{
    CS(0);

    SPI_Soft_Swap(0x05);
    SPI_Soft_Swap(0xFF);
    SPI_Soft_Swap(0xE0);
    SPI_Soft_Swap(0x40);

    CS(1);
}

/**
 * @brief 初始化MCP2515
 * @note 包括复位、配置波特率、配置滤波器等
 */
void MCP2515_Init(void)
{
    SPI_Soft_Init();
    uint8_t ConfigData[3] = {0x02, 0x90, 0x03};
    MCP2515_Reset();
    HAL_Delay(10);
    MCP2515_Write_Register_Data(0x28, ConfigData, 3);

    CS(0);
    SPI_Soft_Swap(0x05);
    SPI_Soft_Swap(0x60);
    SPI_Soft_Swap(0x60);
    SPI_Soft_Swap(0x60);
    CS(1);

    MCP2515_Set_Normal_Mode();
}

/**
 * @brief 读取接收状态寄存器，判断是否收到数据
 * @return RX1收到返回1，RX0收到返回0，未收到返回0xFF
 */
uint8_t MCP2515_Read_RX_Status(void)
{
    uint8_t Status = 0xFF;

    CS(0);

    SPI_Soft_Swap(0xA0);
    Status = SPI_Soft_Swap(0xFF) & 0x03; // 只保留RX1和RX0位

    CS(1);

    if (Status & 0x01)
    {
        Status = 0; // RX0收到数据
    }
    else if (Status & 0x02)
    {
        Status = 1; // RX1收到数据
    }
    else
    {
        Status = 0xFF; // 未收到数据
    }
    return Status;
}

/**
 * @brief 读取接收缓冲区数据
 * @param ReceiveData 接收的数组
 * @param Length 数组长度
 * @note MCP2515支持接收缓冲区连续读取，读取完寄存器后内部指针自动指向下一个寄存器
 */
void MCP2515_Read_Rx_Buffer(uint8_t *ReceiveData, uint8_t Length)
{
    switch (MCP2515_Read_RX_Status())
    {
    case 0:
        CS(0);
        SPI_Soft_Swap(0x92);
        break;
    case 1:
        CS(0);
        SPI_Soft_Swap(0x96);
        break;
    default:
        return; 
    }
    for (uint8_t i = 0; i < Length; i++)
    {
        ReceiveData[i] = SPI_Soft_Swap(0xFF);
    }
    CS(1);
}

/**
 * @brief 向发送缓冲区发送数据
 * @param ID 消息ID
 * @param SendData 发送的数组
 * @param Length 数组长度
 */
void MCP2515_Send_Tx_Buffer(uint16_t ID, uint8_t *SendData, uint8_t Length)
{
    CS(0);

    SPI_Soft_Swap(0x50); // 清除缓冲区
    SPI_Soft_Swap(0x30);
    SPI_Soft_Swap(0x0B);
    SPI_Soft_Swap(0x03);

    CS(1);

    uint8_t ID_H_L[2] = {0};
    ID_H_L[0] = ((ID << 5) & 0xFF00) >> 8; // ID高8位
    ID_H_L[1] = (ID << 5) & 0x00FF;        // ID低5位

    MCP2515_Write_Register_Data(0x31, ID_H_L, 2);  // 写入ID
    MCP2515_Write_Register_Data(0x35, &Length, 1); // 写入数据长度

    CS(0);

    SPI_Soft_Swap(0x41);
    for (uint8_t i = 0; i < Length; i++)
    {
        SPI_Soft_Swap(SendData[i]);
    }

    CS(1);

    CS(0);
    SPI_Soft_Swap(0x81); // 请求发送
    CS(1);
}
