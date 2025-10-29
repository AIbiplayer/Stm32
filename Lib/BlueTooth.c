/**
 * @file BlueTooth.c
 * @brief 蓝牙模块驱动
 * @date 2025/10/8
 */

#include "BlueTooth.h"
#include "usart.h"
#include "string.h"
#include "Chassis.h"
#include "stdio.h"
#include "stdlib.h"
#include "stdarg.h"

char U1_Receive_Buffer[RX_BUFF_SIZE] = {0}; // 串口1接收缓冲区
char U2_Receive_Buffer[RX_BUFF_SIZE] = {0}; // 串口2接收缓冲区
char U3_Receive_Buffer[RX_BUFF_SIZE] = {0}; // 串口3接收缓冲区

uint8_t Receive_Gimbal_Mode = 0; //云台模式
int8_t Receive_Target_Speed[3] = {0}; // x,y,w

extern int8_t Chassis_Speed[3]; // x,y,w
extern uint8_t Chassis_Index; // 底盘索引
extern Chassis Chassis_Control;

/**
 * @brief 接收蓝牙数据
 * @note 数据格式为"x%dy%dw%d\n"或"GS\n"等指令
 * @note 该函数从U1_Receive_Buffer中提取最新的一组数据
 */
void Parse_Bluetooth_Data(void)
{
    char* token = strtok(U1_Receive_Buffer, "\n"); // 按换行分割数据帧

    while (token != NULL)
    {
        // 处理模式指令 (GS/GT/GC/GSTOP)
        if (strcmp(token, "GS") == 0)
        {
            Receive_Gimbal_Mode = 1;
        }
        else if (strcmp(token, "GT") == 0)
        {
            Receive_Gimbal_Mode = 2;
        }
        else if (strcmp(token, "GC") == 0)
        {
            Receive_Gimbal_Mode = 3;
        }
        else if (strcmp(token, "GSTOP") == 0)
        {
            Receive_Gimbal_Mode = 0;
        }
        // 处理底盘索引 (CH3/CH4)
        else if (strcmp(token, "CH3") == 0)
        {
            Chassis_Index = 0;
        }
        else if (strcmp(token, "CH4") == 0)
        {
            Chassis_Index = 1;
        }
        // 处理速度指令 (x%dy%dw%d)
        else if (strlen(token) >= 5 && token[0] == 'x' && strchr(token, 'y') && strchr(token, 'w'))
        {
            char* y_pos = strchr(token, 'y');
            char* w_pos = strchr(token, 'w');
            if (y_pos && w_pos && y_pos < w_pos)
            {
                // 解析x值
                int x_val = atoi(token + 1);
                Receive_Target_Speed[0] = (x_val > 127) ? 127 : (x_val < -128) ? -128 : (int8_t)x_val;
                // 解析y值
                int y_val = atoi(y_pos + 1);
                Receive_Target_Speed[1] = (y_val > 127) ? 127 : (y_val < -128) ? -128 : (int8_t)y_val;
                // 解析w值
                int w_val = atoi(w_pos + 1);
                Receive_Target_Speed[2] = (w_val > 127) ? 127 : (w_val < -128) ? -128 : (int8_t)w_val;
            }
        }
        token = strtok(NULL, "\n"); // 处理下一帧数据
    }
}

/**
 * @brief 解析底盘速度数据
 * @note 该函数从U2_Receive_Buffer中提取最新的一组速度数据
 */
void Parse_Chassis_Speed(void)
{
    char* token;
    uint8_t index = 0;

    // 以逗号和换行符为分隔符分割字符串
    token = strtok(U2_Receive_Buffer, ",\n");

    // 解析三个速度值
    while (token != NULL && index < 3)
    {
        // 字符串转整数
        int val = atoi(token);

        // 存入数组（无需额外限幅，因题目未指定范围，直接强转）
        Chassis_Speed[index++] = (int8_t)val;

        // 解析下一个值
        token = strtok(NULL, ",\n");
    }

    // 清空接收缓冲区，准备下次接收
    memset(U2_Receive_Buffer, 0, sizeof(U2_Receive_Buffer));
}

/**
 * @brief 蓝牙模块初始化
 * @param huart UART句柄
 * @param format 格式化字符串
 * @param ... 可变参数
 * @note 使用DMA方式发送数据
 */
void Uart_printf(UART_HandleTypeDef* huart, char* format, ...)
{
    static char buf[RX_BUFF_SIZE]; // 定义临时数组，根据实际发送大小微调
    va_list args;
    va_start(args, format);
    uint32_t len = vsnprintf((char*)buf, sizeof(buf), (char*)format, args);
    va_end(args);
    HAL_UART_Transmit_DMA(huart, (uint8_t*)buf, len);
}

/**
 * @brief UART的DMA接收
 * @param huart UART句柄
 * @param Size 接收数据长度
 * @note 该函数在接收到数据后被调用,蓝牙收到数据闪烁LED7
 *       蓝牙APP中提供的串口助手实际上是和Usart3通信
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size) // 串口接收回调函数
{
    if (huart->Instance == USART1)
    {
        HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
        Parse_Bluetooth_Data();

        memset(U1_Receive_Buffer, 0, sizeof(U1_Receive_Buffer)); // 清空接收缓冲区
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t*)U1_Receive_Buffer, sizeof(U1_Receive_Buffer));
        __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT); // 禁用DMA半传输中断，避免进入两次回调
    }
    else if (huart->Instance == USART2)
    {
        HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
        Parse_Chassis_Speed();
        Uart_printf(&huart1, "%d,%d,%d\n", Chassis_Speed[0], Chassis_Speed[1], Chassis_Speed[2]);

        memset(U2_Receive_Buffer, 0, sizeof(U2_Receive_Buffer));
        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t*)U2_Receive_Buffer, sizeof(U2_Receive_Buffer));
        __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
    }
    else if (huart->Instance == USART3) // 蓝牙APP中的串口助手与此通信
    {
        memset(U3_Receive_Buffer, 0, sizeof(U3_Receive_Buffer));
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, (uint8_t*)U3_Receive_Buffer, sizeof(U3_Receive_Buffer));
        __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
    }
}

/**
 * @brief 串口初始化
 */
void Uart_Init(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t*)U1_Receive_Buffer, sizeof(U1_Receive_Buffer));
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT); // 禁用DMA半传输中断，避免进入两次回调
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t*)U2_Receive_Buffer, sizeof(U2_Receive_Buffer));
    __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, (uint8_t*)U3_Receive_Buffer, sizeof(U3_Receive_Buffer));
    __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
}

/**
 * @brief 停止蓝牙模块
 * @note 该函数停止UART的DMA接收和中断
 */
void BlueTooth_Stop(void)
{
    HAL_UART_DMAStop(&huart1);
    __HAL_UART_DISABLE_IT(&huart1, UART_IT_IDLE);
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);

    Chassis_Control.Status = CHASSIS_STOP; // 停止底盘
    Chassis_Control.Speed_Multiple = 1; // 速度倍率归一
}

/**
 * @brief 启动蓝牙模块
 * @note 该函数重新启动UART的DMA接收
 */
void BlueTooth_Start(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t*)U1_Receive_Buffer, sizeof(U1_Receive_Buffer));
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);

    Chassis_Control.Status = CHASSIS_NORAML;

    Chassis_Control.Speed_Set[0] = 0;
    Chassis_Control.Speed_Set[1] = 0;
    Chassis_Control.Speed_Set[2] = 0;
    Chassis_Control.Speed_Set[3] = 0;

    Chassis_Control.Speed_Multiple = 1; // 速度倍率归一
}
