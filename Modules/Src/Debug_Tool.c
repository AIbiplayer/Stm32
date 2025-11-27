/**
 * @file Debug_Tool.c
 * @brief 调试工具模块
 * @date 2025/11/25
 * @details 这里把所有需要检测的数据通过串口打印出来，
 *          或者使用FreeMaster等工具进行可视化，使用Uart1
 */

#include "Debug_Tool.h"
#include "Bluetooth.h"
#include "ax_ps2.h"
#include "main.h"
#include "stdarg.h"
#include "stdio.h"
#include "usart.h"

extern Bluetooth_Data_s Instance; // 蓝牙数据
extern JOYSTICK_TypeDef JoystickStruct; // PS2手柄数据

void Debug_Bluetooth(void)
{
    switch (Instance.Mode)
    {
    case MODE_ROCKER:
        Uart_printf(&huart1, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
                    Instance.X_L,
                    Instance.Y_L,
                    Instance.Rocker_Handle_Data.X_R,
                    Instance.Rocker_Handle_Data.Y_R,
                    Instance.Rocker_Handle_Data.L1,
                    Instance.Rocker_Handle_Data.L2,
                    Instance.Rocker_Handle_Data.R1,
                    Instance.Rocker_Handle_Data.R2,
                    Instance.Rocker_Handle_Data.K1,
                    Instance.Rocker_Handle_Data.K2,
                    Instance.Rocker_Handle_Data.K3,
                    Instance.Rocker_Handle_Data.K4);
        break;
    case MODE_HANDLE:
        Uart_printf(&huart1, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
                    Instance.X_L,
                    Instance.Y_L,
                    Instance.Rocker_Handle_Data.X_R,
                    Instance.Rocker_Handle_Data.Y_R,
                    Instance.Dif_Data.Handle_Data.Up,
                    Instance.Dif_Data.Handle_Data.Back,
                    Instance.Dif_Data.Handle_Data.Left,
                    Instance.Dif_Data.Handle_Data.Right,
                    Instance.Dif_Data.Handle_Data.A,
                    Instance.Dif_Data.Handle_Data.B,
                    Instance.Dif_Data.Handle_Data.X,
                    Instance.Dif_Data.Handle_Data.Y,
                    Instance.Rocker_Handle_Data.L1,
                    Instance.Rocker_Handle_Data.L2,
                    Instance.Rocker_Handle_Data.R1,
                    Instance.Rocker_Handle_Data.R2,
                    Instance.Rocker_Handle_Data.K1,
                    Instance.Rocker_Handle_Data.K2,
                    Instance.Rocker_Handle_Data.K3,
                    Instance.Rocker_Handle_Data.K4);
        break;
    case MODE_GRAVITY:
        Uart_printf(&huart1, "%d,%d,%d\r\n",
                    Instance.X_L,
                    Instance.Y_L,
                    Instance.Dif_Data.Gravity_Data.G_Yaw);
        break;
    default:
        break;
    }
}

void Debug_PS2(void)
{
    Uart_printf(&huart1, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
                JoystickStruct.select,
                JoystickStruct.button_L,
                JoystickStruct.button_R,
                JoystickStruct.start,
                JoystickStruct.up,
                JoystickStruct.down,
                JoystickStruct.left,
                JoystickStruct.right,
                JoystickStruct.Triangle,
                JoystickStruct.Circle,
                JoystickStruct.Cross,
                JoystickStruct.Square,
                JoystickStruct.L1,
                JoystickStruct.L2,
                JoystickStruct.R1,
                JoystickStruct.R2,
                JoystickStruct.RJoy_LR,
                JoystickStruct.RJoy_UD,
                JoystickStruct.LJoy_LR,
                JoystickStruct.LJoy_UD);
}

/**
 * @brief 蜂鸣器打开
 */
void BUZZ_ON(void)
{
    HAL_GPIO_WritePin(BUZZ_GPIO_Port,BUZZ_Pin, GPIO_PIN_RESET);
}

/**
 * @brief 蜂鸣器关闭
 */
void BUZZ_OFF(void)
{
    HAL_GPIO_WritePin(BUZZ_GPIO_Port,BUZZ_Pin, GPIO_PIN_SET);
}

/**
 * @brief 格式化输出到UART
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
