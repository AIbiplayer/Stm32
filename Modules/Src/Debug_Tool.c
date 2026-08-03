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
#include "Chassis_Task.h"
#include "Cmd_Task.h"
#include "main.h"
#include "stdarg.h"
#include "stdio.h"
#include "string.h"
#include "usart.h"
#include "MG310.h"

extern Bluetooth_Data_s BL_Instance; // 蓝牙数据
extern JOYSTICK_TypeDef JoystickStruct; // PS2手柄数据
extern Chassis_Instance_s CH_Instance; // 底盘数据
extern Camera_Data_s Cam_Instance; // 摄像头数据
extern Motor_Instance_s MG310[4]; // 电机PID数据
extern char Debug_Buffer[RX_BUFF_SIZE / 2]; // 调试串口接收缓冲区

static uint8_t Debug_StrToFloat(char *str, float *value);

static uint8_t Debug_ParseValue(char *buf, const char *key, float *value);

/**
 * @brief 解析调试串口的PID参数
 * @param len 接收数据长度
 * @note 支持SKP=1.05!、SKI=0.10!、SKD=0.01!，未接收到的参数保持原值
 */
void Debug_Parse(const uint16_t len) {
    static char buf[RX_BUFF_SIZE / 2 + 1] = {0};
    uint16_t copy_len = len < RX_BUFF_SIZE / 2 ? len : RX_BUFF_SIZE / 2;
    uint8_t changed = 0;
    float kp = MG310[0].PID.Kp;
    float ki = MG310[0].PID.Ki;
    float kd = MG310[0].PID.Kd;

    memset(buf, 0, sizeof(buf));
    memcpy(buf, Debug_Buffer, copy_len);

    changed |= Debug_ParseValue(buf, "SKP=", &kp);
    changed |= Debug_ParseValue(buf, "SKI=", &ki);
    changed |= Debug_ParseValue(buf, "SKD=", &kd);

    if (changed)
        MG310_ChangePID(kp, ki, kd);

    memset(Debug_Buffer, 0, sizeof(Debug_Buffer));
}

/**
 * @brief 查找并解析指定PID字段
 */
static uint8_t Debug_ParseValue(char *buf, const char *key, float *value) {
    char *start = strstr(buf, key);
    if (start == NULL)
        return 0;

    return Debug_StrToFloat(start + strlen(key), value);
}

/**
 * @brief 将调试串口字符串转换为float
 * @note 解析到'!'结束，格式错误则返回0
 */
static uint8_t Debug_StrToFloat(char *str, float *value) {
    float result = 0.0f;
    float decimal = 0.1f;
    int8_t sign = 1;
    uint8_t has_num = 0;

    if (*str == '-') {
        sign = -1;
        str++;
    } else if (*str == '+') {
        str++;
    }

    while (*str >= '0' && *str <= '9') {
        result = result * 10.0f + (*str - '0');
        str++;
        has_num = 1;
    }

    if (*str == '.') {
        str++;
        while (*str >= '0' && *str <= '9') {
            result += (*str - '0') * decimal;
            decimal *= 0.1f;
            str++;
            has_num = 1;
        }
    }

    if (!has_num || *str != '!')
        return 0;

    *value = result * sign;
    return 1;
}

/**
 * @brief 通过UART调试PID参数输出
 */
void Debug_PID(void) {
    Uart_printf(&UART_DEBUG, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
                (int) MG310[0].PID.Kp,
                (int) MG310[0].PID.Ki,
                (int) MG310[0].PID.Kd,
                (int) MG310[0].PID.Actual,
                (int) MG310[0].PID.Target,
                (int) MG310[0].PID.Output,
                (int) MG310[1].PID.Actual,
                (int) MG310[1].PID.Target,
                (int) MG310[1].PID.Output,
                (int) MG310[2].PID.Actual,
                (int) MG310[2].PID.Target,
                (int) MG310[2].PID.Output,
                (int) MG310[3].PID.Actual,
                (int) MG310[3].PID.Target,
                (int) MG310[3].PID.Output
    );
}

/**
 * @brief 通过蓝牙调试数据输出
 */
void Debug_Bluetooth(void) {
    switch (BL_Instance.Mode) {
        case MODE_ROCKER:
            Uart_printf(&UART_DEBUG, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
                        BL_Instance.X_L,
                        BL_Instance.Y_L,
                        BL_Instance.Rocker_Handle_Data.X_R,
                        BL_Instance.Rocker_Handle_Data.Y_R,
                        BL_Instance.Rocker_Handle_Data.L1,
                        BL_Instance.Rocker_Handle_Data.L2,
                        BL_Instance.Rocker_Handle_Data.R1,
                        BL_Instance.Rocker_Handle_Data.R2,
                        BL_Instance.Rocker_Handle_Data.K1,
                        BL_Instance.Rocker_Handle_Data.K2,
                        BL_Instance.Rocker_Handle_Data.K3,
                        BL_Instance.Rocker_Handle_Data.K4);
            break;
        case MODE_HANDLE:
            Uart_printf(&UART_DEBUG, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
                        BL_Instance.X_L,
                        BL_Instance.Y_L,
                        BL_Instance.Rocker_Handle_Data.X_R,
                        BL_Instance.Rocker_Handle_Data.Y_R,
                        BL_Instance.Dif_Data.Handle_Data.Up,
                        BL_Instance.Dif_Data.Handle_Data.Back,
                        BL_Instance.Dif_Data.Handle_Data.Left,
                        BL_Instance.Dif_Data.Handle_Data.Right,
                        BL_Instance.Dif_Data.Handle_Data.A,
                        BL_Instance.Dif_Data.Handle_Data.B,
                        BL_Instance.Dif_Data.Handle_Data.X,
                        BL_Instance.Dif_Data.Handle_Data.Y,
                        BL_Instance.Rocker_Handle_Data.L1,
                        BL_Instance.Rocker_Handle_Data.L2,
                        BL_Instance.Rocker_Handle_Data.R1,
                        BL_Instance.Rocker_Handle_Data.R2,
                        BL_Instance.Rocker_Handle_Data.K1,
                        BL_Instance.Rocker_Handle_Data.K2,
                        BL_Instance.Rocker_Handle_Data.K3,
                        BL_Instance.Rocker_Handle_Data.K4);
            break;
        case MODE_GRAVITY:
            Uart_printf(&UART_DEBUG, "%d,%d\r\n",
                        BL_Instance.X_L,
                        BL_Instance.Y_L);
            break;
        default:
            break;
    }
}

/**
 * @brief 通过PS2手柄调试数据输出
 */
void Debug_PS2(void) {
    Uart_printf(&UART_DEBUG, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
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
 * @brief 通过UART调试底盘数据输出
 */
void Debug_Chassis(void) {
    Uart_printf(&UART_DEBUG, "%d,%d,%d,%d,%d,%d\r\n",
                CH_Instance.Move.x,
                CH_Instance.Move.y,
                CH_Instance.Move.w,
                (int8_t) CH_Instance.Status,
                (int8_t) CH_Instance.Control_Mode,
                (int8_t) Cam_Instance.Mode
    );
}

/**
 * @brief 通过UART调试摄像头数据输出
 */
void Debug_Camera(void) {
    Uart_printf(&UART_DEBUG, "%d,%d,%d,%d\r\n",
                (int8_t) Cam_Instance.Mode,
                (int8_t) Cam_Instance.Target_Found,
                Cam_Instance.Error_X,
                Cam_Instance.Error_Y
    );
}

/**
 * @brief 蜂鸣器打开
 */
void BUZZ_ON(void) {
    HAL_GPIO_WritePin(BUZZ_GPIO_Port,BUZZ_Pin, GPIO_PIN_RESET);
}

/**
 * @brief 蜂鸣器关闭
 */
void BUZZ_OFF(void) {
    HAL_GPIO_WritePin(BUZZ_GPIO_Port,BUZZ_Pin, GPIO_PIN_SET);
}

/**
 * @brief 格式化输出到UART
 * @param huart UART句柄
 * @param format 格式化字符串
 * @param ... 可变参数
 * @note 使用DMA方式发送数据
 */
void Uart_printf(UART_HandleTypeDef *huart, char *format, ...) {
    static char buf[RX_BUFF_SIZE]; // 定义临时数组，根据实际发送大小微调
    va_list args;
    va_start(args, format);
    uint32_t len = vsnprintf((char *) buf, sizeof(buf), (char *) format, args);
    va_end(args);
    HAL_UART_Transmit_DMA(huart, (uint8_t *) buf, len);
}
