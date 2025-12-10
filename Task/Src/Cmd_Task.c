/**
* @file Cmd_Task.c
 * @brief 底盘控制任务
 * @date 2025/11/28
 */

#include "Cmd_Task.h"
#include "ax_delay.h"
#include "main.h"
#include "Debug_Tool.h"
#include "Chassis_Task.h"
#include "usart.h"
#include "ax_ps2.h"
#include "MG310.h"
#include "OLED.h"
#include "Mpu6050.h"
#include "stdlib.h"
#include "string.h"
#include "Bluetooth.h"

extern char Bluetooth_Receive_Buffer[2][RX_BUFF_SIZE];
extern char* Buffer_Ptr;
extern double pitch_, yaw, roll_; // 角度滤波后数据
extern Motor_Instance_s MG310[4];
extern Chassis_Instance_s CH_Instance;
extern JOYSTICK_TypeDef JoystickStruct;
extern Bluetooth_Data_s BL_Instance; // 蓝牙数据实例

static bool Toggle = false; // 切换双缓冲区标志

/**
 * @brief 底盘控制任务
 */
void Cmd_Task(void)
{
    KalmanFilter();
    AX_PS2_ScanKey();
    Key_Setting();
    OLED_SHOW();
    AX_Delayms(30);
}

/**
 * @brief 清除实例部分数据
 * @note 保证模式切换时底盘的稳定
 */
void Clear_AllInstance(void)
{
    BUZZ_OFF();

    CH_Instance.Vision_Mode = NONE_VISION;
    CH_Instance.Status = CHASSIS_MEC;
    for (uint8_t i = 0; i < 4; i++)
    {
        PID_Clean_I(&MG310[i].PID);
        MG310[i].PID_Output = 0;
    }
    memset(&CH_Instance.Move, 0, sizeof(CH_Instance.Move));
    memset(CH_Instance.Speed_Set, 0, sizeof(CH_Instance.Speed_Set));
}

/**
 * @brief UART的DMA接收
 * @param huart UART句柄
 * @param Size 接收数据长度
 * @note 此函数是蓝牙和摄像头共用的回调函数
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size) // 串口接收回调函数
{
    //蓝牙部分串口接收
    if (huart->Instance == UART_BLUETOOTH.Instance)
    {
        HAL_UART_DMAStop(&UART_BLUETOOTH);
        //解析并清空缓冲区
        Bluetooth_Parse(Size);
        // 切换缓冲区指针
        Buffer_Ptr = Toggle ? Bluetooth_Receive_Buffer[0] : Bluetooth_Receive_Buffer[1];
        Toggle = !Toggle;
        // 重新启动DMA接收
        HAL_UARTEx_ReceiveToIdle_DMA(&UART_BLUETOOTH, (uint8_t*)Buffer_Ptr, sizeof(Bluetooth_Receive_Buffer[0]));
        __HAL_DMA_DISABLE_IT(UART_BLUETOOTH.hdmarx, DMA_IT_HT); // 禁用DMA半传输中断，避免进入两次回调
    }
}

/**
 * @brief 蓝牙和PS2键位设置
 * @todo 扩展视觉和云台模块
 * @note 重力感应部分没有使用Yaw角速度控制底盘
 */
void Key_Setting(void)
{
    CH_Instance.Control_Mode = JoystickStruct.select_mode ? PS2_MODE : BLUETOOTH_MODE;

    if (JoystickStruct.select_mode != JoystickStruct.select_mode_last)
    {
        !JoystickStruct.select_mode ? Bluetooth_Start() : Bluetooth_Stop();
        Clear_AllInstance();
    }

    switch (CH_Instance.Control_Mode)
    {
    case BLUETOOTH_MODE:
        switch (BL_Instance.Mode)
        {
        case MODE_ROCKER:
            CH_Instance.Move.x = BL_Instance.X_L;
            CH_Instance.Move.y = BL_Instance.Y_L;
            CH_Instance.Move.w = BL_Instance.Rocker_Handle_Data.Y_R;
            if (!BL_Instance.Rocker_Handle_Data.K1 && !BL_Instance.Rocker_Handle_Data.K2)
                CH_Instance.Status = CHASSIS_MEC;
            else if (!BL_Instance.Rocker_Handle_Data.K1 && BL_Instance.Rocker_Handle_Data.K2)
                CH_Instance.Status = CHASSIS_OMNI_TRI;
            else if (BL_Instance.Rocker_Handle_Data.K1 && !BL_Instance.Rocker_Handle_Data.K2)
                CH_Instance.Status = CHASSIS_OMNI_SQU;
            /*
             *这里扩展视觉和云台部分
             */
            break;
        case MODE_HANDLE:
            CH_Instance.Move.x = BL_Instance.X_L;
            CH_Instance.Move.y = BL_Instance.Y_L;
            CH_Instance.Move.w = BL_Instance.Rocker_Handle_Data.Y_R;
            if (!BL_Instance.Rocker_Handle_Data.K1 && !BL_Instance.Rocker_Handle_Data.K2)
                CH_Instance.Status = CHASSIS_MEC;
            else if (!BL_Instance.Rocker_Handle_Data.K1 && BL_Instance.Rocker_Handle_Data.K2)
                CH_Instance.Status = CHASSIS_OMNI_TRI;
            else if (BL_Instance.Rocker_Handle_Data.K1 && !BL_Instance.Rocker_Handle_Data.K2)
                CH_Instance.Status = CHASSIS_OMNI_SQU;
            /*
            *这里扩展视觉和云台部分
            */
            break;
        case MODE_GRAVITY:
            CH_Instance.Move.x = abs(BL_Instance.X_L) > 50
                                     ? BL_Instance.X_L / abs(BL_Instance.X_L) * 50
                                     : BL_Instance.X_L;
            CH_Instance.Move.y = abs(BL_Instance.Y_L) > 50
                                     ? BL_Instance.Y_L / abs(BL_Instance.Y_L) * 50
                                     : BL_Instance.Y_L;
            CH_Instance.Vision_Mode = NONE_VISION;
            break;
        }
        break;
    case PS2_MODE:
        // 这意味着MODE没有变成绿色
        if (JoystickStruct.LJoy_LR == 127 && JoystickStruct.RJoy_LR == 127 &&
            JoystickStruct.LJoy_UD == 128 && JoystickStruct.RJoy_UD == 128)
        {
            memset(&CH_Instance.Move, 0, sizeof(CH_Instance.Move));
        }
        else
        {
            CH_Instance.Move.x = JoystickStruct.LJoy_UD / 1.5f;
            CH_Instance.Move.y = JoystickStruct.LJoy_LR / 1.5f;
            CH_Instance.Move.w = JoystickStruct.RJoy_LR / 1.5f;
            switch (JoystickStruct.Control_Mode)
            {
            case 0:
                CH_Instance.Status = CHASSIS_MEC;
                break;
            case 1:
                CH_Instance.Status = CHASSIS_OMNI_TRI;
                break;
            case 2:
                CH_Instance.Status = CHASSIS_OMNI_SQU;
                break;
            default:
                CH_Instance.Status = CHASSIS_MEC;
                break;
            }
            CH_Instance.Vision_Mode = JoystickStruct.button_R && !JoystickStruct.button_R_Last
                                          ? (CH_Instance.Vision_Mode + 1) % 4
                                          : CH_Instance.Vision_Mode;
            JoystickStruct.button_L ? BUZZ_ON() : BUZZ_OFF();
        }
        /*
        *这里扩展视觉和云台部分
        */
        break;
    }
}

/**
 * @brief OLED显示任务
 */
void OLED_SHOW(void)
{
    OLED_ShowString(1, 1, "Mode:");
    switch (CH_Instance.Status)
    {
    case CHASSIS_MEC:
        OLED_ShowString(1, 7, "Mec     ");
        break;
    case CHASSIS_OMNI_TRI:
        OLED_ShowString(1, 7, "Omni_Tri");
        break;
    case CHASSIS_OMNI_SQU:
        OLED_ShowString(1, 7, "Omni_Squ");
        break;
    }

    OLED_ShowString(2, 1, "Control:");
    switch (CH_Instance.Control_Mode)
    {
    case PS2_MODE:
        OLED_ShowString(2, 10, "PS2");
        break;
    case BLUETOOTH_MODE:
        OLED_ShowString(2, 10, "Ble");
        break;
    }

    OLED_ShowString(3, 1, "Vision:");
    switch (CH_Instance.Vision_Mode)
    {
    case NONE_VISION:
        OLED_ShowString(3, 9, "None ");
        break;
    case TRAIL_VISION:
        OLED_ShowString(3, 9, "Trail");
        break;
    case FACE_VISION:
        OLED_ShowString(3, 9, "Face ");
        break;
    case LASER_VISION:
        OLED_ShowString(3, 9, "Laser");
        break;
    }
    OLED_ShowString(4, 1, "Y:");
    OLED_ShowSignedNum(4, 3, yaw, 2);
    OLED_ShowString(4, 6, "P:");
    OLED_ShowSignedNum(4, 8, pitch_, 2);
    OLED_ShowString(4, 12, "R:");
    OLED_ShowSignedNum(4, 14, roll_, 2);
}
