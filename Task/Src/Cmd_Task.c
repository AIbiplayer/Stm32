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
extern char *Buffer_Ptr;
char Debug_Buffer[RX_BUFF_SIZE / 2];
extern double pitch_, yaw, roll_; // 角度滤波后数据
extern Motor_Instance_s MG310[4];
extern Chassis_Instance_s CH_Instance;
extern JOYSTICK_TypeDef JoystickStruct;
extern Bluetooth_Data_s BL_Instance; // 蓝牙数据实例

bool Toggle = false; // 切换双缓冲区标志
Camera_Data_s Cam_Instance = {NONE_VISION, 1, 0, 0}; // 摄像头数据实例
uint8_t Camera_Receive_Buffer[RX_BUFF_SIZE / 8]; // 摄像头接收缓冲区
uint8_t Camera_Transmit_Buffer[3] = {0xA4, 0x00, 0x03}; // 摄像头发送缓冲区

static void Camera_Parse(uint8_t len);

/**
 * @brief 底盘控制任务
 */
void Cmd_Task(void) {
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
void Clear_AllInstance(void) {
    BUZZ_OFF();

    Cam_Instance.Mode = NONE_VISION;
    CH_Instance.Status = CHASSIS_MEC;
    for (uint8_t i = 0; i < 4; i++) {
        PID_Clean_I(&MG310[i].PID);
        MG310[i].PID.Output = 0;
    }
    memset(&CH_Instance.Move, 0, sizeof(CH_Instance.Move));
    memset(CH_Instance.Speed_Set, 0, sizeof(CH_Instance.Speed_Set));
}

/**
 * @brief 串口初始化
 */
void Uart_Init(void) {
    HAL_UARTEx_ReceiveToIdle_DMA(&UART_BLUETOOTH, (uint8_t *) Buffer_Ptr, sizeof(Bluetooth_Receive_Buffer[0]));
    __HAL_DMA_DISABLE_IT(UART_BLUETOOTH.hdmarx, DMA_IT_HT);
    HAL_UARTEx_ReceiveToIdle_DMA(&UART_CAMERA, Camera_Receive_Buffer, sizeof(Camera_Receive_Buffer));
    __HAL_DMA_DISABLE_IT(UART_CAMERA.hdmarx, DMA_IT_HT);
    HAL_UARTEx_ReceiveToIdle_DMA(&UART_DEBUG, (uint8_t *) Debug_Buffer, sizeof(Debug_Buffer));
    __HAL_DMA_DISABLE_IT(UART_DEBUG.hdmarx, DMA_IT_HT);
}

/**
 * @brief UART的DMA接收
 * @param huart UART句柄
 * @param Size 接收数据长度
 * @note 此函数是蓝牙和摄像头共用的回调函数
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) // 串口接收回调函数
{
    //蓝牙部分串口接收
    if (huart->Instance == UART_BLUETOOTH.Instance) {
        HAL_UART_DMAStop(&UART_BLUETOOTH);
        //解析并清空缓冲区
        Bluetooth_Parse(Size);
        // 切换缓冲区指针
        Buffer_Ptr = Toggle ? Bluetooth_Receive_Buffer[0] : Bluetooth_Receive_Buffer[1];
        Toggle = !Toggle;
        // 重新启动DMA接收
        HAL_UARTEx_ReceiveToIdle_DMA(&UART_BLUETOOTH, (uint8_t *) Buffer_Ptr, sizeof(Bluetooth_Receive_Buffer[0]));
        __HAL_DMA_DISABLE_IT(UART_BLUETOOTH.hdmarx, DMA_IT_HT); // 禁用DMA半传输中断，避免进入两次回调
    }
    if (huart->Instance == UART_CAMERA.Instance) {
        HAL_UART_DMAStop(&UART_CAMERA);
        Camera_Parse(Size);
        HAL_UARTEx_ReceiveToIdle_DMA(&UART_CAMERA, (uint8_t *) Camera_Receive_Buffer, sizeof(Camera_Receive_Buffer));
        __HAL_DMA_DISABLE_IT(UART_CAMERA.hdmarx, DMA_IT_HT); // 禁用DMA半传输中断，避免进入两次回调
    }
    if (huart->Instance == UART_DEBUG.Instance) {
        HAL_UART_DMAStop(&UART_DEBUG);
        Debug_Parse(Size);
        HAL_UARTEx_ReceiveToIdle_DMA(&UART_DEBUG, (uint8_t *) Debug_Buffer, sizeof(Debug_Buffer));
        __HAL_DMA_DISABLE_IT(UART_DEBUG.hdmarx, DMA_IT_HT); // 禁用DMA半传输中断，避免进入两次回调
    }
}

/**
 * @brief 蓝牙和PS2键位设置
 * @todo 扩展视觉和云台模块
 * @note 重力感应部分没有使用Yaw角速度控制底盘
 */
void Key_Setting(void) {
    CH_Instance.Control_Mode = JoystickStruct.select_mode ? PS2_MODE : BLUETOOTH_MODE;

    if (JoystickStruct.select_mode != JoystickStruct.select_mode_last)
        Clear_AllInstance();

    switch (CH_Instance.Control_Mode) {
        case BLUETOOTH_MODE:
            switch (BL_Instance.Mode) {
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
                    break;
                case MODE_GRAVITY:
                    CH_Instance.Move.x = abs(BL_Instance.X_L) > 50
                                             ? BL_Instance.X_L / abs(BL_Instance.X_L) * 50
                                             : BL_Instance.X_L;
                    CH_Instance.Move.y = abs(BL_Instance.Y_L) > 50
                                             ? BL_Instance.Y_L / abs(BL_Instance.Y_L) * 50
                                             : BL_Instance.Y_L;
                    Cam_Instance.Mode = NONE_VISION;
                    break;
            }
            break;
        case PS2_MODE:
            // 这意味着MODE没有变成绿色
            if (JoystickStruct.LJoy_LR == 0xFF81 && JoystickStruct.RJoy_LR == 0xFF81 &&
                JoystickStruct.LJoy_UD == 0xFF80 && JoystickStruct.RJoy_UD == 0xFF80) {
                memset(&CH_Instance.Move, 0, sizeof(CH_Instance.Move));
            } else {
                CH_Instance.Move.x = JoystickStruct.LJoy_UD / 1.5f;
                CH_Instance.Move.y = JoystickStruct.LJoy_LR / 1.5f;
                CH_Instance.Move.w = JoystickStruct.RJoy_LR / 1.5f;
                switch (JoystickStruct.Control_Mode) {
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
                Cam_Instance.Mode = JoystickStruct.button_R && !JoystickStruct.button_R_Last
                                        ? (Cam_Instance.Mode + 1) % 4
                                        : Cam_Instance.Mode;
                JoystickStruct.button_L ? BUZZ_ON() : BUZZ_OFF();
            }
            /*
            *这里扩展视觉和云台部分
            */
            break;
    }
    // 视觉模式处理
    switch (Cam_Instance.Mode) // @todo 这里添加云台代码，包括进入视觉模式时取消云台主动转动
    {
        case FACE_VISION: // 人脸识别
            Camera_Transmit_Buffer[1] = 0x01;
            CH_Instance.Move.x = 0, CH_Instance.Move.y = 0;
            /* 云台数据接收 */
            HAL_UART_Transmit_DMA(&UART_CAMERA, Camera_Transmit_Buffer, sizeof(Camera_Transmit_Buffer));
            break;
        case TRAIL_VISION: // 视觉巡线
            Camera_Transmit_Buffer[1] = 0x02;
            CH_Instance.Move.x = Cam_Instance.Target_Found ? 40 : 0;
            CH_Instance.Move.y = 0;
            CH_Instance.Move.w = Cam_Instance.Target_Found ? Cam_Instance.Error_X : 0;
            HAL_UART_Transmit_DMA(&UART_CAMERA, Camera_Transmit_Buffer, sizeof(Camera_Transmit_Buffer));
            break;
        case LASER_VISION: // 激光避障
            Camera_Transmit_Buffer[1] = 0x03;
            /* 云台数据接收 */
            HAL_UART_Transmit_DMA(&UART_CAMERA, Camera_Transmit_Buffer, sizeof(Camera_Transmit_Buffer));
            break;
    }
}

/**
 * @brief OLED显示任务
 */
void OLED_SHOW(void) {
    // OLED显示底盘状态
    OLED_ShowString(1, 1, "Mode:");
    switch (CH_Instance.Status) {
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
    // 控制模式显示
    OLED_ShowString(2, 1, "Control:");
    switch (CH_Instance.Control_Mode) {
        case PS2_MODE:
            OLED_ShowString(2, 10, "PS2");
            break;
        case BLUETOOTH_MODE:
            OLED_ShowString(2, 10, "Ble");
            break;
    }
    // 视觉模式显示
    OLED_ShowString(3, 1, "Vision:");
    switch (Cam_Instance.Mode) {
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
    if (Cam_Instance.Mode == NONE_VISION) {
        // 显示姿态角
        OLED_ShowString(4, 1, "Y:");
        OLED_ShowSignedNum(4, 3, yaw, 2);
        OLED_ShowString(4, 6, "P:");
        OLED_ShowSignedNum(4, 8, pitch_, 2);
        OLED_ShowString(4, 12, "R:");
        OLED_ShowSignedNum(4, 14, roll_, 2);
    } else {
        OLED_ShowString(4, 1, "EX:");
        OLED_ShowSignedNum(4, 4, Cam_Instance.Error_X, 4);
        OLED_ShowString(4, 9, "EY:");
        OLED_ShowSignedNum(4, 12, Cam_Instance.Error_Y, 4);
    }
}

/**
 * @brief 摄像头数据解析
 * @param len 数据长度
 */
static void Camera_Parse(const uint8_t len) {
    static uint8_t buf[RX_BUFF_SIZE / 8] = {0}; // 临时存储接收数据
    memcpy(buf, Camera_Receive_Buffer, sizeof(Camera_Receive_Buffer));
    // 数据包校验
    if (buf[0] != 0xA5 || len != 5 || (buf[1] & 0x0F) != (uint8_t) Cam_Instance.Mode) // 数据包头错误
    {
        Cam_Instance.Target_Found = 0;
        Cam_Instance.Error_X = 0;
        Cam_Instance.Error_Y = 0;
        return;
    }
    uint8_t checksum = 0;
    for (uint8_t i = 1; i < len - 1; i++) // 计算校验和，最后一个字节为校验和
        checksum += buf[i];
    if (checksum != buf[len - 1])
        return;
    Cam_Instance.Target_Found = (buf[1] >> 4) & 0x01;
    Cam_Instance.Error_X = Cam_Instance.Target_Found ? -buf[2] : 0;
    Cam_Instance.Error_Y = Cam_Instance.Target_Found ? -buf[3] : 0;
}
