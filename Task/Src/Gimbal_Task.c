#include "Gimbal_Task.h"
#include "Debug_Tool.h"
#include "Servo.h"
#include "Cmd_Task.h"
#include "Chassis_Task.h"
#include "tim.h"

extern Camera_Data_s Cam_Instance; // 摄像头数据实例
extern Chassis_Instance_s CH_Instance; // 底盘数据实例

/**
 * @brief 云台任务函数
 */
void Gimbal_Task(void) {
    if (Cam_Instance.Mode == FACE_VISION && Cam_Instance.Target_Found) //人脸识别只动pitch
        Servo_Set_Auto();
    else if (Cam_Instance.Mode == TRAIL_VISION) //视觉巡线
    {
        __HAL_TIM_SET_COMPARE(&htim9, PITCH_CHANNEL, 500);
        __HAL_TIM_SET_COMPARE(&htim9, YAW_CHANNEL, 500);
    }
    else if (CH_Instance.Control_Mode == BLUETOOTH_MODE) //蓝牙模式
        Servo_Set_Ble();
    else if (CH_Instance.Control_Mode == PS2_MODE) //遥控模式
        Servo_Set_PS2();
}
