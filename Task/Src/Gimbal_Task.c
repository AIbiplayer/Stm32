#include "Gimbal_Task.h"
#include "Debug_Tool.h"
#include "../../Motor/Inc/Servo.h"
#include "Gimbal_Trace.h"
#include "Cmd_Task.h"

extern CCMRAM_DATA Camera_Data_s Cam_Instance; // 摄像头数据实例

/**
 * @brief 云台任务函数
 */
void Gimbal_Task(void) {
    Servo_Init();

    // if (Cam_Instance.Mode == FACE_VISION) //人脸识别只动pitch
    // {
    //     auto_tracing(YAW);
    //     auto_tracing(PITCH);
    // } else if (Cam_Instance.Mode == LASER_VISION) //激光避障yaw、pitch都动
    // {
    //     auto_tracing(PITCH);
    //     auto_tracing(YAW);
    // } else if (Cam_Instance.Mode == TRAIL_VISION) //寻迹时均不动
    // {
    // }
}
