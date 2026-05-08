/**
* @file Ins_Task.c
 * @brief 获取陀螺仪任务
 * @author Shen FeiLin
 * @date 2025/11/9
 */

#include "usb_device.h"
#include "cmsis_os.h"
#include "INS.h"
#include "TMC.h"
#include "DM_Motor.h"
#include "referee.h"
#include "daemon.h"
#include "can_comm.h"
#include "master_process.h"

extern INS_t *Gimbal_IMU_Data; ///< 云台IMU数据
extern DM_Motor_Instance *Gimbal_Pitch_Up; ///< Pitch轴达妙电机小Pitch
extern DM_Motor_Instance *Gimbal_Pitch_Down; ///< Pitch轴达妙电机大Pitch
extern CANCommInstance *CANCOM; // 底盘或云台的CAN通信实例指针
extern referee_info_t *Referee_data; // 裁判系统数据

TMC_To_Gimbal_s *Gimbal_Rec; // 云台与底盘数据结构体实例
DM_IMU_Instance_s DM_IMU; // 保留给达妙驱动未使用接口链接，业务逻辑不再读取达妙IMU

static void IMU_Init(void);

/**
 * @brief 陀螺仪任务
 * @note Init函数在Gimbal_Task中调用过了
 */
void INSTask(void const *argument) {
    taskENTER_CRITICAL();

    IMU_Init();

    taskEXIT_CRITICAL();
    for (;;) {
        INS_Task();
        DaemonTask();
#ifdef MCU_GIMBAL
        VisionUpdateRealtimeData(
            Gimbal_IMU_Data,
            Gimbal_Pitch_Up != NULL ? Gimbal_Pitch_Up->Measure.total_angle : 0.0f,
            Gimbal_Pitch_Down != NULL ? Gimbal_Pitch_Down->Measure.total_angle : 0.0f);
        VisionUpdateNRTData(
            Gimbal_Rec != NULL ? Gimbal_Rec->Shoot_Upload_Data.heat : 0u,
            Gimbal_Rec != NULL ? Gimbal_Rec->Shoot_Upload_Data.shooter_heat_limit : 0u,
            Gimbal_Rec != NULL ? Gimbal_Rec->Shoot_Upload_Data.robot_color : 0u,
            Gimbal_Rec != NULL ? (float) Gimbal_Rec->Shoot_Upload_Data.bullet_speed / 1024.0f : 0.0f);
        VisionSend();

#endif
        osDelay(1);
    }
}

/**
 * @brief 陀螺仪任务初始化
 * @note 这里可以调整IMU参数
 */
void IMU_Init(void) {
    Gimbal_IMU_Data = INS_Init();
    Gimbal_Rec = (TMC_To_Gimbal_s *) CANCommGet(CANCOM);
    LED_Green_Up;
}
