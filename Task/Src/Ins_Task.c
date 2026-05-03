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
extern CANCommInstance *CANCOM; // 底盘或云台的CAN通信实例指针
extern referee_info_t *Referee_data; // 裁判系统数据

TMC_To_Gimbal_s *Gimbal_Rec; // 云台与底盘数据结构体实例
DM_IMU_Instance_s DM_IMU; // 达妙IMU数据结构体实例

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
            &DM_IMU.Measure,
            Gimbal_Pitch_Up != NULL ? Gimbal_Pitch_Up->Measure.total_angle : 0.0f,
            Gimbal_Rec != NULL ? Gimbal_Rec->Shoot_Upload_Data.rec_vx : 0,
            Gimbal_Rec != NULL ? Gimbal_Rec->Shoot_Upload_Data.rec_vy : 0,
            INS_TASK_PERIOD * 0.001f);
        VisionUpdateNRTData(
            Gimbal_Rec != NULL ? Gimbal_Rec->Shoot_Upload_Data.heat : 0u,
            Gimbal_Rec != NULL ? Gimbal_Rec->Shoot_Upload_Data.shooter_heat_limit : 0u,
            Gimbal_Rec != NULL ? Gimbal_Rec->Shoot_Upload_Data.robot_color : 0u);
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

    DM_IMU.Can_Init_Config.can_handle = &hcan2;
    DM_IMU.Can_Init_Config.rx_id = 0x014;
    DM_IMU.Can_Init_Config.tx_id = 0x010;
    DM_IMU.Can_Init_Config.id = &DM_IMU;
    DM_IMU.Can_Init_Config.can_module_callback = Decode_dm_imu;

    DM_IMU.IMU_Can_Instance = CANRegister(&DM_IMU.Can_Init_Config);
    dm_imu_reset();
    LED_Green_Up;
}
