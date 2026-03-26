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
#include "referee.h"
#include "can_comm.h"

extern INS_t *Gimbal_IMU_Data; ///< 云台IMU数据
extern CANCommInstance *CANCOM; // 底盘或云台的CAN通信实例指针
extern referee_info_t *Referee_data; // 裁判系统数据

TMC_To_Gimbal_s *Gimbal_Rec; // 云台与底盘数据结构体实例

/**
 * @brief 陀螺仪任务
 * @note Init函数在Gimbal_Task中调用过了
 */
void INSTask(void const *argument) {
    taskENTER_CRITICAL();
    Gimbal_IMU_Data = INS_Init();
    LED_Green_Up;
    Gimbal_Rec = (TMC_To_Gimbal_s *) CANCommGet(CANCOM);
    taskEXIT_CRITICAL();
    for (;;) {
        INS_Task();

#ifdef MCU_GIMBAL
        VisionSetAltitude(Gimbal_IMU_Data->Yaw, Gimbal_IMU_Data->Roll,
                          Referee_data->GameRobotState.robot_id > 100 ? 1 : 0,
                          AIM_NORMAL); //@todo 根据C板放置位置，Pitch和Roll调换位置
        VisionSend();

#endif
        osDelay(1);
    }
}
