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
#include "can_comm.h"
#include "HC_SR04.h"

extern INS_t *Gimbal_IMU_Data; ///< 云台IMU数据
extern CANCommInstance *CANCOM; // 底盘或云台的CAN通信实例指针

CCMRAM float HC_Measure = 0.0f; ///< 超声波测量值
TMC_To_Gimbal_s *Gimbal_Rec; // 云台与底盘数据结构体实例

/**
 * @brief 陀螺仪任务
 * @note Init函数在Gimbal_Task中调用过了
 */
void INSTask(void const *argument) {
    taskENTER_CRITICAL();
    Gimbal_IMU_Data = INS_Init();
    // HC_Init();
    LED_Green_Up;
    Gimbal_Rec = (TMC_To_Gimbal_s *) CANCommGet(CANCOM);
    taskEXIT_CRITICAL();
    for (;;) {
        INS_Task();
        osDelay(1);

// #ifdef MCU_CHASSIS
//         Count == 1 ? HC_Send_Trig() : Count > 20 ? (HC_Measure = HC_Get_Measure(), Count = 0) : 0;
//         Count++;
// // #elifdef MCU_GIMBAL
// //         if (Gimbal_Rec != NULL)
// //             HC_Measure = Gimbal_Rec->distance;
// #endif
    }
}
