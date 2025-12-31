/**
* @file Ins_Task.c
 * @brief 获取陀螺仪任务
 * @author Shen FeiLin
 * @date 2025/11/9
 */

#include "usb_device.h"
#include "cmsis_os.h"
#include "INS.h"
#include "Vofa_Debug.h"
#include "HC_SR04.h"

extern INS_t* Gimbal_IMU_Data; ///< 云台IMU数据

float HC_Measure = 0.0f; ///< 超声波测量值
static uint8_t Count = 0; ///< 计数器

/**
 * @brief 陀螺仪任务
 * @note Init函数在Gimbal_Task中调用过了
 */
void INSTask(void const* argument)
{
    taskENTER_CRITICAL();
    MX_USB_DEVICE_Init();
    Gimbal_IMU_Data = INS_Init();
    HC_Init();
    LED_Green_Up;
    taskEXIT_CRITICAL();
    for (;;)
    {
        INS_Task();
        osDelay(1);
        Count == 1 ? HC_Send_Trig() : Count > 20 ? (HC_Measure = HC_Get_Measure(), Count = 0) : 0;
        Count++;
    }
}
