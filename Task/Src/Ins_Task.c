/**
* @file Ins_Task.c
 * @brief 获取陀螺仪任务
 * @author Shen FeiLin
 * @date 2025/11/9
 */

#include "usb_device.h"
#include "cmsis_os.h"
#include "INS.h"
#include "HC_SR04.h"
#include "tim.h"

static uint16_t Count = 0;
float HC_Measure = 0;
/**
 * @brief 陀螺仪任务
 * @note Init函数在Gimbal_Task中调用过了
 */
void INSTask(void const* argument)
{
    MX_USB_DEVICE_Init();
    HC_Init();
    for (;;)
    {
        INS_Task();
        osDelay(1);

        Count == 1 ? HC_Send_Trig() : Count > 20 ? (HC_Measure = HC_Get_Measure(), Count = 0) : 0;
        Count++;
    }
}
