/**
* @file Ins_Task.c
 * @brief 获取陀螺仪任务
 * @author Shen FeiLin
 * @date 2025/11/9
 */

#include "usb_device.h"
#include "cmsis_os.h"
#include "INS.h"
#include "bsp_dwt.h"
#include "Vofa_Debug.h"

/**
 * @brief 陀螺仪任务
 * @note Init函数在Gimbal_Task中调用过了
 */
void INSTask(void const* argument)
{
    MX_USB_DEVICE_Init();
    for (;;)
    {
        // INS_Task();
        osDelay(1);
    }
}
