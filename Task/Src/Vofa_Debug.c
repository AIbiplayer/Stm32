/**
* @file Vofa_Debug.c
 * @brief VOFA调试程序
 * @author Shen FeiLin
 * @date 2025/11/1
 */

#include "bsp_usart.h"
#include "usart.h"
#include "remote_control.h"
#include "FreeRTOS.h"
#include "task.h"

extern volatile RC_ctrl_t rc_ctrl[2]; // 遥控器数据,初始化时返回

/**
 * @brief 打印遥控器数据
 */
void VOFA_Print_RC(void)
{

    Uart_printf(USART_TRANSFER_BLOCKING, &huart6, "%d,%d,%d,%d,%d,%d,%d\n",
                rc_ctrl[TEMP].rc.rocker_l_, rc_ctrl[TEMP].rc.rocker_l1,
                rc_ctrl[TEMP].rc.rocker_r_, rc_ctrl[TEMP].rc.rocker_r1,
                rc_ctrl[TEMP].rc.switch_left, rc_ctrl[TEMP].rc.switch_right,
                rc_ctrl[TEMP].rc.dial);
}
