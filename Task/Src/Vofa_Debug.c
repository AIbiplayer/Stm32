/**
* @file Vofa_Debug.c
 * @brief VOFA调试程序
 * @author Shen FeiLin
 * @date 2025/11/1
 * @note 现在发现FreeMaster比Vofa更好用，嘻嘻
 */

#include "Vofa_Debug.h"
#include "bsp_usart.h"
#include "usart.h"
#include "DJI_Motor.h"
#include "remote_control.h"
#include "INS.h"

extern RC_ctrl_t rc_ctrl[2]; // 遥控器数据,初始化时返回
extern INS_t INS;

/**
 * @brief 打印遥控器数据
 */
void VOFA_Print_RC(void)
{
    Uart_printf(USART_TRANSFER_DMA, &huart6, "%d,%d,%d,%d,%d,%d,%d\n",
                rc_ctrl[TEMP].rc.rocker_l_, rc_ctrl[TEMP].rc.rocker_l1,
                rc_ctrl[TEMP].rc.rocker_r_, rc_ctrl[TEMP].rc.rocker_r1,
                rc_ctrl[TEMP].rc.switch_left, rc_ctrl[TEMP].rc.switch_right,
                rc_ctrl[TEMP].rc.dial);
}

/**
 * @brief 打印陀螺仪数据
 */
void VOFA_Print_INS(void)
{
    Uart_printf(USART_TRANSFER_DMA, &huart6, "%d,%d,%d\n", (int16_t)INS.Yaw, (int16_t)INS.Pitch, (int16_t)INS.Roll);
}
