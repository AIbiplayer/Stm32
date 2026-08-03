/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2025-12-23 15:23:06
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2025-12-23 15:33:34
 * @FilePath: \MDK-ARMf:\Desktop\MotorDrivers_F407\MotorDrivers_F407\Motor\Inc\Servo.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __SERVO_H__
#define __SERVO_H__

#include <stdint.h>
#define YAW 1
#define PITCH 2
#define PITCH_CHANNEL TIM_CHANNEL_1
#define YAW_CHANNEL TIM_CHANNEL_2

void Servo_Init(void);
void Servo_Set_Auto(void);
void Servo_Set_Ble(void);
void Servo_Set_PS2(void);
#endif

