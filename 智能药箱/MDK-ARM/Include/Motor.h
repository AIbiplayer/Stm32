/**
 * @brief 下方旋转舵机控制（TIM2）
 * @date 2025/4/17
 */

#ifndef MOTOR_H
#define MOTOR_H

void Rotate_Motor_Move(uint8_t CabinetList);
void Rotate_Motor_Up(void);
void Rotate_Motor_Down(void);
void Motor_Operation(uint8_t *List);

#endif
