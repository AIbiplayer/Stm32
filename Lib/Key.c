/**
* @file Key.c
 * @brief 按键处理模块
 * @date 2025/10/4
 */

#include "Key.h"
#include  "stdbool.h"

KEY SW[4] = {
    {false, false, 0}, // SW2
    {false, false, 0}, // SW3
    {false, false, 0}, // SW4
    {false, false, 0} // SW5
}; // 按键结构体

GPIO_TypeDef* SW_GPIO_Port[4] = {SW2_GPIO_Port, SW3_GPIO_Port, SW4_GPIO_Port, SW5_GPIO_Port}; // 按键端口
uint16_t SW_Pin[4] = {SW2_Pin, SW3_Pin, SW4_Pin, SW5_Pin}; // 按键引脚
Control_Mode Chassis_Mode = NONE_MODE; // 控制模式

uint8_t Current_Key_Flag = 0; // 当前按键标志，用来判断按键的切换
uint8_t Last_Key_Flag = 0;
uint8_t Latest_Key_Flag = 0; // 最新按键单次标志
bool Mode_Change_Flag = false; //模式改变标志

extern uint8_t Chassis_Index;
extern uint8_t Inf_Interval;

/**
 * @brief 按键服务函数
 * @note 每100us调用一次
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    if (htim->Instance == TIM3)
    {
        Inf_Interval++;
        for (uint8_t i = 0; i < 4; i++) // 四个按键轮询
        {
            Last_Key_Flag = SW[i].Key_OnceFlag;

            SW[i].Key_PutStatus = SW[i].Key_Time > 100 ? (SW[i].Key_Time = 101) : false; // 按下时间大于10ms认为按下

            SW[i].Key_Time = HAL_GPIO_ReadPin(SW_GPIO_Port[i], SW_Pin[i]) == GPIO_PIN_RESET
                                 ? SW[i].Key_Time + 1
                                 : 0;

            SW[i].Key_PutStatus && HAL_GPIO_ReadPin(SW_GPIO_Port[i], SW_Pin[i]) == GPIO_PIN_SET
                ? (SW[i].Key_PutStatus = false, SW[i].Key_OnceFlag = !SW[i].Key_OnceFlag, SW[i].Key_Time = 0)
                : 0; // 按键单次标志

            Current_Key_Flag = SW[i].Key_OnceFlag;
            Current_Key_Flag != Last_Key_Flag ? (Latest_Key_Flag = i, Mode_Change_Flag = true) : 0; // 按键切换
        }
        if (SW[0].Key_OnceFlag && !SW[1].Key_OnceFlag) // SW2为真,SW3为假
        {
            Chassis_Mode = BLUETOOTH_MODE;
            HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, GPIO_PIN_RESET);
        }
        if (SW[1].Key_OnceFlag && !SW[0].Key_OnceFlag) // SW3为真,SW2为假
        {
            Chassis_Mode = INFRARE_MODE;
            HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, GPIO_PIN_SET);
        }
        if (SW[0].Key_OnceFlag && SW[1].Key_OnceFlag) // SW2和SW3都为真
        {
            switch (Latest_Key_Flag)
            {
            case 0:
                SW[1].Key_OnceFlag = false;
                break;
            case 1:
                SW[0].Key_OnceFlag = false;
                break;
            default:
                break;
            }
        }
        if (!SW[0].Key_OnceFlag && !SW[1].Key_OnceFlag) // SW2和SW3都为假
        {
            Chassis_Mode = NONE_MODE;
            HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, GPIO_PIN_RESET);
        }
    }
    if (Chassis_Mode == NONE_MODE)
    {
        SW[2].Key_OnceFlag
            ? (Chassis_Index = 1, HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, GPIO_PIN_SET))
            : (Chassis_Index = 0, HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, GPIO_PIN_RESET)); // SW4切换三轮和四轮
    }

    SW[3].Key_OnceFlag
        ? (HAL_GPIO_WritePin(Buzz_GPIO_Port, Buzz_Pin, GPIO_PIN_RESET))
        : (HAL_GPIO_WritePin(Buzz_GPIO_Port, Buzz_Pin, GPIO_PIN_SET));

    Last_Key_Flag = Current_Key_Flag;
}
