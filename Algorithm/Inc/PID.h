/**
* @file PID.h
 * @brief PID控制程序
 * @author Shen Feilin
 * @date 2025/8/5
 */

#ifndef PID_H
#define PID_H

#include "main.h"

typedef enum
{
    None = 0X00, //无
    Integral_Limit = 0x01, //积分限幅
    Derivative_On_Measurement = 0x02, //微分先行
    Trapezoid_Intergral = 0x04, //梯形积分
    OutputFilter = 0x10, //输出滤波
} Improvement;

typedef struct
{
    float Target;
    float Actual;
    float Last_Actual;
    float Error;
    float Last_Error;
    float Error_Sum;

    float Kp;
    float Ki;
    float Kd;

    float POut;
    float IOut;
    float Dout;
    float Last_DOut;

    float Output;
    float I_Limit; ///< 积分限幅，只能为正值
    float Max_Output; ///< 输出限幅，只能为正值
    float Dead_Zone;
    float DOut_Filter;

    Improvement Improve;
} PID_Typedef;

void PID_Param(PID_Typedef* PID_, const float Kp_, const float Ki_, const float Kd_, const Improvement Imp_,
               const float DOUT_Filter_, float DeadZone_, float I_Limit_, float Max_Output_);
float PID_Calculate(PID_Typedef* PID_, float Target_, float Actual_);
void PID_Clean_I(PID_Typedef* PID_);

#endif
