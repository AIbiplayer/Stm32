/**
 * @file PID.c
 * @brief PID控制程序
 * @author Shen Feilin
 * @date 2025/8/5
 */

#include "main.h"
#include "PID.h"
#include "math.h"

/**
 * @brief PID初始化及参数配置
 * @param PID_ 输入结构体
 * @param Kp_ 设定Kp
 * @param Ki_ 设定Ki
 * @param Kd_ 设定Kd
 * @param Imp_ 优化
 * @param DOUT_Filter_ 对微分滤波值，范围0~1
 * @param DeadZone_ 死区设置，低于死区不会执行PID
 * @param I_Limit_ 积分限幅
 * @param Max_Output_ 输出限幅
 */
void PID_Param(PID_Typedef *PID_, const float Kp_, const float Ki_, const float Kd_, const Improvement Imp_,
               const float DOUT_Filter_, float DeadZone_, float I_Limit_, float Max_Output_) {
    PID_->Kp = Kp_;
    PID_->Ki = Ki_;
    PID_->Kd = Kd_;
    PID_->DOut_Filter = DOUT_Filter_;
    PID_->Improve = Imp_;
    PID_->Dead_Zone = DeadZone_;
    PID_->I_Limit = I_Limit_;
    PID_->Max_Output = Max_Output_;
}

/**
 * @brief PID控制运算
 * @param PID_ 输入结构体
 * @param Target_ 目标值
 * @param Actual_ 实际值
 * @return 输出值
 */
float PID_Calculate(PID_Typedef *PID_, float Target_, float Actual_) {
    PID_->Last_Actual = PID_->Actual;
    PID_->Last_Error = PID_->Error;
    PID_->Last_DOut = PID_->Dout;
    PID_->Target = Target_;
    PID_->Actual = Actual_;
    PID_->Error = Target_ - Actual_;
    // 死区处理
    if (fabsf(PID_->Error) < PID_->Dead_Zone) {
        PID_->Output = 0.0f;
        return PID_->Output;
    }
    // 积分计算
    PID_->Error_Sum += PID_->Improve & Trapezoid_Intergral ? (PID_->Error + PID_->Last_Error) / 2 : PID_->Error;
    // 积分限幅
    if (PID_->Improve & Integral_Limit) {
        PID_->Error_Sum = fabsf(PID_->Error_Sum) > PID_->I_Limit
                              ? (int16_t) (PID_->Error_Sum) > 0
                                    ? PID_->I_Limit
                                    : -(PID_->I_Limit)
                              : PID_->Error_Sum;
    }
    // PID计算
    PID_->POut = PID_->Kp * PID_->Error;
    PID_->IOut = PID_->Ki * PID_->Error_Sum;
    PID_->Dout = PID_->Kd * (PID_->Error - PID_->Last_Error);
    // 微分先行
    if (PID_->Improve & Derivative_On_Measurement) {
        PID_->Dout = PID_->Kd * (PID_->Last_Actual - PID_->Actual);
    }
    if (PID_->Improve & OutputFilter) {
        PID_->Dout = PID_->DOut_Filter * PID_->Dout + (1 - PID_->DOut_Filter) * PID_->Last_DOut;
    }
    PID_->Output = PID_->POut + PID_->IOut + PID_->Dout;
    // 输出限幅
    PID_->Output = fabsf(PID_->Output) > PID_->Max_Output
                       ? (int32_t) PID_->Output > 0
                             ? PID_->Max_Output
                             : -PID_->Max_Output
                       : PID_->Output;
    return PID_->Output;
}

/**
 * @brief 清除PID积分
 * @param PID_ 输入结构体
 */
void PID_Clean_I(PID_Typedef *PID_) {
    PID_->Error_Sum = 0.0f;
    PID_->IOut = 0.0f;
}
