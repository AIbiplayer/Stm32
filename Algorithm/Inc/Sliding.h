/**
* @file Sliding.h
 * @brief 滑动平均滤波算法实现
 * @see https://github.com/GreenHand314/Sliding.git
 * @date 2025/11/7
 */

#ifndef SLIDING_H
#define SLIDING_H

#define SAMPLE_PERIOD 0.002///<采样周期2ms
#define V_EORROR_INTEGRAL_MAX 2000///<速度积分限幅
#define P_EORROR_INTEGRAL_MAX 2000///<位置积分限幅

typedef enum
{
    EXPONENT = 0, ///<指数型滑模，适用Yaw角控制
    POWER, ///<幂次型滑模,适用位置控制
    TFSMC, ///<二次型滑模,适用角度控制
    VELSMC, ///<速度型滑模,适用速度控制,仅含比例项
    EISMC ///<积分型滑模，适用Pitch、拨弹盘控制
} SMC_Mode_e;



#endif //SLIDING_H
