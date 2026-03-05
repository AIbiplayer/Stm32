/*
 * @Descripttion:
 * @version:
 * @Author: Chenfu
 * @Date: 2022-12-02 21:32:47
 * @LastEditTime: 2022-12-05 15:25:46
 */
#ifndef SUPER_CAP_H
#define SUPER_CAP_H

#include <stdbool.h>

#include "bsp_can.h"

typedef enum
{
    REFEREE_POWER = 0,      // 裁判系统功率限制--正常功率模式--系统当前以满足裁判系统功率需求为主要控制目标
    CAPARR_VOLTAGE_MAX,     // 电容电压接近上限--电容电压最大限制--系统当前受电容电压上限保护限制
    CAPARR_VOLTAGE_NORMAL,  // 小功率充电模式--电容电压正常限制
    IB_POSITIVE,            // 充电电流越限--电容正电流限制（充电限制）
    IB_NEGATIVE,            // 放电电流越限--电容负电流限制（放电限制）
} LimitFactor;

typedef enum
{
    NO_ERROR = 0,               // 无错误
    ERROR_RECOVER_AUTO = 1,     // 错误，可通过自动恢复
    ERROR_RECOVER_MANUAL = 2,   // 错误，可通过发信息恢复
    ERROR_UNRECOVERABLE = 3,    // 错误，不可恢复
    WARNING                     // 警告
} ErrorLevel;

typedef enum
{
    WPT_ERROR = 0,      // 非无线充电硬件，或发生错误
    WPT_OFF = 1,        // 无线充电关闭
    WPT_CHARGING = 2,   // 无线充电中
    WPT_FINISHED = 3    // 无线充电完成(电压>98%, 能量大于96%)
} WPTStatus;

#pragma pack(1)

typedef struct  {
    uint8_t enableDCDC: 1;                  // 允许启动DCDC
    uint8_t systemRestart: 1;               // 系统重启
    uint8_t resv0: 3;                       // 保留位
    uint8_t clearError: 1;                  // 手动清除可清除的错误
    uint8_t enableActiveChargingLimit: 1;   // 是否启用主动充电限制
    uint8_t useNewFeedbackMessage: 1;       // 是否使用新的反馈消息格式

    uint16_t refereePowerLimit;             // 裁判限制功率，单位W
    uint16_t refereeEnergyBuffer;           // 裁判能量缓冲，单位J
    uint8_t activeChargingLimitRatio;       // 主动充电限制比例（能量），0-255
    int16_t resv2;
} SuperCap_TxData_s;

typedef struct{                     // 0x051 (useNewFeedbackMessage = 0)
    uint8_t statusCode;             // 状态信息
    float chassisPower;             // 底盘功率，单位W
    uint16_t chassisPowerLimit;     // 底盘最大可用功率（包括裁判系统）
    uint8_t capEnergy;              // 电容现有能量，0-250
} SuperCap_RxData_s;

typedef struct {                  // 0x052 (useNewFeedbackMessage = 1)
    uint8_t statusCode;             // 状态信息
    uint16_t chassisPower;          // 底盘功率，功率*64+16384 (-256W~+768W, 精度0.015625)
    uint16_t refereePower;          // 裁判系统功率，功率*64+16384 (-256W~+768W, 精度0.015625)
    uint16_t chassisPowerLimit;     // 底盘最大可用功率（包括裁判系统）
    uint8_t capEnergy;              // 电容现有能量，0-250
}  SuperCap_RxData_New_s;

/* 超级电容初始化配置 */
typedef struct
{
    uint8_t enableDCDC: 1;                  // 允许启动DCDC
    uint8_t systemRestart: 1;               // 系统重启
    uint8_t clearError: 1;                  // 手动清除可清除的错误
    uint8_t enableActiveChargingLimit: 1;   // 是否启用主动充电限制
    uint8_t useNewFeedbackMessage: 1;       // 是否使用新的反馈消息格式
    CAN_Init_Config_s can_config;
} SuperCap_Init_Config_s;

#pragma pack()

//    SuperCap_Msg_New_s cap_msg; // 超级电容信息

typedef struct
{
    bool outputABEnabled;               // 是否使能AB两侧输出，当它等于0时，功控板不工作
    bool useNewFeedbackMessage;         // 是否使用新的反馈信息格式，这个值用于区别TxData和TxDataNew类型反馈数据
    WPTStatus wptStatus;                // 无线充电状态。
    LimitFactor limitFactor;            // 功率限制因素
    ErrorLevel errorLevel;              // 错误等级
    float chassisPower;                 // 底盘功率，单位W
    float refereePower;                 // 裁判系统认为超限的功率，即当前等级功率上限，单位W
    uint16_t chassisPowerLimit;         // 超级电容可提供的最大功率加上裁判系统允许的最大功率，单位W
    float capEnergy;                  // 电容现有能量，百分比，0-100
}SuperCap_Msg_s;

/* 超级电容实例 */
typedef struct
{
    CANInstance *can_ins; // CAN实例
    SuperCap_RxData_New_s cap_rx_data; // 接收到的数据
    SuperCap_TxData_s cap_tx_data; // 超级电容发送数据
    SuperCap_Msg_s cap_msg; // 超级电容信息,即为反馈数据的解析结果
} SuperCapInstance;


/**
 * @brief 初始化超级电容
 *
 * @param supercap_config 超级电容初始化配置
 * @return SuperCapInstance* 超级电容实例指针
 */
SuperCapInstance *SuperCapInit(SuperCap_Init_Config_s *supercap_config);

/**
 *
 * @param instance 要发送数据的超级电容实例
 * @param refereePowerLimit 当前裁判系统限制功率
 * @param refereeEnergyBuffer 当前缓冲能量
 * @param activeChargingLimitRatio 超级电容充满电量百分比, 只能是0到100的值
 */
void SuperCapSend(SuperCapInstance *instance, uint16_t refereePowerLimit, uint16_t refereeEnergyBuffer, float activeChargingLimitRatio);

/**
 * @brief 清除超级电容错误
 *
 * @param instance 超级电容实例指针
 */
void SuperCapClearError(SuperCapInstance *instance);

/**
 * @brief 重启超级电容系统
 *
 * @param instance 超级电容实例指针
 */
void SuperCapSystemRestart(SuperCapInstance *instance);

#endif // !SUPER_CAP_Hd
