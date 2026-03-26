/*
 * @Descripttion:
 * @version:
 * @Author: Chenfu
 * @Date: 2022-12-02 21:32:47
 * @LastEditTime: 2022-12-05 15:29:49
 */
#include "super_cap.h"
#include "memory.h"
#include "bsp_can.h"
#include "stdlib.h"

static SuperCapInstance *super_cap_instance = NULL; // 可以由app保存此指针

static void SuperCap_RxData_NewDecode(uint8_t *rxbuff)
{
    SuperCap_RxData_New_s Msg;
    memcpy(&Msg, rxbuff, 8); //这里不用sizeof是担心改变结构体大小后导致指针越界
    super_cap_instance->cap_msg.outputABEnabled = Msg.statusCode & (1 << 7);
    super_cap_instance->cap_msg.useNewFeedbackMessage = Msg.statusCode & (1 << 6);
    super_cap_instance->cap_msg.wptStatus = (Msg.statusCode & (1 << 5)) | (Msg.statusCode & (1 << 4));
    super_cap_instance->cap_msg.limitFactor = (Msg.statusCode & (1 << 3)) | (Msg.statusCode & (1 << 2));
    super_cap_instance->cap_msg.errorLevel = (Msg.statusCode & (1 << 1)) | (Msg.statusCode & (1 << 0));
    super_cap_instance->cap_msg.chassisPower = (float)(Msg.chassisPower - 16384) / 64.0f; // 计算底盘实际功率值,单位W
    super_cap_instance->cap_msg.refereePower = (float)(Msg.refereePower - 16384) / 64.0f; // 裁判系统实际功率值,单位W
    super_cap_instance->cap_msg.chassisPowerLimit = Msg.chassisPowerLimit; // 底盘最大可用功率（包括裁判系统）
    super_cap_instance->cap_msg.capEnergy = (float)(Msg.capEnergy * 100) / 250; // 超级电容当前能量,百分比
}

static void SuperCap_RxData_Decode(uint8_t *rxbuff)
{
    SuperCap_RxData_s Msg;
    memcpy(&Msg, rxbuff, 8); //这里不用sizeof是担心改变结构体大小后导致指针越界
    super_cap_instance->cap_msg.outputABEnabled = Msg.statusCode & (1 << 7);
    super_cap_instance->cap_msg.useNewFeedbackMessage = Msg.statusCode & (1 << 6);
    super_cap_instance->cap_msg.wptStatus = (Msg.statusCode & (1 << 5)) | (Msg.statusCode & (1 << 4));
    super_cap_instance->cap_msg.limitFactor = (Msg.statusCode & (1 << 3)) | (Msg.statusCode & (1 << 2));
    super_cap_instance->cap_msg.errorLevel = (Msg.statusCode & (1 << 1)) | (Msg.statusCode & (1 << 0));
    super_cap_instance->cap_msg.chassisPower = Msg.chassisPower; // 底盘实际功率值,单位W
    super_cap_instance->cap_msg.refereePower = 0.0f; // 旧消息没有裁判系统功率反馈，置0
    super_cap_instance->cap_msg.chassisPowerLimit = Msg.chassisPowerLimit; // 底盘最大可用功率（包括裁判系统）
    super_cap_instance->cap_msg.capEnergy = (float)(Msg.capEnergy * 100) / 250; // 超级电容当前能量，百分比
}

static void SuperCapRxCallback(CANInstance *_instance)
{
    uint8_t *rxbuff;
    rxbuff = _instance->rx_buff;
    if ((rxbuff[0] & (0x01 << 6)) != 0) // 判断是否为新反馈消息
    {
        // 解析新版反馈消息
        SuperCap_RxData_NewDecode(rxbuff);
    }
    else
    {
        // 解析旧版反馈消息
        SuperCap_RxData_Decode(rxbuff);
    }

}

SuperCapInstance *SuperCapInit(SuperCap_Init_Config_s *supercap_config)
{
    super_cap_instance = (SuperCapInstance *)malloc(sizeof(SuperCapInstance));
    memset(super_cap_instance, 0, sizeof(SuperCapInstance));
    super_cap_instance->cap_tx_data.enableDCDC = supercap_config->enableDCDC;
    super_cap_instance->cap_tx_data.systemRestart = supercap_config->systemRestart;
    super_cap_instance->cap_tx_data.resv0 = 0;
    super_cap_instance->cap_tx_data.resv2 = 0;
    super_cap_instance->cap_tx_data.clearError = supercap_config->clearError;
    super_cap_instance->cap_tx_data.enableActiveChargingLimit = supercap_config->useNewFeedbackMessage;
    super_cap_instance->cap_tx_data.useNewFeedbackMessage = supercap_config->useNewFeedbackMessage;

    supercap_config->can_config.can_module_callback = SuperCapRxCallback;
    super_cap_instance->can_ins = CANRegister(&supercap_config->can_config);
    return super_cap_instance;
}

/**
 *
 * @param instance 要发送数据的超级电容实例
 * @param refereePowerLimit 当前裁判系统限制功率
 * @param refereeEnergyBuffer 当前缓冲能量
 * @param activeChargingLimitRatio 超级电容充满电量百分比, 只能是0到1的值
 */
void SuperCapSend(SuperCapInstance *instance, uint16_t refereePowerLimit, uint16_t refereeEnergyBuffer, float activeChargingLimitRatio)
{
    super_cap_instance->cap_tx_data.refereePowerLimit = refereePowerLimit;
    super_cap_instance->cap_tx_data.refereeEnergyBuffer = refereeEnergyBuffer;
    super_cap_instance->cap_tx_data.activeChargingLimitRatio = (uint8_t)(255 * activeChargingLimitRatio);
    memcpy(instance->can_ins->tx_buff, (uint8_t*)(&(super_cap_instance->cap_tx_data)), 8);
    CANTransmit(instance->can_ins,1);
}

SuperCap_Msg_s SuperCapGet(SuperCapInstance *instance)
{
    return instance->cap_msg;
}

void SuperCapClearError(SuperCapInstance *instance)
{
    instance->cap_tx_data.clearError = 1;
}

void SuperCapSystemRestart(SuperCapInstance *instance)
{
    instance->cap_tx_data.systemRestart = 1;
}