/**
 * @file DJT_Motor.c
 * @brief 大疆电机控制
 * @author Shen Feilin
 * @date 2025/10/24
 */

#include "DJI_Motor.h"

#include "bsp_dwt.h"
#include <stdlib.h>
#include <string.h>
#include "stdbool.h"
#include "daemon.h"

static uint8_t Idx = 0; ///< 电机索引
static bool Send_Enable_Flag[6]; ///< 六组电机发送标志位，哪一个为True说明哪一个可以发送
static DJI_Motor_Instance* Instance_Group[DJI_MOTOR_CNT]; ///< 把所有电机实例放到一个组，之后统一进行PID计算，注意这里是指针类型

/**
 * @brief 由于DJI电机发送以四个一组的形式进行,故对其进行特殊处理,用6个(2can*3group)can_instance专门负责发送
 *        该变量将在 DJIMotorControl() 中使用,分组在 MotorSenderGrouping()中进行
 * @note  因为只用于发送,所以不需要在bsp_can中注册
 * C610(m2006)/C620(m3508):0x200,0x1ff,0x200;
 * GM6020:0x1fe,0x2fe
 * 反馈(rx_id): GM6020: 0x204+id ; C610/C620: 0x200+id
 * can1: [0]:0x1Ff,[1]:0x200,[2]:0x2FF
 * can2: [3]:0x1Fe,[4]:0x200,[5]:0x2FF
 */
static CANInstance sender_assignment[6] = {
    [0] = {
        .can_handle = &hcan1, .txconf.StdId = 0x1fe, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA,
        .txconf.DLC = 0x08, .tx_buff = {0}
    },
    [1] = {
        .can_handle = &hcan1, .txconf.StdId = 0x200, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA,
        .txconf.DLC = 0x08, .tx_buff = {0}
    },
    [2] = {
        .can_handle = &hcan1, .txconf.StdId = 0x2ff, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA,
        .txconf.DLC = 0x08, .tx_buff = {0}
    },
    [3] = {
        .can_handle = &hcan2, .txconf.StdId = 0x1fe, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA,
        .txconf.DLC = 0x08, .tx_buff = {0}
    },
    [4] = {
        .can_handle = &hcan2, .txconf.StdId = 0x200, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA,
        .txconf.DLC = 0x08, .tx_buff = {0}
    },
    [5] = {
        .can_handle = &hcan2, .txconf.StdId = 0x2ff, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA,
        .txconf.DLC = 0x08, .tx_buff = {0}
    },
};

static void Decode_DJI_Motor(CANInstance* Instance);
static void Motor_Grouping(DJI_Motor_Instance* DJI_Motor, CAN_Init_Config_s* Config);

/**
 * @brief 注册大疆电机实例
 * @return 电机实例指针
 */
DJI_Motor_Instance* DJI_Motor_Init(Motor_Init_s* Motor_Init)
{
    DJI_Motor_Instance* Instance = (DJI_Motor_Instance*)malloc(sizeof(DJI_Motor_Instance)); //创建动态内存，便于创造实例
    memset(Instance, 0, sizeof(DJI_Motor_Instance));
    Instance->Control_Setting = Motor_Init->Control_Setting; //将电机部分内容转移
    Instance->Motor_Type = Motor_Init->Motor_Type;

    Motor_Grouping(Instance, &Motor_Init->Can_Init_Config);

    Motor_Init->Can_Init_Config.can_module_callback = Decode_DJI_Motor; //注册电机到CAN总线
    Motor_Init->Can_Init_Config.id = Instance;
    Instance->Motor_Can_Instance = CANRegister(&Motor_Init->Can_Init_Config);

    DJI_MotorEnable(Instance);
    Instance_Group[Idx++] = Instance;

    return Instance;
}

/**
 * @brief 根据手册查找电机ID并对其分组，方便管理
 */
static void Motor_Grouping(DJI_Motor_Instance* DJI_Motor, CAN_Init_Config_s* Config)
{
    const uint8_t Motor_ID = Config->tx_id - 1;
    uint8_t InGroup_ID;
    uint8_t Group;

    switch (DJI_Motor->Motor_Type)
    {
    case M3508:
        Group = 1;
        InGroup_ID = Motor_ID;
        Config->rx_id = 0x200 + Motor_ID + 1;
        Send_Enable_Flag[Group] = true;
        DJI_Motor->Send_Group = Group;
        DJI_Motor->Message_Num = InGroup_ID;
        break;
    default:
        break;
    }
}

/**
 * @brief 大疆电机解析数据
 */
static void Decode_DJI_Motor(CANInstance* Instance)
{
    /*这里将Instance的ID强转为DJI_Motor，是为了将CANInstance特有的功能拼接给DJI_Motor
     *可以把CANInstance理解为拓展拼图，其中的void*为拼图上凸起的接口*/
    const uint8_t* Rx_Buff = Instance->rx_buff;
    DJI_Motor_Instance* DJI_Instance = (DJI_Motor_Instance*)Instance->id;
    DJI_Motor_Measure_s* Measure = &DJI_Instance->Measure;

    DaemonReload(DJI_Instance->Daemon);

    DJI_Instance->dt = DWT_GetDeltaT(&DJI_Instance->Feed_Cnt);

    Measure->Last_Ecd = Measure->Ecd;
    Measure->Ecd = (uint16_t)Rx_Buff[0] << 8 | Rx_Buff[1];
    Measure->Angle = ((float)Measure->Ecd * ECD_ANGLE_COEF_DJI);
    Measure->Speed = (int16_t)(Rx_Buff[2] << 8 | Rx_Buff[3]);
    Measure->Current = (1.0f - CURRENT_SMOOTH_COEF) * Measure->Current +
        CURRENT_SMOOTH_COEF * (float)((int16_t)(Rx_Buff[4] << 8 | Rx_Buff[5]));
    Measure->Temp = Rx_Buff[6];

    /* 这里添加多圈记录 */
}

/**
 * @brief 对大疆电机进行控制并发送CAN
 */
void DJI_Motor_Control(void)
{
    //对已注册的电机进行控制
    for (uint8_t i = 0; i < Idx; i++)
    {
        const DJI_Motor_Instance* DJI_Instance = Instance_Group[i]; //使用指针提取电机实例
        Motor_Control_Setting_s Control_Setting = DJI_Instance->Control_Setting;
        const DJI_Motor_Measure_s Measure = DJI_Instance->Measure; //电机测量值
        float PID_Ref = Control_Setting.Target; //保存设定值，防止PID进行中目标值被修改

        if (Control_Setting.Reverse_Flag == MOTOR_REVERSE) //判断取反
            PID_Ref *= -1;
        switch (Control_Setting.Loop_Control) // 按闭环控制类型进行控制
        {
        case SPEED_CONTROL:
            PID_Ref = PID_Calculate(&Control_Setting.Speed_PID, PID_Ref, Measure.Speed);
            break;
        case ANGLE_CONTROL:
            PID_Ref = PID_Calculate(&Control_Setting.Angle_PID, PID_Ref, Measure.Angle);
            break;
        case ANGLE_SPEED_CONTROL:
            PID_Ref = PID_Calculate(&Control_Setting.Angle_PID, PID_Ref, Measure.Angle);
            PID_Ref = PID_Calculate(&Control_Setting.Speed_PID, PID_Ref, Measure.Speed);
            break;
        case SPEED_ANGLE_CONTROL:
            PID_Ref = PID_Calculate(&Control_Setting.Speed_PID, PID_Ref, Measure.Speed);
            PID_Ref = PID_Calculate(&Control_Setting.Angle_PID, PID_Ref, Measure.Angle);
            break;
        default: break;
        }

        /* 在这里增加前馈 */

        const int16_t Set = (int16_t)PID_Ref; //CAN发送的设定值
        const uint8_t Group = DJI_Instance->Send_Group;
        const uint8_t InGroup_ID = DJI_Instance->Message_Num;

        sender_assignment[Group].tx_buff[2 * InGroup_ID] = (uint8_t)(Set >> 8); //高八位
        sender_assignment[Group].tx_buff[2 * InGroup_ID + 1] = (uint8_t)Set; //低八位

        if (DJI_Instance->Working_Type == MOTOR_STOP)
            memset(sender_assignment[Group].tx_buff + 2 * InGroup_ID, 0, sizeof(sender_assignment[Group]));
    }
    for (uint8_t i = 0; i < 6; i++)
    {
        if (Send_Enable_Flag[i])
            CANTransmit(&sender_assignment[i], 100);
    }
}

/**
 * @brief 电机停止
 */
void DJI_MotorStop(DJI_Motor_Instance* motor)
{
    motor->Working_Type = MOTOR_STOP;
}

/**
 * @brief 电机启动
 */
void DJI_MotorEnable(DJI_Motor_Instance* motor)
{
    motor->Working_Type = MOTOR_ENABLE;
}

/**
 * @brief 修改电机控制环
 */
void DJI_MotorChangeLoop(DJI_Motor_Instance* motor, const Motor_Loop_Control_Type_e Loop)
{
    motor->Control_Setting.Loop_Control = Loop;
}

/**
 * @brief 改变电机旋转方向
 */
void DJI_MotorChangeReverse(DJI_Motor_Instance* motor, const Motor_Reverse_Flag_e motor_reverse_flag)
{
    motor->Control_Setting.Reverse_Flag = motor_reverse_flag;
}

/**
 * @brief 设置PID目标值
 */
void DJI_MotorSetTarget(DJI_Motor_Instance* motor, const float Target_)
{
    motor->Control_Setting.Target = Target_;
}
