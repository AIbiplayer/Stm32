/**
* @file DM_Motor.c
 * @brief 达妙电机驱动模块
 * @author Shen FeiLin
 * @date 2025/12/13
 * @note 借鉴大疆电机函数，稍有改动
 */

#include "DM_Motor.h"

#include <sys/types.h>

#include "cmsis_os.h"
#include "main.h"
#include "math.h"
#include "daemon.h"
#include "stdbool.h"
#include "robot_def.h"
#include "string.h"
#include "stdlib.h"

static uint8_t Idx = 0; ///< 电机索引
static uint8_t Setting_Buffer[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00}; ///< 使能、失能等使用

// 根据达妙手册填写
static CANInstance sender_assignment = {
    .can_handle = &hcan2, .txconf.StdId = 0x3FE, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA,
    .txconf.DLC = 0x08, .tx_buff = {0}
}; ///< 达妙电机发送CAN实例，达妙电机协议为1拖4，后续可以根据协议进行调整

static DM_Motor_Instance *DM_Instance_Group[DM_MOTOR_CNT]; ///< 把所有电机实例放到一个组，之后统一进行PID计算，注意这里是指针类型

static void Decode_DM_Motor(CANInstance *Instance);

static float uint_to_float(int x_int, float x_min, float x_max, int bits);

/**
 * @brief 注册达妙电机实例
 * @return 电机实例指针
 * @todo 在达妙1拖4协议中，电机ID为0-3，CAN ID为0x201-0x204，这里需要根据协议进行设置
 */
DM_Motor_Instance *DM_Motor_Init(DM_Motor_Init_s *Motor_Init) {
    DM_Motor_Instance *Instance = (DM_Motor_Instance *) malloc(sizeof(DM_Motor_Instance)); //创建动态内存，便于创造实例
    memset(Instance, 0, sizeof(DM_Motor_Instance));
    Instance->Control_Setting = Motor_Init->Control_Setting; //将电机部分内容转移

    Motor_Init->Can_Init_Config.rx_id = 0x300 + Motor_Init->Can_Init_Config.tx_id; //根据协议设置CAN ID
    Instance->id = Motor_Init->Can_Init_Config.rx_id; //电机ID

    Motor_Init->Can_Init_Config.can_module_callback = Decode_DM_Motor; //注册电机到CAN总线
    Motor_Init->Can_Init_Config.id = Instance;
    Instance->Motor_Can_Instance = CANRegister(&Motor_Init->Can_Init_Config);
    DM_Instance_Group[Idx++] = Instance;
    return Instance;
}

/**
 * @brief 对达妙电机进行控制并发送CAN
 * @todo 这里为1拖4控制，和DJI一样
 */
void DM_Motor_Control(void) {
    //对已注册的电机进行控制
    for (uint8_t i = 0; i < Idx; i++) {
        DM_Motor_Instance *DM_Instance = DM_Instance_Group[i]; //使用指针提取电机实例
        float PID_Ref = DM_Instance->Control_Setting.Target; //PID参考值
        const DM_Motor_Measure_s Measure = DM_Instance->Measure; //电机测量值

        float PID_Measure_Speed = 0.0f; //PID速度测量值
        float PID_Measure_Angle = 0.0f; //PID角度测量值

        if (DM_Instance->Control_Setting.Reverse_Flag == MOTOR_REVERSE) //判断取反
            PID_Ref *= -1;

        PID_Measure_Speed = DM_Instance->Control_Setting.Speed_Feedback_Source == OTHER_FEEDBACK
                            && DM_Instance->Control_Setting.Other_Speed_Feedback_Ptr != NULL
                                ? *DM_Instance->Control_Setting.Other_Speed_Feedback_Ptr
                                : Measure.speed;

        PID_Measure_Angle = DM_Instance->Control_Setting.Angle_Feedback_Source == OTHER_FEEDBACK
                            && DM_Instance->Control_Setting.Other_Angle_Feedback_Ptr != NULL
                                ? *DM_Instance->Control_Setting.Other_Angle_Feedback_Ptr
                                : Measure.angle;

        switch (DM_Instance->Control_Setting.Loop_Control) // 按闭环控制类型进行控制
        {
            case SPEED_CONTROL:
                PID_Ref = PID_Calculate(&DM_Instance->Control_Setting.Speed_PID, PID_Ref, PID_Measure_Speed);
                break;
            case ANGLE_CONTROL:
                PID_Ref = PID_Calculate(&DM_Instance->Control_Setting.Angle_PID, PID_Ref, PID_Measure_Angle);
                break;
            case ANGLE_SPEED_CONTROL:
                PID_Ref = PID_Calculate(&DM_Instance->Control_Setting.Angle_PID, PID_Ref, PID_Measure_Angle);
                PID_Ref = PID_Calculate(&DM_Instance->Control_Setting.Speed_PID, PID_Ref, PID_Measure_Speed);
                break;
            case SPEED_ANGLE_CONTROL:
                PID_Ref = PID_Calculate(&DM_Instance->Control_Setting.Speed_PID, PID_Ref, PID_Measure_Speed);
                PID_Ref = PID_Calculate(&DM_Instance->Control_Setting.Angle_PID, PID_Ref, PID_Measure_Angle);
                break;
            default: break;
        }
        PID_Ref = DM_Instance->Control_Setting.Feedforward_Flag == CURRENT_FEEDFORWARD
                  && DM_Instance->Control_Setting.Feedforward_Ptr != NULL
                      ? PID_Ref + *DM_Instance->Control_Setting.Feedforward_Ptr
                      : PID_Ref;

        DM_Instance->Control_Setting.Power_Output = (int16_t) PID_Ref;
        sender_assignment.tx_buff[(DM_Instance->id - 1) * 2 + 1] = (uint8_t) (
            DM_Instance->Control_Setting.Power_Output >> 8);
        sender_assignment.tx_buff[(DM_Instance->id - 1) * 2] = (uint8_t) DM_Instance->Control_Setting.Power_Output;

        if (DM_Instance->Control_Setting.Work_Type == MOTOR_STOP)
            memset(sender_assignment.tx_buff + DM_Instance->id * 2, 0, 16u);
    }
    CANTransmit(&sender_assignment, 1);
}

/**
 * @brief 达妙电机解析函数
 * @note 修改测量范围在这里
 */
void Decode_DM_Motor(CANInstance *Instance) {
    const uint8_t *Rx_Buff = Instance->rx_buff;
    DM_Motor_Instance *DM_Instance = Instance->id;
    DM_Motor_Measure_s *Measure = &DM_Instance->Measure;
    Measure->ecd = (uint16_t) Rx_Buff[0] << 8 | Rx_Buff[1];
    Measure->angle = ((float) Measure->ecd * ECD_ANGLE_COEF_DJI);
    Measure->speed = (int16_t) (Rx_Buff[2] << 8 | Rx_Buff[3]);
    Measure->speed /= 100; // 达妙电机速度单位为rpm，除以100进行换算
    Measure->current = (1.0f - CURRENT_SMOOTH_COEF) * Measure->current +
                       CURRENT_SMOOTH_COEF * (float) ((int16_t) (Rx_Buff[4] << 8 | Rx_Buff[5]));
    Measure->error = (DM_error_e) Rx_Buff[7];
}

void Decode_dm_imu(CANInstance *Instance) {
    const uint8_t *Rx_Buff = Instance->rx_buff;
    DM_IMU_Instance_s *DM_IMU_Instance = Instance->id;
    DM_IMU_Measure_s *Measure = &DM_IMU_Instance->Measure;
    switch (Rx_Buff[0]) {
        case 0x02: // Gyro
            for (uint8_t i = 1; i < 4; i++) {
                Measure->Gyro[i - 1] = uint_to_float((uint16_t) (Rx_Buff[i * 2 + 1] << 8 | Rx_Buff[i * 2]),
                                                     -2000.0f, 2000.0f, 16);
            }
            break;
        case 0x03: // Roll, Pitch, Yaw
            Measure->Roll = uint_to_float((uint16_t) (Rx_Buff[7] << 8 | Rx_Buff[6]), -180.0f, 180.0f, 16);
            Measure->Pitch = uint_to_float((uint16_t) (Rx_Buff[3] << 8 | Rx_Buff[2]), -180.0f, 180.0f, 16);
            Measure->Yaw = uint_to_float((uint16_t) (Rx_Buff[5] << 8 | Rx_Buff[4]), -180.0f, 180.0f, 16);
            break;
        default: break;
    }
}

/**
 * @brief 达妙电机离线处理函数
 * @note 这里简单地将工作类型设置为停止，后续可以根据需要进行优化，比如增加报警等
 */
void DM_motor_offline(void *owner_id) {
    DM_Motor_Instance *DM_Instance = (DM_Motor_Instance *) owner_id;
    DM_Instance->Control_Setting.Work_Type = MOTOR_STOP;
}

/**
************************************************************************
* @brief:      	float_to_uint: 浮点数转换为无符号整数函数
* @param[in]:   x_float:	待转换的浮点数
* @param[in]:   x_min:		范围最小值
* @param[in]:   x_max:		范围最大值
* @param[in]:   bits: 		目标无符号整数的位数
* @retval:     	无符号整数结果
* @details:    	将给定的浮点数 x 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个指定位数的无符号整数
************************************************************************
**/
static int float_to_uint(float x_float, float x_min, float x_max, int bits) {
    /* Converts a float to an unsigned int, given range and number of bits */
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x_float - offset) * ((float) ((1 << bits) - 1)) / span);
}

/**
************************************************************************
* @brief:      	uint_to_float: 无符号整数转换为浮点数函数
* @param[in]:   x_int: 待转换的无符号整数
* @param[in]:   x_min: 范围最小值
* @param[in]:   x_max: 范围最大值
* @param[in]:   bits:  无符号整数的位数
* @retval:     	浮点数结果
* @details:    	将给定的无符号整数 x_int 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个浮点数
************************************************************************
**/
static float uint_to_float(int x_int, float x_min, float x_max, int bits) {
    /* converts unsigned int to float, given range and number of bits */
    float span = x_max - x_min;
    float offset = x_min;
    return ((float) x_int) * span / ((float) ((1 << bits) - 1)) + offset;
}

/**
 * @brief 电机启动
 */
void DM_MotorEnable(void) {
    for (uint8_t i = 0; i < Idx; i++) {
        DM_Motor_Instance *DM_Instance = DM_Instance_Group[i];
        DM_Instance->Control_Setting.Work_Type = MOTOR_ENABLE;
    }
    // Setting_Buffer[7] = 0xFC;
    // memcpy(motor->Motor_Can_Instance->tx_buff, Setting_Buffer, sizeof(Setting_Buffer));
    // CANTransmit(motor->Motor_Can_Instance, 1);
}

/**
 * @brief 电机停止
 */
void DM_MotorStop(void) {
    for (uint8_t i = 0; i < Idx; i++) {
        DM_Motor_Instance *DM_Instance = DM_Instance_Group[i];
        DM_Instance->Control_Setting.Work_Type = MOTOR_STOP;
    }
    // Setting_Buffer[7] = 0xFD;
    // memcpy(motor->Motor_Can_Instance->tx_buff, Setting_Buffer, sizeof(Setting_Buffer));
    // CANTransmit(motor->Motor_Can_Instance, 1);
}

/**
 * @brief 电机校准零点
 */
void DM_MotorSaveZero(DM_Motor_Instance *motor) {
    Setting_Buffer[7] = 0xFE;
    memcpy(motor->Motor_Can_Instance->tx_buff, Setting_Buffer, sizeof(Setting_Buffer));
    CANTransmit(motor->Motor_Can_Instance, 1);
}

/**
 * @brief 电机校准零点
 */
void DM_MotorClearError(DM_Motor_Instance *motor) {
    Setting_Buffer[7] = 0xFB;
    memcpy(motor->Motor_Can_Instance->tx_buff, Setting_Buffer, sizeof(Setting_Buffer));
    CANTransmit(motor->Motor_Can_Instance, 1);
}

/**
 * @brief 改变电机旋转方向
 */
void DM_MotorChangeReverse(DM_Motor_Instance *motor, const Motor_Reverse_Flag_e motor_reverse_flag) {
    motor->Control_Setting.Reverse_Flag = motor_reverse_flag;
}

/**
 * @brief 弧度角度换算
 * @return 弧度值
 */
static float rad_to_degree(float rad) {
    float degree = (float) (rad * 180.0f / M_PI);
    return degree;
}

/**
 * @brief 设置目标值
 * @param motor 电机实例指针
 */
void DM_MotorSet(DM_Motor_Instance *motor, float Target_) {
    motor->Control_Setting.Target = Target_;
}
