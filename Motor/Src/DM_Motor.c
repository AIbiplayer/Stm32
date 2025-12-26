/**
* @file DM_Motor.c
 * @brief 达妙电机驱动模块
 * @author Shen FeiLin
 * @date 2025/12/13
 * @note 借鉴大疆电机函数，稍有改动
 */

#include "DM_Motor.h"
#include "main.h"
#include "math.h"
#include "stdbool.h"
#include "robot_def.h"
#include "string.h"
#include "stdlib.h"

static uint8_t Idx = 0; ///< 电机索引
static uint8_t Setting_Buffer[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00}; ///< 使能、失能等使用
static DM_Motor_Instance* DM_Instance_Group[DM_MOTOR_CNT]; ///< 把所有电机实例放到一个组，之后统一进行PID计算，注意这里是指针类型

static void Decode_DM_Motor(CANInstance* Instance);
static int float_to_uint(float x_float, float x_min, float x_max, int bits);
static float uint_to_float(int x_int, float x_min, float x_max, int bits);
static float degree_to_rad(int16_t degree);

/**
 * @brief 注册大疆电机实例
 * @return 电机实例指针
 */
DM_Motor_Instance* DM_Motor_Init(DM_Motor_Init_s* Motor_Init)
{
    DM_Motor_Instance* Instance = (DM_Motor_Instance*)malloc(sizeof(DM_Motor_Instance)); //创建动态内存，便于创造实例
    memset(Instance, 0, sizeof(DM_Motor_Instance));
    Instance->Control_Setting = Motor_Init->DM_Control; //将电机部分内容转移
    Instance->Work_Type = Motor_Init->Working_Type;

    Motor_Init->Can_Init_Config.tx_id = Motor_Init->Mode + Motor_Init->Can_Init_Config.rx_id;
    Motor_Init->Can_Init_Config.rx_id -= 1;

    Motor_Init->Can_Init_Config.can_module_callback = Decode_DM_Motor; //注册电机到CAN总线
    Motor_Init->Can_Init_Config.id = Instance;
    Instance->Motor_Can_Instance = CANRegister(&Motor_Init->Can_Init_Config);
    DM_Instance_Group[Idx++] = Instance;
    return Instance;
}

/**
 * @brief 对达妙电机进行控制并发送CAN
 * @todo 这里只控制位置速度模式，也就是说Target默认为角度，其它模式控制以后再添加
 */
void DM_Motor_Control(void)
{
    //对已注册的电机进行控制
    for (uint8_t i = 0; i < Idx; i++)
    {
        const DM_Motor_Instance* DM_Instance = DM_Instance_Group[i]; //使用指针提取电机实例
        DM_Control_Setting_s Control_Setting = DM_Instance->Control_Setting;
        if (Control_Setting.a_target_last == Control_Setting.a_target)
            continue; //目标值未变化则不发送
        if (Control_Setting.Reverse_Flag == MOTOR_REVERSE) //判断取反
            Control_Setting.a_target *= -1;
        Control_Setting.a_target = degree_to_rad((int16_t)Control_Setting.a_target);

        uint8_t* a_ptr = (uint8_t*)&Control_Setting.a_target;
        uint8_t* v_ptr = (uint8_t*)&Control_Setting.v_target;
        DM_Instance->Motor_Can_Instance->tx_buff[0] = *a_ptr;
        DM_Instance->Motor_Can_Instance->tx_buff[1] = *(a_ptr + 1);
        DM_Instance->Motor_Can_Instance->tx_buff[2] = *(a_ptr + 2);
        DM_Instance->Motor_Can_Instance->tx_buff[3] = *(a_ptr + 3);
        DM_Instance->Motor_Can_Instance->tx_buff[4] = *v_ptr;
        DM_Instance->Motor_Can_Instance->tx_buff[5] = *(v_ptr + 1);
        DM_Instance->Motor_Can_Instance->tx_buff[6] = *(v_ptr + 2);
        DM_Instance->Motor_Can_Instance->tx_buff[7] = *(v_ptr + 3);

        if (DM_Instance->Work_Type == MOTOR_ENABLE)
            CANTransmit(DM_Instance->Motor_Can_Instance, 1);
    }
}

/**
 * @brief 达妙电机解析函数
 * @note 修改测量范围在这里
 */
void Decode_DM_Motor(CANInstance* Instance)
{
    const uint8_t* Rx_Buff = Instance->rx_buff;
    DM_Motor_Instance* DM_Instance = Instance->id;
    DM_Motor_Measure_s* Measure = &DM_Instance->Measure;

    if ((Rx_Buff[0] & 0x0F) + 0x10 != Instance->rx_id)
        return;

    Measure->id = Rx_Buff[0] & 0x0F;
    Measure->state = Rx_Buff[0] >> 4;
    //@note 这里修改范围
    Measure->pos = uint_to_float((uint16_t)((Rx_Buff[1] << 8) | Rx_Buff[2]), -12.5f, 12.5f, 16);
    Measure->vel = uint_to_float((uint16_t)((Rx_Buff[3] << 4) | Rx_Buff[4] >> 4), -45.0f, 45.0f, 12);
    Measure->tor = uint_to_float((uint16_t)(((Rx_Buff[4] & 0x0F) << 8) | Rx_Buff[5]), -18.0f, 18.0f, 12);

    DM_Instance->Work_Type = Measure->state == 0 ? MOTOR_STOP : MOTOR_ENABLE;
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
static int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
    /* Converts a float to an unsigned int, given range and number of bits */
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x_float - offset) * ((float)((1 << bits) - 1)) / span);
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
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    /* converts unsigned int to float, given range and number of bits */
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

/**
 * @brief 电机启动
 */
void DM_MotorEnable(DM_Motor_Instance* motor)
{
    Setting_Buffer[7] = 0xFC;
    memcpy(motor->Motor_Can_Instance->tx_buff, Setting_Buffer, sizeof(Setting_Buffer));
    CANTransmit(motor->Motor_Can_Instance, 1);
}

/**
 * @brief 电机停止
 */
void DM_MotorStop(DM_Motor_Instance* motor)
{
    Setting_Buffer[7] = 0xFD;
    memcpy(motor->Motor_Can_Instance->tx_buff, Setting_Buffer, sizeof(Setting_Buffer));
    CANTransmit(motor->Motor_Can_Instance, 1);
}

/**
 * @brief 电机校准零点
 */
void DM_MotorSaveZero(DM_Motor_Instance* motor)
{
    Setting_Buffer[7] = 0xFE;
    memcpy(motor->Motor_Can_Instance->tx_buff, Setting_Buffer, sizeof(Setting_Buffer));
    CANTransmit(motor->Motor_Can_Instance, 1);
}

/**
 * @brief 电机校准零点
 */
void DM_MotorClearError(DM_Motor_Instance* motor)
{
    Setting_Buffer[7] = 0xFB;
    memcpy(motor->Motor_Can_Instance->tx_buff, Setting_Buffer, sizeof(Setting_Buffer));
    CANTransmit(motor->Motor_Can_Instance, 1);
}

/**
 * @brief 改变电机旋转方向
 */
void DM_MotorChangeReverse(DM_Motor_Instance* motor, const Motor_Reverse_Flag_e motor_reverse_flag)
{
    motor->Control_Setting.Reverse_Flag = motor_reverse_flag;
}

/**
 * @brief 弧度角度换算
 * @return 弧度值
 */
static float degree_to_rad(int16_t degree)
{
    degree = abs(degree) > 720 * REDUCTION_TRACK * REDUCTION_RATIO_WHEEL
                 ? 720 * REDUCTION_TRACK * REDUCTION_RATIO_WHEEL * abs(degree) / degree
                 : degree;
    float rad = (float)degree * M_PI / 180.0f;
    return rad;
}

/**
 * @brief 设置目标值
 * @param motor 电机实例指针
 * @param aTarget_ 目标角度 单位：度
 * @param vTarget_ 目标速度 单位：弧度每秒
 */
void DM_MotorSet(DM_Motor_Instance* motor, float aTarget_, float vTarget_)
{
    aTarget_ = fabsf(aTarget_) > 190.0f * REDUCTION_RATIO_WHEEL * REDUCTION_TRACK
                   ? 190.0f * REDUCTION_RATIO_WHEEL * REDUCTION_TRACK
                   : aTarget_;
    aTarget_ = aTarget_ < 0.0f ? 0.0f : aTarget_;
    motor->Control_Setting.a_target_last = motor->Control_Setting.a_target;
    motor->Control_Setting.a_target = aTarget_;
    motor->Control_Setting.v_target = vTarget_;
}
