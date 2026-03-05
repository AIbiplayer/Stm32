/* 当前只能控制3508电机*/

#include "dji_motor.h"
#include "general_def.h"
#include "arm_math.h"
#include "user_lib.h"
#include "power_limit.h"

//以下三种功率控制宏只能定义一种
#define POWER_ATTENUATION        //衰减功率
// #define CURRENT_ATTENUATION   //衰减电流
// #define ERROR_ALLOCATION         //等比缩放加P分配,减小功率控制对底盘运动状态的影响，未完成，效果不太好

#define K0 0.6641993430428782
#define K1 0.006444295981325497
#define K2 0.0001423857166749977
#define K3 0.01764443017662443
#define K4 0.16501438467529175
#define K5 3.0967217636825096e-05
#define TORQUE_CONSTANT 0.3f //转矩常数，单位N*m/A

#define ECD_2_I (1/819.2f)//编码器数值转电流值（A）
#define I_2_ECD 819.2f//电流值转编码器值

// static DJIMotorInstance *motor;

float I0, I1;

static float initial_I[16] = {0}; //电机的原始电流数据
static float I_Limited[16] = {0}; //限制后的电流数据
static float speed_rads[16] = {0}; //角速度，弧度制

static float MaxPower = 45; //最大功率

// static float I_feedback[16];

#ifdef POWER_ATTENUATION
static float initial_power[16] = {0}; //经过计算后得到的未限制的功率
float power_positive_sum = 0; //正数功率之和
float power_negative_sum = 0;
static float att_factor = 0; //衰减因数
#endif

#ifdef ERROR_ALLOCATION
extern ChassisData chassis_data;       //底盘整体数据
static float initial_power[16] = {0}; //经过计算后得到的未限制的功率
static float K[16] = {0}; //功率分配系数
static float error[16] = {0};//误差值
static float error_sum = 0;//误差值和
static float E_upper = 100;//误差上阈值
static float E_lower = 20;//误差下阈值
static float K_coe;//置信度
static float Power_sum;//功率之和
static float Power_Limited[16];//限制后的功率
#endif

static DJI_Motor_Instance *motor_instance[8]; //需要控制的电机注册到这里
static uint8_t idx = 0; //索引

void PLMotor_Register(DJI_Motor_Instance *motor) //将需要控制功率的大疆电机注册到这里
{
    motor_instance[idx++] = motor;
}

#ifdef POWER_ATTENUATION

static float att_factor_calculate(void) //计算功率衰减因数
{
    power_positive_sum = 0; //正数功率之和
    power_negative_sum = 0; //负数功率之和
    for (int i = 0; i < 4; i++) {
        if (initial_power[i] > 0) {
            power_positive_sum += initial_power[i];
        } else {
            power_negative_sum += initial_power[i]; //发电电机的总负功率，即发电功率
        }
    }
    if (power_positive_sum <= MaxPower - power_negative_sum)
        return 1.0f;
    else
        return (MaxPower - power_negative_sum) / power_positive_sum; //返回衰减因数
}

#endif
//以下为公有函数

void SetPowerMax(float _power_max) {
    MaxPower = _power_max;
}

float power_calculate(float I, float speed_rads) {
    I = I / 819.2f; //编码器电流值转化为真实电流值，单位A
    speed_rads = speed_rads * PI / 180;
    return (float) (K0 + K1 * I + K2 * speed_rads + K3 * I * speed_rads +
                    K4 * I * I + K5 * speed_rads * speed_rads);
}

#ifdef POWER_ATTENUATION
// 衰减功率算法
/**
 * @brief 限制功率（未完全完成）
 *
 */
void power_limit() {
    I0 = motor_instance[0]->Control_Setting.Power_Output;

    for (uint8_t i = 0; i < idx; i++) {
        initial_I[i] = motor_instance[i]->Measure.Current / 819.2f;
        speed_rads[i] = (float) (motor_instance[i]->Measure.Speed) * PI / 180.0f; //将度/秒转化为弧度/秒
    }

    for (uint8_t i = 0; i < idx; i++) {
        initial_power[i] = motor_instance[i]->Control_Setting.Power_Estimate;
    }
    att_factor = att_factor_calculate();
    if (att_factor >= 1.0f)
        return;
    float a = 0, b = 0, c = 0;
    for (uint8_t i = 0; i < idx; i++) {
        if (initial_power[i] <= 0) {
            I_Limited[i] = initial_I[i];
            continue;
        }

        a = K4;
        b = K1 + K3 * speed_rads[i];
        c = K0 + K2 * speed_rads[i] + K5 * speed_rads[i] * speed_rads[i] - att_factor * initial_power[i];
        float delta_ = b * b - 4 * a * c;
        float delta_sqrt;
        if (delta_ < 0.0f) {
            I_Limited[i] = 0; //无需功率控制
            continue;
        }
        arm_sqrt_f32(delta_, &delta_sqrt);
        float result_1, result_2;
        result_1 = (-b + delta_sqrt) / (2.0f * a);
        result_2 = (-b - delta_sqrt) / (2.0f * a);

        // 两个潜在的可行电流值, 取绝对值最小的那个
        if ((result_1 > 0.0f && result_2 < 0.0f) || (result_1 < 0.0f && result_2 > 0.0f)) {
            if ((initial_I[i] > 0.0f && result_1 > 0.0f) || (initial_I[i] < 0.0f && result_1 < 0.0f)) {
                I_Limited[i] = result_1;
            } else {
                I_Limited[i] = result_2;
            }
        } else {
            if (fabsf(result_1) < fabsf(result_2)) {
                I_Limited[i] = result_1;
            } else {
                I_Limited[i] = result_2;
            }
        }
    }

    for (uint8_t i = 0; i < idx; i++) {
        motor_instance[i]->Control_Setting.Power_Output = I_Limited[i] * 819.2f; //将电流值A转化为编码器的值
    }
    I1 = motor_instance[0]->Control_Setting.Power_Output;
}
#endif

#ifdef CURRENT_ATTENUATION
static float att_factor = 0; //衰减因数
//衰减电流算法
/**
 * @brief 限制功率（未完全完成，还需要从裁判系统读取当前等级）
 */
void power_limit()
{
    for (uint8_t i = 0; i < idx; i++) {
        initial_I[i] = motor_instance[i]->set_current / 819.2f;
    }
    speed_rads[0] = motor_instance[0]->measure.speed_aps * PI / 180;//将度/秒转化为弧度/秒
    speed_rads[1] = motor_instance[1]->measure.speed_aps *  PI / 180;
    speed_rads[2] = motor_instance[2]->measure.speed_aps *  PI / 180;
    speed_rads[3] = motor_instance[3]->measure.speed_aps *  PI / 180;

    float a = 0, b = 0, c = 0;

    // 替换原有的a、b、c计算逻辑
    for (uint8_t i = 0; i < idx; i++) {
        // 单个电机的二次项系数（α²）
        a += K4 * initial_I[i] * initial_I[i];
        // 单个电机的一次项系数（α）
        b += (K1 * initial_I[i]) + (K3 * initial_I[i] * speed_rads[i]);
        // 单个电机的常数项（不含α）
        c += K0 + (K2 * speed_rads[i]) + (K5 * speed_rads[i] * speed_rads[i]);
    }
    // 总功率等于MaxPower，整理后的常数项
    c = c - MaxPower;
        // switch (solve_quadratic(a, b, c, &root[0], &root[1])) {
        //     case 0:
        //         att_factor = 0;
        //         break;
        //     case 1:
        //         att_factor = root[0];
        //         break;
        //     case 2:
        //         if ((root[0] >= 0 && root[0] <= 1) && (root[1] >= 0 && root[1] <= 1)) {
        //             att_factor = root[0] > root[1] ? root[0] : root[1];
        //         }
        //         else if ((root[0] >= 0 && root[0] <= 1) && (root[1] < 0 || root[1] > 1)) {
        //             att_factor = root[0];
        //         }
        //         else if ((root[1] >= 0 && root[1] <= 1) && (root[0] < 0 || root[0] > 1)) {
        //             att_factor = root[1];
        //         }
        //         else
        //             att_factor = 0;
        //         break;
        //     default:
        //         att_factor = 0;
        //         break;
        // }
    float delta_ = b * b - 4 * a * c;
    float delta_sqrt;
    if (delta_ < 0.0f) {
        // I_Limited[0] = 0;//无需功率控制
        memset(I_Limited, 0, sizeof(I_Limited));
        return;
    }
    arm_sqrt_f32(delta_, &delta_sqrt);
    float result_1, result_2;
    result_1 = (-b + delta_sqrt) / (2.0f * a);
    result_2 = (-b - delta_sqrt) / (2.0f * a);

    if ((result_1 >= 0 && result_1 <= 1) && (result_2 >= 0 && result_2 <= 1)) {
            att_factor = result_1 > result_2 ? result_1 : result_2;
        }
    else if ((result_1 >= 0 && result_1 <= 1) && (result_2 < 0 || result_2 > 1)) {
        att_factor = result_1;
    }
    else if ((result_2 >= 0 && result_2 <= 1) && (result_1 < 0 || result_1 > 1)) {
        att_factor = result_2;
    }
    else
        att_factor = 0;

    if (att_factor > 1 || att_factor <= 0)//衰减系数不合理直接退出
        return;

    for (uint8_t i = 0; i < idx; i++) {
        I_Limited[i] = initial_I[i] * att_factor;
    }

    for (uint8_t i = 0; i < idx; i++) {
        motor_instance[i]->set_current = I_Limited[i] * 819.2f;
    }

    // //调试用
    // for (uint8_t i = 0; i < 4; i++) {
    //     power_sum_origin += power_calculate(I[i], speed_rads[i]);
    //     power_sum_limited += power_calculate(I_Limited[i], speed_rads[i]);
    // }
    // data.fdata[0] = power_sum_origin;
    // data.fdata[1] = power_sum_limited;
    // data.fdata[2] = 60;
    // HAL_UART_Transmit(&huart6, (uint8_t *)&data, sizeof(data), 100);
}
#endif

#ifdef ERROR_ALLOCATION
 void power_limit()
 {
    Power_sum = 0;
    for (uint8_t i = 0; i < idx; i++) {
        initial_I[i] = motor_instance[i]->set_current / 819.2f;
        speed_rads[i] = motor_instance[i]->measure.speed_aps * PI / 180;//将度/秒转化为弧度/秒
        initial_power[i] = motor_instance[i]->measure.Power_Estimate;
        // I_feedback[i] = motor_instance[i]->measure.real_current / 819.2f;
        if (initial_power[i] > 0)
            Power_sum += initial_power[i];
    }
    error_sum = 0;
    for (uint8_t i = 0; i < 4; i++) {
        error[i] = chassis_data.Target_Wheel_Omega[i] - speed_rads[i];
        error_sum += error[i];
    }

    if (error_sum > E_upper) {
        K_coe = 1;
    }
    else if (error_sum < E_lower) {
        K_coe = 0;
    }
    else {
        K_coe = (error_sum - E_lower) / (E_upper - E_lower);
    }

    for (uint8_t i = 0; i < idx; i++) {
        K[i] = K_coe * error[i] / error_sum + (1 - K_coe) *(initial_power[i] / Power_sum);
        Power_Limited[i] = K[i] * MaxPower;
    }

    float a = 0, b = 0, c = 0;
     for (uint8_t i = 0; i < idx; i++) {
         if (initial_power[i] <= 0) {
             I_Limited[i] = initial_I[i];
             continue;
         }

         a = K4;
         b = K1 + K3 * speed_rads[i];
         c = K0 + K2 * speed_rads[i] + K5 * speed_rads[i] * speed_rads[i] - Power_Limited[i];
         float delta_ = b * b - 4 * a * c;
         float delta_sqrt;
         if (delta_ < 0.0f) {
             I_Limited[i] = 0;//无需功率控制
             continue;
         }
         arm_sqrt_f32(delta_, &delta_sqrt);
         float result_1, result_2;
         result_1 = (-b + delta_sqrt) / (2.0f * a);
         result_2 = (-b - delta_sqrt) / (2.0f * a);

         // 两个潜在的可行电流值, 取绝对值最小的那个
         if ((result_1 > 0.0f && result_2 < 0.0f) || (result_1 < 0.0f && result_2 > 0.0f))
         {
             if ((initial_I[i] > 0.0f && result_1 > 0.0f) || (initial_I[i] < 0.0f && result_1 < 0.0f))
             {
                 I_Limited[i] = result_1;
             }
             else
             {
                 I_Limited[i] = result_2;
             }
         }
         else
         {
             if (abs(result_1) < abs(result_2))
             {
                 I_Limited[i] = result_1;
             }
             else
             {
                 I_Limited[i] = result_2;
             }
         }
         // motor_instance[i]->set_current = I_Limited[i] * 819.2f;//将电流值A转化为编码器的值
     }

     for (uint8_t i = 0; i < idx; i++) {
         motor_instance[i]->set_current = I_Limited[i] * 819.2f;//将电流值A转化为编码器的值
     }


 }
#endif
