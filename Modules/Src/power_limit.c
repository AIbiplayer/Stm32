#include "power_limit.h"

#include "referee.h"
#include "robot_def.h"

#define POWER_LIMIT_MAX_MOTOR_CNT 8u
#define POWER_LIMIT_DEFAULT_MAX_W 60.0f
#define POWER_LIMIT_MIN_ACTIVE_W 5.0f
#define POWER_LIMIT_RELEASE_ALPHA 0.08f
#define POWER_LIMIT_MEASURE_MARGIN_W 1.5f
#define POWER_LIMIT_CURRENT_TO_AMP (1.0f / 819.2f)
#define POWER_LIMIT_RPM_TO_RADPS (PI / 30.0f)

#define POWER_LIMIT_LOW_BUFFER_J 10u
#define POWER_LIMIT_FULL_BUFFER_J 40u
#define POWER_LIMIT_LOW_BUFFER_SCALE 0.75f

#define K0 0.6641993430428782f
#define K1 0.006444295981325497f
#define K2 0.0001423857166749977f
#define K3 0.01764443017662443f
#define K4 0.16501438467529175f
#define K5 3.0967217636825096e-05f

static DJI_Motor_Instance *power_limit_motor_list[POWER_LIMIT_MAX_MOTOR_CNT];
static uint8_t power_limit_motor_cnt = 0u;
static float power_limit_override_w = POWER_LIMIT_DEFAULT_MAX_W;
static float power_limit_last_scale = 1.0f;

static float Clamp01(float value) {
    if (value < 0.0f)
        return 0.0f;
    if (value > 1.0f)
        return 1.0f;
    return value;
}

static float MinFloat(float lhs, float rhs) {
    return lhs < rhs ? lhs : rhs;
}

static float GetMeasuredChassisPower(void) {
    if (Referee_data == NULL || !Referee_data->referee_online_state)
        return -1.0f;
    return Referee_data->PowerHeatData.chassis_power;
}

static float GetActivePowerLimit(void) {
    float limit_w = power_limit_override_w;

    if (Referee_data == NULL || !Referee_data->referee_online_state)
        return limit_w;

    if (Referee_data->GameRobotState.chassis_power_limit > 0u)
        limit_w = (float) Referee_data->GameRobotState.chassis_power_limit;

    if (Referee_data->PowerHeatData.buffer_energy < POWER_LIMIT_FULL_BUFFER_J) {
        float buffer_scale = POWER_LIMIT_LOW_BUFFER_SCALE;

        if (Referee_data->PowerHeatData.buffer_energy > POWER_LIMIT_LOW_BUFFER_J) {
            const float buffer_ratio = (float) (Referee_data->PowerHeatData.buffer_energy - POWER_LIMIT_LOW_BUFFER_J)
                                       / (float) (POWER_LIMIT_FULL_BUFFER_J - POWER_LIMIT_LOW_BUFFER_J);
            buffer_scale += (1.0f - POWER_LIMIT_LOW_BUFFER_SCALE) * buffer_ratio;
        }

        limit_w *= buffer_scale;
    }

    if (limit_w < POWER_LIMIT_MIN_ACTIVE_W)
        limit_w = POWER_LIMIT_MIN_ACTIVE_W;

    return limit_w;
}

void PLMotor_Register(DJI_Motor_Instance *motor) {
    if (motor == NULL)
        return;

    for (uint8_t i = 0; i < power_limit_motor_cnt; i++) {
        if (power_limit_motor_list[i] == motor)
            return;
    }

    if (power_limit_motor_cnt >= POWER_LIMIT_MAX_MOTOR_CNT)
        return;

    power_limit_motor_list[power_limit_motor_cnt++] = motor;
}

void SetPowerMax(float power_max) {
    if (power_max > 0.0f)
        power_limit_override_w = power_max;
}

float power_calculate(float output_current, float speed_rpm) {
    const float current_a = output_current * POWER_LIMIT_CURRENT_TO_AMP;
    const float speed_radps = speed_rpm * POWER_LIMIT_RPM_TO_RADPS;

    return K0 + K1 * current_a + K2 * speed_radps + K3 * current_a * speed_radps
           + K4 * current_a * current_a + K5 * speed_radps * speed_radps;
}

void power_limit(void) {
    float positive_power_sum = 0.0f;
    float estimated_power[POWER_LIMIT_MAX_MOTOR_CNT] = {0.0f};
    float original_output[POWER_LIMIT_MAX_MOTOR_CNT] = {0.0f};

    if (power_limit_motor_cnt == 0u)
        return;

    const float active_limit_w = GetActivePowerLimit();
    if (active_limit_w <= POWER_LIMIT_MIN_ACTIVE_W) {
        power_limit_last_scale = 0.0f;
        for (uint8_t i = 0; i < power_limit_motor_cnt; i++)
            power_limit_motor_list[i]->Control_Setting.Power_Output = 0;
        return;
    }

    for (uint8_t i = 0; i < power_limit_motor_cnt; i++) {
        original_output[i] = (float) power_limit_motor_list[i]->Control_Setting.Power_Output;
        if (original_output[i] == 0.0f)
            continue;

        estimated_power[i] = power_calculate(original_output[i], (float) power_limit_motor_list[i]->Measure.Speed);
        if (estimated_power[i] > 0.0f)
            positive_power_sum += estimated_power[i];
    }

    float target_scale = 1.0f;
    if (positive_power_sum > active_limit_w)
        target_scale = active_limit_w / positive_power_sum;

    const float measured_power_w = GetMeasuredChassisPower();
    if (measured_power_w > active_limit_w + POWER_LIMIT_MEASURE_MARGIN_W)
        target_scale = MinFloat(target_scale, active_limit_w / measured_power_w);

    target_scale = Clamp01(target_scale);

    if (target_scale < power_limit_last_scale) {
        power_limit_last_scale = target_scale;
    } else {
        power_limit_last_scale += (target_scale - power_limit_last_scale) * POWER_LIMIT_RELEASE_ALPHA;
        if (power_limit_last_scale > 0.999f)
            power_limit_last_scale = 1.0f;
    }

    for (uint8_t i = 0; i < power_limit_motor_cnt; i++) {
        if (estimated_power[i] <= 0.0f)
            continue;

        power_limit_motor_list[i]->Control_Setting.Power_Output =
            (int16_t) (original_output[i] * power_limit_last_scale);
    }
}
