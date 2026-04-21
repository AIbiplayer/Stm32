#include "Trajectory_planning.h"
#include "main.h"
#include "string.h"
#include "Pos.h"

namespace {
    struct Pose_pair {
        float Start_pos[3];
        float Target_pos[3];
    };

    /**
     * 0号轴：yaw轴，1号轴：pitch_down轴，2号轴：pitch_up轴
     * 开始工作的角度
     */
    const Pose_pair work_pos = {
        {YAW_RESET_POS, PITCH_DOWN_RESET_POS, PITCH_UP_RESET_POS},
        {YAW_RESET_POS, PITCH_DOWN_WORK_POS, PITCH_UP_RESET_POS}
    };

    const Pose_pair reset_pos_final = {
        {YAW_RESET_POS, PITCH_DOWN_WORK_POS, PITCH_UP_RESET_POS},
        {YAW_RESET_POS, PITCH_DOWN_RESET_POS, PITCH_UP_RESET_POS}
    };

    /**
     * 根据轨迹模式获取对应的起始位置和目标位置
     * @param mode 轨迹模式
     * @return 对应的起始位置和目标位置的指针，如果模式无效则返回nullptr
     */
    const Pose_pair *get_pose_pair(Trajectory_mode mode) {
        switch (mode) {
            case ZERO_POS:
                return &reset_pos_final;
            case MID_POS:
                return &work_pos;
            default:
                return nullptr;
        }
    }

    /**
     * 清除轨迹规划的标志位，将所有轴的标志位设置为0
     * @param flag 轨迹规划的标志位数组，长度为3
     */
    void clear_plan_flag(uint8_t flag[3]) {
        memset(flag, 0, sizeof(uint8_t) * 3);
    }

    /**
     * 获取轨迹规划的最大持续时间，即三个轴中持续时间最长的那个
     * @param Duration 轨迹规划的持续时间数组，长度为3
     * @return 最大持续时间
     */
    float get_max_duration(const float Duration[3]) {
        float max_duration = Duration[0];
        for (int i = 1; i < 3; ++i) {
            if (Duration[i] > max_duration) {
                max_duration = Duration[i];
            }
        }
        return max_duration;
    }
}

/**
 * @brief 更新轨迹规划的时间步长，根据当前的时间步长和轨迹规划的状态更新步长
 */
void Trajectory::update_step() {
    if (step <= duration)
        step += step_size;
}

/**
 * @brief 更新轨迹规划的运动开关，判断是否需要切换到下一个轨迹模式
 * @param Start_pos 起始位置数组，长度为3
 * @param Target_pos 目标位置数组，长度为3
 * @param Duration 轨迹规划的持续时间数组，长度为3
 */
float Trajectory::update(float Start_pos, float Target_pos, uint8_t Joint_index, float Duration) {
    Joint_index -= 1;
    k0 = A[Joint_index][0];
    k1 = A[Joint_index][1];
    k2 = A[Joint_index][2];
    k3 = A[Joint_index][3];
    k4 = A[Joint_index][4];
    k5 = A[Joint_index][5];
    float Start_vel = start_vel[Joint_index];
    float Target_vel = target_vel[Joint_index];

    if (!flag[Joint_index]) {
        A[Joint_index][0] = Start_pos;
        A[Joint_index][1] = Start_vel; // 起点速度
        A[Joint_index][2] = Target_vel; // 终点速度
        A[Joint_index][3] = 20.0f * (Target_pos - Start_pos) / (2 * Duration * Duration * Duration);
        A[Joint_index][4] = (30.0f * (Start_pos - Target_pos) + (14 * Target_vel + 16 * Start_vel) * Duration) / (
                                2 * Duration * Duration * Duration * Duration);
        A[Joint_index][5] = (12.0f * (-Start_pos + Target_pos) - 6 * (Target_vel + Start_vel) * Duration) / (
                                2 * Duration * Duration * Duration * Duration * Duration);
        flag[Joint_index] = 1;
        return Start_pos;
    }
    if (step > Duration)
        return Target_pos;
    return k0 + k1 * step + k2 * step * step + k3 * step * step * step + k4 * step * step * step * step + k5 * step *
           step * step * step * step;
}

/**
 * @brief 重置轨迹规划的状态和计数器，将状态设置为准备状态，清除标志位，重置步长和步长标志
 */
void Trajectory::reset() {
    state = TRAJECTORY_READY;
    clear_plan_flag(flag);
    step = 0.0f;
    step_flag = 0;
}

void Trajectory::Auto_motion(float *Start_pos, Trajectory_mode mode) {
    if (mode == last_mode || step < duration) {
        switch (mode) {
            case ZERO_POS:
        }
    }
}


