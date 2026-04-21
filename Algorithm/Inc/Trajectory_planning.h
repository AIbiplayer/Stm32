//
// Created by Administrator on 26-4-19.
//

#ifndef TRAJECTORY_PLANNING_H
#define TRAJECTORY_PLANNING_H

#include "main.h"
#include "bsp_dwt.h"

#define duration 2 //轨迹规划的时间，单位为秒
#define step_size 0.001f //轨迹规划的步长，单位为秒

enum Trajectory_state {
    TRAJECTORY_READY = 0,
    TRAJECTORY_RUNNING
};

enum Trajectory_mode {
    NONE = 0, //正常姿态
    ZERO_POS, // 零点位置
    MID_POS // 中点位置
};

class Gimbal_joint {
private:
    float Max_position{0.0f}; // 最大位置
    float Min_position{0.0f}; // 最小位置

public:
    float set_target(float target) const {
        if (target > Max_position) target = Max_position;
        else if (target < Min_position) target = Min_position;
        return target;
    }

    Gimbal_joint(float max_pos, float min_pos) : Max_position(max_pos), Min_position(min_pos) {
    };
};

class Trajectory {
public:
    Trajectory_state state{TRAJECTORY_RUNNING};
    float target_pos[3]{0.0f}; // 目标位置

private:
    float target_vel[3]{0.0f}; // 目标速度
    float start_vel[3]{0.0f}; // 起始速度
    float k0{0.0f}, k1{0.0f}, k2{0.0f}, k3{0.0f}, k4{0.0f}, k5{0.0f}; // 五次多项式的系数缓存
    float A[3][6]{0.0f}; // 五次多项式的系数矩阵,每行对应一个轴，每列对应一个系数
    uint8_t flag[3]{0}; // 标志位，表示每个轴是否需要进行轨迹规划
    float step{0.0f}; // 当前轨迹规划的时间步长
    uint8_t step_flag{0}; // 步长标志，表示是否需要更新步长

    Trajectory_mode last_mode{}; // 上一次的轨迹模式
public:
    void Auto_motion(float *Start_pos, Trajectory_mode mode); // 自动运动函数，根据起始位置和模式计算目标位置和速度，并设置轨迹规划的标志位
    void reset(); // 重置函数，清除轨迹规划的状态和计数器

private:
    float update(float Start_pos, float Target_pos, uint8_t Joint_index, float Duration);

    void update_step(); // 更新步长函数，根据当前的时间步长和轨迹规划的状态更新步长
    bool update_motion_switch_next(const float Start_pos[3], const float Target_pos[3], const float Duration_[3]);

    bool update_motion_switch_end(const float Start_pos[3], const float Target_pos[3], const float Duration_[3]);

    // 更新函数，根据当前的时间步长和起始位置、目标位置计算当前的轨迹位置)
};

#endif //TRAJECTORY_PLANNING_H
