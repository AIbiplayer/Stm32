//
// Created by Administrator on 26-4-19.
//

#ifndef POS_H
#define POS_H

#include "robot_def.h"

enum Gimbal_axis_index {
    GIMBAL_YAW_INDEX = 0,
    GIMBAL_PITCH_DOWN_INDEX,
    GIMBAL_PITCH_UP_INDEX,
    GIMBAL_AXIS_COUNT
};

#define GIMBAL_ANGLE_PERIOD 360.0f
#define TRAJECTORY_STEP_SIZE 0.001f

/*
 * 下面这些姿态常量默认复用现有 robot_def.h 中的宏。
 * 如果后续机械限位或装配角度变化，优先只改这里。
 */
#ifndef YAW_TIGHTEN_ANGLE
#define YAW_TIGHTEN_ANGLE YAW_RESET_ANGLE
#endif

#ifndef PITCH_HOLD_ANGLE
#define PITCH_HOLD_ANGLE PITCH_HOLD_EXTEND_ANGLE
#endif

#ifndef PITCH_TIGHTEN_ANGLE
#define PITCH_TIGHTEN_ANGLE PITCH_HOLD_RESET_ANGLE
#endif

#ifndef PITCH_UP_TIGHTEN_ROUND_ANGLE
#define PITCH_UP_TIGHTEN_ROUND_ANGLE 5.0f
#endif

#ifndef PITCH_UP_MAX_ANGLE
#define PITCH_UP_MAX_ANGLE PITCH_MAX_ANGLE
#endif

#ifndef PITCH_UP_MIN_ANGLE
#define PITCH_UP_MIN_ANGLE PITCH_MIN_ANGLE
#endif

/*
 * 轨迹时间单位均为秒，调用周期默认为 1ms。
 * Yaw/Pitch_Up 先走到缩紧姿态，再允许 Pitch_Down 下压到缩紧角。
 */
#define YAW_TIGHTEN_DURATION 1.6f
#define PITCH_UP_TIGHTEN_DURATION 0.6f
#define PITCH_DOWN_TIGHTEN_DURATION 1.0f
#define PITCH_DOWN_RELEASE_DURATION 1.0f

#endif //POS_H
