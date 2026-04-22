//
// Created by Administrator on 26-4-19.
//

#ifndef TRAJECTORY_PLANNING_H
#define TRAJECTORY_PLANNING_H

#include "main.h"
#include "Pos.h"

typedef enum {
    GIMBAL_TRAJECTORY_READY = 0,
    GIMBAL_TRAJECTORY_RUNNING
} gimbal_trajectory_state_e;

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 轨迹规划只负责两类过渡：
 * 1. 任意非缩紧模式 -> 缩紧模式
 * 2. 缩紧模式 -> GYRO / VISION
 *
 * GYRO/VISION 模式下的 yaw 与 pitch_up 目标为实时直通，
 * pitch_down 固定保持在 PITCH_HOLD_ANGLE。
 */
void GimbalTrajectory_Init(void);
void GimbalTrajectory_Reset(void);
void GimbalTrajectory_SetRealtimeTarget(float yaw_target, float pitch_up_target);
void GimbalTrajectory_Update(const float start_pos[GIMBAL_AXIS_COUNT], gimbal_mode_e mode);
void GimbalTrajectory_GetTarget(float target_pos[GIMBAL_AXIS_COUNT]);
gimbal_trajectory_state_e GimbalTrajectory_GetState(void);
uint8_t GimbalTrajectory_IsRunning(void);

#ifdef __cplusplus
}
#endif

#endif //TRAJECTORY_PLANNING_H
