#include "Trajectory_planning.h"

#include <math.h>
#include <string.h>

namespace {
    enum trajectory_step_e {
        TRAJECTORY_STEP_IDLE = 0,
        TRAJECTORY_STEP_RELEASE_PITCH_DOWN,
        TRAJECTORY_STEP_TIGHTEN_HEAD,
        TRAJECTORY_STEP_TIGHTEN_PITCH_DOWN
    };

    struct GimbalJointLimit {
        float max_position;
        float min_position;
    };

    struct GimbalTrajectoryPlanner {
        gimbal_trajectory_state_e state;
        float target_pos[GIMBAL_AXIS_COUNT];
        float realtime_target[GIMBAL_AXIS_COUNT];
        float tighten_target[GIMBAL_AXIS_COUNT];
        float start_vel[GIMBAL_AXIS_COUNT];
        float target_vel[GIMBAL_AXIS_COUNT];
        float start_acc[GIMBAL_AXIS_COUNT];
        float target_acc[GIMBAL_AXIS_COUNT];
        float coefficient[GIMBAL_AXIS_COUNT][6];
        uint8_t flag[GIMBAL_AXIS_COUNT];
        float step;
        trajectory_step_e step_flag;
        gimbal_mode_e last_mode;
        uint8_t first_release_pending;
        uint8_t initialized;
    };

    GimbalTrajectoryPlanner planner;

    const GimbalJointLimit pitch_up_limit = {
        PITCH_UP_MAX_ANGLE,
        PITCH_UP_MIN_ANGLE
    };

    uint8_t is_angle_close(float current, float target, float threshold) {
        return fabsf(current - target) <= threshold;
    }

    float get_nearest_target(float current_angle, float fixed_angle) {
        return fixed_angle + roundf((current_angle - fixed_angle) / GIMBAL_ANGLE_PERIOD) * GIMBAL_ANGLE_PERIOD;
    }

    uint8_t is_tighten_pose(const float start_pos[GIMBAL_AXIS_COUNT]) {
        return is_angle_close(start_pos[GIMBAL_PITCH_DOWN_INDEX], PITCH_TIGHTEN_ANGLE, 3.0f) &&
               is_angle_close(start_pos[GIMBAL_PITCH_UP_INDEX],
                              get_nearest_target(start_pos[GIMBAL_PITCH_UP_INDEX], PITCH_UP_TIGHTEN_ROUND_ANGLE),
                              3.0f);
    }

    void clear_plan_flag(uint8_t flag[GIMBAL_AXIS_COUNT]) {
        memset(flag, 0, sizeof(uint8_t) * GIMBAL_AXIS_COUNT);
    }

    void copy_pose(float target[GIMBAL_AXIS_COUNT], const float source[GIMBAL_AXIS_COUNT]) {
        uint8_t i;
        for (i = 0; i < GIMBAL_AXIS_COUNT; ++i) {
            target[i] = source[i];
        }
    }

    float get_max_duration(const float duration[GIMBAL_AXIS_COUNT]) {
        uint8_t i;
        float max_duration = duration[0];
        for (i = 1; i < GIMBAL_AXIS_COUNT; ++i) {
            if (duration[i] > max_duration) {
                max_duration = duration[i];
            }
        }
        return max_duration;
    }

    float limit_target(const GimbalJointLimit *joint, float target) {
        if (target > joint->max_position) {
            return joint->max_position;
        }
        if (target < joint->min_position) {
            return joint->min_position;
        }
        return target;
    }

    void reset_motion_only() {
        planner.state = GIMBAL_TRAJECTORY_READY;
        clear_plan_flag(planner.flag);
        planner.step = 0.0f;
        planner.step_flag = TRAJECTORY_STEP_IDLE;
        planner.last_mode = GIMBAL_NONE;
    }

    void init_planner() {
        memset(&planner, 0, sizeof(planner));
        planner.state = GIMBAL_TRAJECTORY_READY;
        planner.realtime_target[GIMBAL_YAW_INDEX] = YAW_TIGHTEN_ANGLE;
        planner.realtime_target[GIMBAL_PITCH_DOWN_INDEX] = PITCH_HOLD_ANGLE;
        planner.realtime_target[GIMBAL_PITCH_UP_INDEX] = PITCH_UP_TIGHTEN_ROUND_ANGLE;
        planner.tighten_target[GIMBAL_YAW_INDEX] = YAW_TIGHTEN_ANGLE;
        planner.tighten_target[GIMBAL_PITCH_DOWN_INDEX] = PITCH_TIGHTEN_ANGLE;
        planner.tighten_target[GIMBAL_PITCH_UP_INDEX] = PITCH_UP_TIGHTEN_ROUND_ANGLE;
        planner.first_release_pending = 1;
        planner.initialized = 1;
    }

    void ensure_initialized() {
        if (!planner.initialized) {
            init_planner();
        }
    }

    void update_tighten_target(const float start_pos[GIMBAL_AXIS_COUNT]) {
        planner.tighten_target[GIMBAL_YAW_INDEX] = get_nearest_target(start_pos[GIMBAL_YAW_INDEX], YAW_TIGHTEN_ANGLE);
        planner.tighten_target[GIMBAL_PITCH_DOWN_INDEX] = PITCH_TIGHTEN_ANGLE;
        planner.tighten_target[GIMBAL_PITCH_UP_INDEX] =
            get_nearest_target(start_pos[GIMBAL_PITCH_UP_INDEX], PITCH_UP_TIGHTEN_ROUND_ANGLE) +
            start_pos[GIMBAL_PITCH_DOWN_INDEX] - PITCH_TIGHTEN_ANGLE;
    }

    void update_direct_target() {
        planner.target_pos[GIMBAL_YAW_INDEX] = planner.realtime_target[GIMBAL_YAW_INDEX];
        planner.target_pos[GIMBAL_PITCH_DOWN_INDEX] = PITCH_HOLD_ANGLE;
        planner.target_pos[GIMBAL_PITCH_UP_INDEX] = limit_target(&pitch_up_limit,
                                                                 planner.realtime_target[GIMBAL_PITCH_UP_INDEX]);
        planner.state = GIMBAL_TRAJECTORY_READY;
    }

    void update_step() {
        if (planner.state == GIMBAL_TRAJECTORY_RUNNING) {
            planner.step += TRAJECTORY_STEP_SIZE;
        }
    }

    float update_axis(float start_pos, float target_pos, uint8_t joint_index, float duration) {
        float delta_pos;
        float duration_square;
        float duration_cube;
        float duration_fourth;
        float duration_fifth;
        float t;

        if (duration <= 0.0f) {
            return target_pos;
        }

        delta_pos = target_pos - start_pos;
        if (fabsf(delta_pos) < 1e-6f) {
            return target_pos;
        }

        if (!planner.flag[joint_index]) {
            duration_square = duration * duration;
            duration_cube = duration_square * duration;
            duration_fourth = duration_cube * duration;
            duration_fifth = duration_fourth * duration;

            planner.coefficient[joint_index][0] = start_pos;
            planner.coefficient[joint_index][1] = planner.start_vel[joint_index];
            planner.coefficient[joint_index][2] = 0.5f * planner.start_acc[joint_index];
            planner.coefficient[joint_index][3] =
                (20.0f * delta_pos
                 - (8.0f * planner.target_vel[joint_index] + 12.0f * planner.start_vel[joint_index]) * duration
                 - (3.0f * planner.start_acc[joint_index] - planner.target_acc[joint_index]) * duration_square)
                / (2.0f * duration_cube);
            planner.coefficient[joint_index][4] =
                (30.0f * (-delta_pos)
                 + (14.0f * planner.target_vel[joint_index] + 16.0f * planner.start_vel[joint_index]) * duration
                 + (3.0f * planner.start_acc[joint_index] - 2.0f * planner.target_acc[joint_index]) * duration_square)
                / (2.0f * duration_fourth);
            planner.coefficient[joint_index][5] =
                (12.0f * delta_pos
                 - (6.0f * planner.target_vel[joint_index] + 6.0f * planner.start_vel[joint_index]) * duration
                 - (planner.start_acc[joint_index] - planner.target_acc[joint_index]) * duration_square)
                / (2.0f * duration_fifth);
            planner.flag[joint_index] = 1;
        }

        if (planner.step >= duration) {
            return target_pos;
        }

        t = planner.step;
        return (((((planner.coefficient[joint_index][5] * t + planner.coefficient[joint_index][4]) * t
                    + planner.coefficient[joint_index][3]) * t
                  + planner.coefficient[joint_index][2]) * t
                 + planner.coefficient[joint_index][1]) * t
                + planner.coefficient[joint_index][0]);
    }

    uint8_t update_motion_switch_next(const float start_pos[GIMBAL_AXIS_COUNT],
                                      const float target_pos[GIMBAL_AXIS_COUNT],
                                      const float duration[GIMBAL_AXIS_COUNT],
                                      trajectory_step_e next_step) {
        uint8_t i;

        for (i = 0; i < GIMBAL_AXIS_COUNT; ++i) {
            planner.target_pos[i] = update_axis(start_pos[i], target_pos[i], i, duration[i]);
        }

        planner.state = GIMBAL_TRAJECTORY_RUNNING;
        if (planner.step >= get_max_duration(duration)) {
            copy_pose(planner.target_pos, target_pos);
            planner.step = 0.0f;
            planner.step_flag = next_step;
            clear_plan_flag(planner.flag);
            return 1;
        }
        return 0;
    }

    uint8_t update_motion_switch_end(const float start_pos[GIMBAL_AXIS_COUNT],
                                     const float target_pos[GIMBAL_AXIS_COUNT],
                                     const float duration[GIMBAL_AXIS_COUNT]) {
        uint8_t i;

        for (i = 0; i < GIMBAL_AXIS_COUNT; ++i) {
            planner.target_pos[i] = update_axis(start_pos[i], target_pos[i], i, duration[i]);
        }

        planner.state = GIMBAL_TRAJECTORY_RUNNING;
        if (planner.step >= get_max_duration(duration)) {
            copy_pose(planner.target_pos, target_pos);
            planner.step = 0.0f;
            planner.step_flag = TRAJECTORY_STEP_IDLE;
            clear_plan_flag(planner.flag);
            planner.state = GIMBAL_TRAJECTORY_READY;
            return 1;
        }
        return 0;
    }

    void planner_update(const float start_pos[GIMBAL_AXIS_COUNT], gimbal_mode_e mode) {
        uint8_t keep_release_step = 0;

        if ((mode == GIMBAL_GYRO_MODE || mode == GIMBAL_VISION) &&
            planner.state == GIMBAL_TRAJECTORY_RUNNING &&
            planner.step_flag == TRAJECTORY_STEP_RELEASE_PITCH_DOWN) {
            keep_release_step = 1;
        }

        if (mode != planner.last_mode && !keep_release_step) {
            clear_plan_flag(planner.flag);
            planner.step = 0.0f;

            switch (mode) {
                case GIMBAL_TIGHTEN:
                    update_tighten_target(start_pos);
                    planner.step_flag = TRAJECTORY_STEP_TIGHTEN_HEAD;
                    planner.state = GIMBAL_TRAJECTORY_RUNNING;
                    break;

                case GIMBAL_GYRO_MODE:
                case GIMBAL_VISION:
                    if (planner.last_mode == GIMBAL_TIGHTEN || is_tighten_pose(start_pos) ||
                        planner.first_release_pending) {
                        update_tighten_target(start_pos);
                        planner.step_flag = TRAJECTORY_STEP_RELEASE_PITCH_DOWN;
                        planner.state = GIMBAL_TRAJECTORY_RUNNING;
                        planner.first_release_pending = 0;
                    } else {
                        planner.step_flag = TRAJECTORY_STEP_IDLE;
                        planner.state = GIMBAL_TRAJECTORY_READY;
                    }
                    break;

                case GIMBAL_NONE:
                default:
                    planner.step_flag = TRAJECTORY_STEP_IDLE;
                    planner.state = GIMBAL_TRAJECTORY_READY;
                    break;
            }
        }

        switch (mode) {
            case GIMBAL_TIGHTEN:
                if (planner.step_flag == TRAJECTORY_STEP_TIGHTEN_HEAD) {
                    const float head_target[GIMBAL_AXIS_COUNT] = {
                        planner.tighten_target[GIMBAL_YAW_INDEX],
                        start_pos[GIMBAL_PITCH_DOWN_INDEX],
                        planner.tighten_target[GIMBAL_PITCH_UP_INDEX]
                    };
                    const float head_duration[GIMBAL_AXIS_COUNT] = {
                        YAW_TIGHTEN_DURATION,
                        0.0f,
                        PITCH_UP_TIGHTEN_DURATION
                    };
                    update_motion_switch_next(start_pos, head_target, head_duration, TRAJECTORY_STEP_TIGHTEN_PITCH_DOWN);
                } else if (planner.step_flag == TRAJECTORY_STEP_TIGHTEN_PITCH_DOWN) {
                    const float tighten_duration[GIMBAL_AXIS_COUNT] = {
                        0.0f,
                        PITCH_DOWN_TIGHTEN_DURATION,
                        0.0f
                    };
                    update_motion_switch_end(start_pos, planner.tighten_target, tighten_duration);
                } else {
                    copy_pose(planner.target_pos, planner.tighten_target);
                    planner.state = GIMBAL_TRAJECTORY_READY;
                }
                break;

            case GIMBAL_GYRO_MODE:
            case GIMBAL_VISION:
                if (planner.step_flag == TRAJECTORY_STEP_RELEASE_PITCH_DOWN) {
                    const float release_target[GIMBAL_AXIS_COUNT] = {
                        planner.tighten_target[GIMBAL_YAW_INDEX],
                        PITCH_HOLD_ANGLE,
                        planner.tighten_target[GIMBAL_PITCH_UP_INDEX]
                    };
                    const float release_duration[GIMBAL_AXIS_COUNT] = {
                        0.0f,
                        PITCH_DOWN_RELEASE_DURATION,
                        0.0f
                    };
                    update_motion_switch_end(start_pos, release_target, release_duration);
                } else {
                    update_direct_target();
                }
                break;

            case GIMBAL_NONE:
            default:
                copy_pose(planner.target_pos, start_pos);
                planner.state = GIMBAL_TRAJECTORY_READY;
                break;
        }

        planner.last_mode = mode;
        update_step();
    }
}

extern "C" void GimbalTrajectory_Init(void) {
    init_planner();
}

extern "C" void GimbalTrajectory_Reset(void) {
    ensure_initialized();
    reset_motion_only();
}

extern "C" void GimbalTrajectory_SetRealtimeTarget(float yaw_target, float pitch_up_target) {
    ensure_initialized();
    planner.realtime_target[GIMBAL_YAW_INDEX] = yaw_target;
    planner.realtime_target[GIMBAL_PITCH_DOWN_INDEX] = PITCH_HOLD_ANGLE;
    planner.realtime_target[GIMBAL_PITCH_UP_INDEX] = pitch_up_target;
}

extern "C" void GimbalTrajectory_Update(const float start_pos[GIMBAL_AXIS_COUNT], gimbal_mode_e mode) {
    ensure_initialized();
    planner_update(start_pos, mode);
}

extern "C" void GimbalTrajectory_GetTarget(float target_pos[GIMBAL_AXIS_COUNT]) {
    ensure_initialized();
    copy_pose(target_pos, planner.target_pos);
}

extern "C" gimbal_trajectory_state_e GimbalTrajectory_GetState(void) {
    ensure_initialized();
    return planner.state;
}

extern "C" uint8_t GimbalTrajectory_IsRunning(void) {
    return GimbalTrajectory_GetState() == GIMBAL_TRAJECTORY_RUNNING;
}
