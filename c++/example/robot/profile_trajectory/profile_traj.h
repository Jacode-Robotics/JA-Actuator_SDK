/*
 * @Author: NO3 
 * @Date: 2024-08-21 12:42:42 
 * @Copyright (c): Jacode Robotics (Hong Kong) Limited.
 */
#ifndef PROFILE_TRAJ_H_
#define PROFILE_TRAJ_H_

#include <cstdint>
#include <cmath>

/* Trajectory profile period, in milliseconds */
#define CONTROL_PERIOD 10 
#define POSITION_RESOLUTION 32768.0

/**
 * Time in milliseconds
 * Position in count
 * Velocity in 0.01 rev/min
 * Acceleration in rev/min^2
 */
class ProfileTraj {
private:
    uint8_t type;
    bool move_direction;
    int32_t profile_pos;
    int32_t profile_vel;
    int32_t profile_acc;
    int16_t profile_time;
    int16_t profile_acc_time;
    float goal_acc;
    float waypoint_pos;
    float waypoint_vel;
    float waypoint_acc;
    uint32_t t;
    uint32_t t1;
    uint32_t t2;
    uint32_t t3;

public:
    ProfileTraj(uint8_t type = 0 /* 0: Velocity-based, 1: Time-based */, 
                int32_t max_vel = 1386 * 0.3, 
                int32_t max_acc = 2079 * 0.3, 
                int16_t motion_time = 3000, 
                int16_t acc_time = 1000) 
        : type(type), 
          profile_vel(max_vel), 
          profile_acc(max_acc), 
          profile_time(motion_time), 
          profile_acc_time(acc_time) {}

    void SetGoalPos(int32_t starting_position, int32_t goal_position) {
        float delta_pos = abs(goal_position - starting_position);
        move_direction = (goal_position > starting_position);

        t = 0;
        profile_pos = goal_position;
        waypoint_pos = starting_position;

        switch (type) {
        // Velocity-based Profile
        case 0:
            t1 = 600.0 * profile_vel / profile_acc;
            t2 = (6000000.0 / POSITION_RESOLUTION) * (delta_pos / profile_vel);
            if (t1 > t2) {
                t1 = sqrt(delta_pos / profile_acc / POSITION_RESOLUTION) * 60000.0;
                t2 = t1;
            }
            t3 = t1 + t2;

            goal_acc = profile_acc;
            break;

        // Time-based Profile
        case 1:
            t1 = profile_acc_time;
            t3 = profile_time;
            t2 = t3 - t1;

            goal_acc = (delta_pos / t1 * t2) * (60000.0 * 60000.0 / POSITION_RESOLUTION);       
            break;

        default:
            break;
        }
    }

    void SetGoalPos(int32_t starting_position, int32_t goal_position, uint8_t type, int32_t param_vel, int32_t param_acc) {
        if (1 == type) {
            profile_time = param_vel;
            profile_acc_time = param_acc;
        } else {
            type = 0;
            profile_vel = param_vel;
            profile_acc = param_acc;
        }
        
        SetGoalPos(starting_position, goal_position);
    }


    bool ExecutionPos(int32_t& pos, int32_t& vel, int32_t& acc) {
        bool ret = false;

        if (t >= t3) {
            waypoint_acc = 0;
            waypoint_vel = 0;
            waypoint_pos = profile_pos;

            ret = true;
        } else {
            if (t < t1) {
                waypoint_acc = move_direction ? goal_acc : -goal_acc;
            } else if (t < t2) {
                waypoint_acc = 0;
            } else {
                waypoint_acc = move_direction ? -goal_acc : goal_acc;
            }

            waypoint_vel += waypoint_acc / 60000.0 * 100.0 * CONTROL_PERIOD;
            waypoint_pos += waypoint_vel / 60000.0 / 100.0 * POSITION_RESOLUTION * CONTROL_PERIOD;

            t += CONTROL_PERIOD;
        }

        pos = static_cast<int32_t>(waypoint_pos);
        vel = static_cast<int32_t>(waypoint_vel);
        acc = static_cast<int32_t>(waypoint_acc);

        return ret;
    }
};

#endif /* PROFILE_TRAJ_H_ */