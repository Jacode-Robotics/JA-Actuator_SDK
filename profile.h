#ifndef PROFILE_H_
#define PROFILE_H_

#include <cstdint>
#include <cmath>

class PF_Handle {
public:
    uint8_t type;
    bool lock;
    bool move_direction;
    int32_t profile_pos;
    int32_t profile_vel;
    int32_t profile_acc;
    int16_t profile_time;
    int16_t profile_acc_time;
    float goal_acc;
    int32_t goal_vel;
    float f_trajectory_pos;
    float f_trajectory_vel;
    float f_trajectory_acc;
    int32_t trajectory_pos;
    int32_t trajectory_vel;
    int32_t trajectory_acc;    
    uint32_t t;
    uint32_t t1;
    uint32_t t2;
    uint32_t t3;

    PF_Handle();
    // PF_Handle(int32_t starting_position);

    void NewGoalPos(int32_t starting_position, int32_t goal_position);

    bool ExecutionPos();
};

PF_Handle::PF_Handle() {
    type = 0;
    lock = false;
    profile_pos = 0;
    f_trajectory_pos = 0;
    f_trajectory_vel = 0.0;
    f_trajectory_acc = 0.0;
    trajectory_pos = 0;
    trajectory_vel = 0;
    trajectory_acc = 0;
    t = 0;
    t1 = 0;
    t2 = 0;
    t3 = 0;
    goal_vel = 0;
    profile_vel      = 1386 * 0.3;
    profile_acc      = 2079 * 0.3;
    profile_time     = 2000;
    profile_acc_time = 700;
}

void PF_Handle::NewGoalPos(int32_t starting_position, int32_t goal_position) {
    int32_t delta_pos = (goal_position > starting_position) ? goal_position - starting_position : starting_position - goal_position;

    lock = true;

    move_direction = (goal_position > starting_position) ? true : false;
    profile_pos = goal_position;
    f_trajectory_pos = starting_position;

    t = 0;

    switch (type) {
        // Velocity-based Profile
    case 0:
        t1 = 600 * profile_vel / profile_acc;
        t2 = (6000000 / 32768) * (static_cast<float>(delta_pos) / static_cast<float>(profile_vel));
        if (t1 > t2) {
            t1 = sqrt(static_cast<float>(delta_pos) / static_cast<float>(profile_acc) / 32768) * 60000;
            t2 = t1;
        }
        t3 = t1 + t2;

        goal_acc = profile_acc;
        break;

    // Time-based Profile
    case 1:
        t1 = profile_acc_time;
        t3 = profile_time * 0.1;
        t2 = t3 - t1;

        goal_acc = (static_cast<float>(delta_pos) / t1) * (10986328.1250000 / t2);       
        break;

    default:
        break;
    }

    lock = false;
}

bool PF_Handle::ExecutionPos() {
    bool ret = false;

    if (lock) {
        return ret;
    }

    if (t >= t3) {
        f_trajectory_acc = 0;
        f_trajectory_vel = 0;
        f_trajectory_pos = profile_pos;

        ret = true;
    } else {
        if (t < t1) {
            f_trajectory_acc = move_direction ? goal_acc : -goal_acc;
        } else if (t < t2) {
            f_trajectory_acc = 0;
        } else {
            f_trajectory_acc = move_direction ? -goal_acc : goal_acc;
        }

        // f_trajectory_vel += f_trajectory_acc * (1 / 60000) * 100 * 10;
        // f_trajectory_pos += f_trajectory_vel * (1 / 60000) / 100 * 32768 * 10;

        f_trajectory_vel += f_trajectory_acc * (1.0 / 60000.0) * 100.0 * 10.0;
        f_trajectory_pos += f_trajectory_vel * (1.0 / 60000.0) / 100.0 * 32768.0 * 10.0;

        t += 10;
    }

    trajectory_pos = static_cast<int32_t>(f_trajectory_pos);
    trajectory_vel = static_cast<int32_t>(f_trajectory_vel);
    trajectory_acc = static_cast<int32_t>(f_trajectory_acc);

    return ret;
}

#endif /* PROFILE_H_ */
