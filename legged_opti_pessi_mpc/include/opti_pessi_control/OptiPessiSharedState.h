#pragma once

#include <mutex>

namespace ocs2 {
namespace quadruped {

struct ObstacleState {
    std::mutex mutex;
    double x = 2.0;
    double y = 0.0;
    double max_vel = 0.5;
    double init_time = 0.0;  // MPC horizon start (for relative time in constraints)

    ObstacleState() = default;
    ObstacleState(const ObstacleState& other) {
        x = other.x;
        y = other.y;
        max_vel = other.max_vel;
        init_time = other.init_time;
    }
};

}  // namespace quadruped
}  // namespace ocs2
