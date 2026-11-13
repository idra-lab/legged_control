#pragma once
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>
#include <array>
#include <random>


#include "acados_legged_controller/LipmConfig.hpp"




namespace legged {

// What OCP needs to know about obstacles
struct ObstacleState {
    double x, y;        // m
    double radius;      // m
    double y_dot_max;   // m/s
};


// Base class: each subclass defines a behaviour in update()
class Obstacle {

public:
    explicit Obstacle(const ObstacleState& state) : state_(state) {}
    virtual ~Obstacle() = default;
    
    virtual void update(double dt, double t, double robot_x, double robot_y) = 0;
    // true: it follows the robot, so it stands still while the robot is not walking (updateAll)
    virtual bool chasesRobot() const { return false; }
    const ObstacleState& state() const { return state_; }

protected:
    ObstacleState state_;
};



// STATIC
class StaticObstacle : public Obstacle {
public:
    using Obstacle::Obstacle;
    void update(double, double, double, double) override {}
};



// DYNAMIC BACK AND FORTH
class DynamicObstacle : public Obstacle {
public:
    DynamicObstacle(const ObstacleState& state, double end_x, double end_y, double speed) 
        : Obstacle(state), start_x_(state.x), start_y_(state.y), end_x_(end_x), end_y_(end_y),
        speed_(speed), length_(std::hypot(end_x - state.x, end_y - state.y)) {}

    void update(double, double t, double, double) override {
        if (length_ < 1e-5) return;
        const double s = std::fmod(speed_ * t, 2.0 *length_); // position along the whole cycle
        const double travelled = (s <= length_) ? s : 2.0 * length_ - s; // second half: way back
        const double ratio = travelled / length_;
        state_.x = start_x_ + ratio * (end_x_ - start_x_);
        state_.y = start_y_ + ratio * (end_y_ - start_y_);
    }

private: 
    double start_x_, start_y_, end_x_, end_y_;
    double speed_;
    double length_;
};




// CIRCULAR: around a centre, starting from pos, constant speed along the circle (radius = |pos - centre|).
// Position from the time of the motion like DYNAMIC: same path at every test, restarts from pos after a reset
class CircularObstacle : public Obstacle {
public:
    CircularObstacle(const ObstacleState& state, double center_x, double center_y, double speed, bool clockwise)
        : Obstacle(state), center_x_(center_x), center_y_(center_y),
          radius_(std::hypot(state.x - center_x, state.y - center_y)),
          angle0_(std::atan2(state.y - center_y, state.x - center_x)),
          omega_((clockwise ? -1.0 : 1.0) * (radius_ > 1e-5 ? speed / radius_ : 0.0)) {}

    void update(double, double t, double, double) override {
        const double angle = angle0_ + omega_ * t;
        state_.x = center_x_ + radius_ * std::cos(angle);
        state_.y = center_y_ + radius_ * std::sin(angle);
    }

private:
    double center_x_, center_y_;
    double radius_;
    double angle0_;
    double omega_;  // rad/s, negative = clockwise
};



// ADVERARIAL: straight towards the robot
class AdversarialObstacle : public Obstacle {
public: 
    AdversarialObstacle(const ObstacleState& state, double speed) : Obstacle(state), speed_(speed) {}
    bool chasesRobot() const override { return true; }


    void update(double dt, double, double robot_x, double robot_y) override {
        const double dir_x = robot_x - state_.x;
        const double dir_y = robot_y - state_.y;
        const double dist = std::hypot(dir_x, dir_y);
        if ( dist < 1e-3) return;
        state_.x += dir_x / dist * speed_ * dt;
        state_.y += dir_y / dist * speed_ * dt;
    }

private:
    double speed_;

};


// RANDOM: constant speed, noisy heading, bounces on borders of an area
class RandomObstacle : public Obstacle {
public:
    RandomObstacle(const ObstacleState& state, double speed, const std::array<double, 4>& bounds, unsigned int seed)
        : Obstacle(state), speed_(speed), bounds_(bounds), rng_(seed), noise_(-1.0, 1.0) {
            heading_ = M_PI * noise_(rng_);
        }

    void update(double dt, double, double, double) override {
        // sqrt(dt): heading noise per second does not depend on update rate
        heading_ += noise_(rng_) * std::sqrt(0.175 * dt);
        double vx = speed_ * std::cos(heading_);
        double vy = speed_ * std::sin(heading_);
        double next_x = state_.x + vx * dt;
        double next_y = state_.y + vy * dt;

        if (next_x < bounds_[0] || next_x > bounds_[1]) {
            vx = -vx;
            heading_ = std::atan2(vy, vx);
            next_x = std::clamp(next_x, bounds_[0], bounds_[1]);
        }
        if (next_y < bounds_[2] || next_y > bounds_[3]) {
            vy = -vy;
            heading_ = std::atan2(vy, vx);
            next_y = std::clamp(next_y, bounds_[2], bounds_[3]);
        }

        state_.x = next_x;
        state_.y = next_y;
    }


private:
    double speed_;
    std::array<double, 4> bounds_;
    std::mt19937 rng_;
    std::uniform_real_distribution<double> noise_;
    double heading_;

};




class ObstacleManager {
public:
    explicit ObstacleManager(const LipmConfig& cfg) : n_obs_max_(cfg.N_OBS_MAX) {
        const auto& rnd = cfg.random_obstacles;
        if (static_cast<int>(cfg.obstacles.size()) + rnd.count > n_obs_max_){
            throw std::runtime_error("[ObstacleManager]: more obstacle than N_OBS_MAX!!");
        }
        for (const auto& obs : cfg.obstacles){
            const ObstacleState state{obs.x, obs.y, obs.r_obs, obs.y_dot_max};
            // WE WILL ADD NEW BEHAVIOURS HERE
            if (obs.type == "static") {
                obstacles_.push_back(std::make_unique<StaticObstacle>(state));
            } else if (obs.type == "dynamic") {
                obstacles_.push_back(std::make_unique<DynamicObstacle>(state, obs.end_x, obs.end_y, obs.speed));
            } else if (obs.type == "circular") {
                obstacles_.push_back(std::make_unique<CircularObstacle>(state, obs.center_x, obs.center_y, obs.speed, obs.clockwise));
            } else if (obs.type == "adversarial") {
                obstacles_.push_back(std::make_unique<AdversarialObstacle>(state, obs.speed));
            } else {
                throw std::runtime_error("[ObstacleManager]: unknown obstacle type '" + obs.type + "'");
            }
        }
        // Random obstacle: position, radius and speed drawn from ranges in config.yaml
        std::mt19937 rng(rnd.seed);
        auto uniform = [&rng](double min, double max) {
            return std::uniform_real_distribution<double>(min, max)(rng);
        };
        for (int i = 0; i < rnd.count; ++i){
            const ObstacleState state{
                uniform(rnd.bounds[0], rnd.bounds[1]), uniform(rnd.bounds[2], rnd.bounds[3]),
                uniform(rnd.r_range[0], rnd.r_range[1]), rnd.y_dot_max
            };
            const double speed = uniform(rnd.speed_range[0], rnd.speed_range[1]);
            obstacles_.push_back(std::make_unique<RandomObstacle>(state, speed, rnd.bounds, rng()));
        }
    }


    // robot_walking false: the obstacles that chase the robot (adversarial) stand still, the others keep moving
    void updateAll(double dt, double t, double robot_x, double robot_y, bool robot_walking = true){
        for (const auto& obs : obstacles_) {
            obs->update((robot_walking || !obs->chasesRobot()) ? dt : 0.0, t, robot_x, robot_y);
        }
    }


    // Only real obstacles
    std::vector<ObstacleState> active() const{
        std::vector<ObstacleState> states;
        for (const auto& obs : obstacles_) states.push_back(obs->state());
        return states;
    }


    // For all N_OBS_MAX obstacles: free slots are far-away ghosts
    std::vector<ObstacleState> slots() const {
        std::vector<ObstacleState> states = active();
        for (int i = static_cast<int>(states.size()); i < n_obs_max_; ++i){
            states.push_back({kGhostObsPos + 2.0 * i, kGhostObsPos, 0.0, 0.0});
        }
        return states;
    }


private:
    static constexpr double kGhostObsPos = 15;
    int n_obs_max_;
    std::vector<std::unique_ptr<Obstacle>> obstacles_;


};



} // namespace legged