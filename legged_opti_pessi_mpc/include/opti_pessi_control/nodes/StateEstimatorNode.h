/**
 * @file StateEstimatorNode.h
 * @brief State estimator node declaration: /lowstate → /odom.
 *
 * Converts IMU and joint data from the Unitree LowState message into a
 * standard Odometry message to feed the MPC node.
 *
 * Logic:
 * - Orientation is taken directly from the IMU (quaternion/RPY)
 * - Position is estimated by integrating the linear velocity derived from
 *   trunk FK + joint velocities
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <unitree_go/msg/low_state.hpp>
#include <cmath>

namespace {

// Go2 kinematic constants
constexpr double SE_THIGH_LENGTH  = 0.213;
constexpr double SE_CALF_LENGTH   = 0.213;
constexpr double SE_HIP_OFFSET    = 0.0955;
constexpr double SE_LEG_OFFSET_X  = 0.1934;
constexpr double SE_LEG_OFFSET_Y  = 0.0465;
constexpr double SE_NOMINAL_HEIGHT = 0.23;

/**
 * @brief Simplified FK for one leg: returns foot z in the trunk frame,
 *        used to estimate CoM height.
 */
inline double footHeightFK(double q_thigh, double q_calf) {
    return -(SE_THIGH_LENGTH * std::cos(q_thigh) +
             SE_CALF_LENGTH  * std::cos(q_thigh + q_calf));
}

} // anonymous namespace

class StateEstimatorNode : public rclcpp::Node {
public:
    StateEstimatorNode();

private:
    // =========================================================================
    //  Callbacks
    // =========================================================================
    void lowstateCallback(const unitree_go::msg::LowState::SharedPtr msg);

    // =========================================================================
    //  Publisher timer
    // =========================================================================
    void publishOdom();

    // =========================================================================
    //  Member variables
    // =========================================================================

    // ROS 2
    rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr lowstate_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr      odom_pub_;
    rclcpp::TimerBase::SharedPtr                               timer_;

    // State
    bool state_received_ = false;

    // Estimated position
    double pos_x_ = 0.0, pos_y_ = 0.0;
    double vel_x_ = 0.0, vel_y_ = 0.0;
    double com_height_ = SE_NOMINAL_HEIGHT;

    // IMU
    double roll_ = 0.0, pitch_ = 0.0, yaw_ = 0.0;
    double gyro_x_ = 0.0, gyro_y_ = 0.0, gyro_z_ = 0.0;
    double qw_ = 1.0, qx_ = 0.0, qy_ = 0.0, qz_ = 0.0;

    // Joints
    double joint_pos_[12] = {};
    double joint_vel_[12] = {};
};
