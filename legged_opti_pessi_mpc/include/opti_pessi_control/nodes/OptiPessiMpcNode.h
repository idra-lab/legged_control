/**
 * @file OptiPessiMpcNode.h
 * @brief OptiPessi MPC node declaration.
 *
 * Runs the SQP-based MPC solver and publishes the optimal trajectory as an
 * MpcFlattenedController message for consumption by the WBC controller node.
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

// OCS2 solver stack
#include <ocs2_mpc/MPC_BASE.h>
#include <ocs2_sqp/SqpMpc.h>
#include <ocs2_core/initialization/DefaultInitializer.h>
#include <ocs2_core/soft_constraint/StateInputSoftConstraint.h>
#include <ocs2_core/penalties/penalties/RelaxedBarrierPenalty.h>

// OCS2 message types used for the WBC pipeline
#include <ocs2_msgs/msg/mpc_flattened_controller.hpp>
#include <ocs2_msgs/msg/mode_schedule.hpp>
#include <ocs2_msgs/msg/mpc_state.hpp>
#include <ocs2_msgs/msg/mpc_input.hpp>
#include <ocs2_msgs/msg/mpc_observation.hpp>

// Custom problem components
#include "opti_pessi_control/OptiPessiSharedState.h"
#include "opti_pessi_control/OptiPessiAugmentedDynamics.h"
#include "opti_pessi_control/OptiPessiObstacleConstraint.h"
#include "opti_pessi_control/OptiPessiCouplingConstraint.h"
#include "opti_pessi_control/OptiPessiAugmentedCost.h"

// Custom ROS 2 messages
#include "legged_opti_pessi_mpc/msg/mpc_output.hpp"
#include "legged_opti_pessi_mpc/msg/footstep_command.hpp"

struct ObstacleManager {
    double x = 0.0;
    double y = 0.0;
    void update(double nx, double ny) { x = nx; y = ny; }
};

class OptiPessiMpcNode : public rclcpp::Node {
public:
    OptiPessiMpcNode();

private:
    // =========================================================================
    //  Callbacks
    // =========================================================================
    void obstacleCallback(const geometry_msgs::msg::Point::SharedPtr msg);
    void goalCallback(const geometry_msgs::msg::Point::SharedPtr msg);
    void robotStateCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    // =========================================================================
    //  Main MPC control loop (50 Hz)
    // =========================================================================
    void mpcControlLoop();

    // =========================================================================
    //  Member variables
    // =========================================================================

    double t_start_       = -1.0;
    bool   state_received_ = false;
    double current_yaw_   = 0.0;

    // ROS 2
    rclcpp::Publisher<ocs2_msgs::msg::MpcFlattenedController>::SharedPtr mpc_policy_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr             robot_state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr           obstacle_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr           goal_sub_;
    rclcpp::TimerBase::SharedPtr                                         control_loop_timer_;

    // OCS2
    std::unique_ptr<ocs2::SqpMpc>                         mpc_solver_;
    ocs2::vector_t                                        current_augmented_state_;
    ocs2::scalar_t                                        current_time_;
    std::shared_ptr<ocs2::quadruped::ObstacleState>       obs_state_;
};
