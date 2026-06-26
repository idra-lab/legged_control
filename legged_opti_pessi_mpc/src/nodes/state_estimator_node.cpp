/**
 * @file state_estimator_node.cpp
 * @brief Method implementations for StateEstimatorNode.
 *
 * Converts IMU and joint data from /lowstate to a standard /odom message.
 */

#include "opti_pessi_control/nodes/StateEstimatorNode.h"

// =============================================================================
//  StateEstimatorNode — constructor
// =============================================================================

StateEstimatorNode::StateEstimatorNode() : Node("state_estimator") {
    RCLCPP_INFO(this->get_logger(), "Inizializzazione State Estimator...");

    // Sottoscrizione a /lowstate
    lowstate_sub_ = this->create_subscription<unitree_go::msg::LowState>(
        "/lowstate", 10,
        std::bind(&StateEstimatorNode::lowstateCallback, this, std::placeholders::_1));

    // Publisher odometry per l'MPC
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

    // Timer a 200Hz per la pubblicazione
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(5),
        std::bind(&StateEstimatorNode::publishOdom, this));

    RCLCPP_INFO(this->get_logger(), "State Estimator pronto! (200Hz)");
}

// =============================================================================
//  Callbacks
// =============================================================================

void StateEstimatorNode::lowstateCallback(const unitree_go::msg::LowState::SharedPtr msg) {
    state_received_ = true;

    // --- IMU: orientamento e velocità angolari ---
    roll_  = msg->imu_state.rpy[0];
    pitch_ = msg->imu_state.rpy[1];
    yaw_   = msg->imu_state.rpy[2];

    gyro_x_ = msg->imu_state.gyroscope[0];
    gyro_y_ = msg->imu_state.gyroscope[1];
    gyro_z_ = msg->imu_state.gyroscope[2];

    // Quaternione dall'IMU
    qw_ = msg->imu_state.quaternion[0];
    qx_ = msg->imu_state.quaternion[1];
    qy_ = msg->imu_state.quaternion[2];
    qz_ = msg->imu_state.quaternion[3];

    // --- Giunti: posizioni e velocità ---
    for (int i = 0; i < 12; ++i) {
        joint_pos_[i] = msg->motor_state[i].q;
        joint_vel_[i] = msg->motor_state[i].dq;
    }

    // --- Stima altezza CoM dalla media delle altezze dei piedi in stance ---
    double avg_foot_z = 0.0;
    for (int leg = 0; leg < 4; ++leg) {
        double q_thigh = joint_pos_[leg * 3 + 1];
        double q_calf  = joint_pos_[leg * 3 + 2];
        avg_foot_z += footHeightFK(q_thigh, q_calf);
    }
    avg_foot_z /= 4.0;
    com_height_ = -avg_foot_z;

    // --- Stima velocità lineare del CoM (semplificata) ---
    double acc_x = msg->imu_state.accelerometer[0];
    double acc_y = msg->imu_state.accelerometer[1];

    // Integrazione semplice della velocità con decadimento per stabilità
    constexpr double dt    = 0.005;
    constexpr double decay = 0.995;
    vel_x_ = vel_x_ * decay + acc_x * dt;
    vel_y_ = vel_y_ * decay + acc_y * dt;

    // Integrazione della posizione
    double cos_yaw = std::cos(yaw_);
    double sin_yaw = std::sin(yaw_);
    pos_x_ += (vel_x_ * cos_yaw - vel_y_ * sin_yaw) * dt;
    pos_y_ += (vel_x_ * sin_yaw + vel_y_ * cos_yaw) * dt;
}

// =============================================================================
//  Publisher timer
// =============================================================================

void StateEstimatorNode::publishOdom() {
    if (!state_received_) return;

    nav_msgs::msg::Odometry odom;
    odom.header.stamp    = this->get_clock()->now();
    odom.header.frame_id = "odom";
    odom.child_frame_id  = "base_link";

    // Posizione
    odom.pose.pose.position.x = pos_x_;
    odom.pose.pose.position.y = pos_y_;
    odom.pose.pose.position.z = com_height_;

    // Orientamento (quaternione dall'IMU)
    odom.pose.pose.orientation.w = qw_;
    odom.pose.pose.orientation.x = qx_;
    odom.pose.pose.orientation.y = qy_;
    odom.pose.pose.orientation.z = qz_;

    // Velocità (nel body frame)
    odom.twist.twist.linear.x  = vel_x_;
    odom.twist.twist.linear.y  = vel_y_;
    odom.twist.twist.linear.z  = 0.0;
    odom.twist.twist.angular.x = gyro_x_;
    odom.twist.twist.angular.y = gyro_y_;
    odom.twist.twist.angular.z = gyro_z_;

    odom_pub_->publish(odom);
}

// =============================================================================
//  Entry point
// =============================================================================
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StateEstimatorNode>();
    RCLCPP_INFO(node->get_logger(), "State Estimator Node avviato.");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
