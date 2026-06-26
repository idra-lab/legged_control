//
// Created by qiayuan on 2022/7/24.
// Refactored for ROS 2
//

#pragma once

#include "legged_estimation/StateEstimateBase.h"
#include <realtime_tools/realtime_buffer.hpp>

namespace legged {
using namespace ocs2;

class FromTopicStateEstimate : public StateEstimateBase {
 public:
  FromTopicStateEstimate(rclcpp::Node::SharedPtr node, PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                         const PinocchioEndEffectorKinematics& eeKinematics);

  void updateImu(const Eigen::Quaternion<scalar_t>& quat, const vector3_t& angularVelLocal, const vector3_t& linearAccelLocal,
                 const matrix3_t& orientationCovariance, const matrix3_t& angularVelCovariance,
                 const matrix3_t& linearAccelCovariance) override{};

  vector_t update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

 private:
  void callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  realtime_tools::RealtimeBuffer<nav_msgs::msg::Odometry> buffer_;
};

}  // namespace legged