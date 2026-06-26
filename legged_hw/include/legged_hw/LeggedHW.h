//
// Created by qiayuan on 6/24/22.
// Refactored for ROS 2 Control
//

#pragma once

#include <memory>
#include <string>
#include <vector>

// ROS 2 C++ client library
#include <rclcpp/rclcpp.hpp>

// ROS 2 Control Hardware Interface
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>

namespace legged {

class LeggedHW : public hardware_interface::SystemInterface {
 public:
  LeggedHW() = default;
  virtual ~LeggedHW() = default;

  // ROS 2 Control Lifecycle Methods
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

 protected:
  // Instead of standalone Interface classes, ROS 2 control holds raw data buffers
  // that are mapped directly to controllers via export_ functions.
  std::vector<double> hw_commands_positions_;
  std::vector<double> hw_commands_velocities_;
  std::vector<double> hw_commands_kps_;
  std::vector<double> hw_commands_kds_;
  std::vector<double> hw_commands_feedforward_torques_;

  std::vector<double> hw_states_positions_;
  std::vector<double> hw_states_velocities_;
  std::vector<double> hw_states_torques_;

  // IMU states
  double imu_orientation_[4] = {0.0, 0.0, 0.0, 1.0};
  double imu_angular_velocity_[3] = {0.0, 0.0, 0.0};
  double imu_linear_acceleration_[3] = {0.0, 0.0, 0.0};
};

}  // namespace legged