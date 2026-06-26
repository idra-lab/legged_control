//
// Created by qiayuan on 1/24/22.
// Refactored for ROS 2 Control SystemInterface
//

#include "legged_hw/LeggedHW.h"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace legged {

hardware_interface::CallbackReturn LeggedHW::on_init(const hardware_interface::HardwareInfo & info) {
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Allocate data storage slots matching the hardware profile configurations parsed from the URDF
  for (const auto & joint : info_.joints) {
    // State layout tracking
    hw_states_positions_.push_back(0.0);
    hw_states_velocities_.push_back(0.0);
    hw_states_torques_.push_back(0.0);

    // Command interface mapping tracking 
    hw_commands_positions_.push_back(0.0);
    hw_commands_velocities_.push_back(0.0);
    hw_commands_kps_.push_back(0.0);
    hw_commands_kds_.push_back(0.0);
    hw_commands_feedforward_torques_.push_back(0.0);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> LeggedHW::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Export standard joint reading variables
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocities_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_states_torques_[i]));
  }

  // Export IMU state reading parameters if configured in the URDF data profile
  if (!info_.sensors.empty()) {
    const auto & imu_sensor = info_.sensors[0];
    state_interfaces.emplace_back(hardware_interface::StateInterface(imu_sensor.name, "orientation.x", &imu_orientation_[0]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(imu_sensor.name, "orientation.y", &imu_orientation_[1]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(imu_sensor.name, "orientation.z", &imu_orientation_[2]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(imu_sensor.name, "orientation.w", &imu_orientation_[3]));

    state_interfaces.emplace_back(hardware_interface::StateInterface(imu_sensor.name, "angular_velocity.x", &imu_angular_velocity_[0]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(imu_sensor.name, "angular_velocity.y", &imu_angular_velocity_[1]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(imu_sensor.name, "angular_velocity.z", &imu_angular_velocity_[2]));

    state_interfaces.emplace_back(hardware_interface::StateInterface(imu_sensor.name, "linear_acceleration.x", &imu_linear_acceleration_[0]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(imu_sensor.name, "linear_acceleration.y", &imu_linear_acceleration_[1]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(imu_sensor.name, "linear_acceleration.z", &imu_linear_acceleration_[2]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> LeggedHW::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Export the standard hybrid command parameters that the OCS2 WBC controller populates
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_positions_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocities_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, "kp", &hw_commands_kps_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, "kd", &hw_commands_kds_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_commands_feedforward_torques_[i]));
  }

  return command_interfaces;
}

hardware_interface::return_type LeggedHW::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
  // TODO: Add your low-level hardware interface reading logic here (e.g., Unitree SDK UDP recv)
  // Fill the hw_states_* vectors and imu_* arrays from raw robot communication packets.
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type LeggedHW::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
  // TODO: Add your low-level hardware interface command dispatch logic here (e.g., Unitree SDK UDP send)
  // Read target values out of the hw_commands_* vectors and pack them into the low-level vendor SDK structs.
  return hardware_interface::return_type::OK;
}

}  // namespace legged

// Export macro so pluginlib can recognize this package target as a viable ros2_control backend
PLUGINLIB_EXPORT_CLASS(legged::LeggedHW, hardware_interface::SystemInterface)