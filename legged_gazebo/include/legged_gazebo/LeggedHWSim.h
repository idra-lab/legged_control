//
// Created by qiayuan on 2/10/21.
// Refactored for ROS 2 gazebo_ros2_control
//

#pragma once

#include <deque>
#include <unordered_map>
#include <vector>
#include <string>
#include <memory>

// ROS 2 and Gazebo Classic bindings
#include <gazebo_ros2_control/gazebo_system_interface.hpp>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <gazebo/physics/physics.hh>

namespace legged {

struct HybridJointCommand {
  rclcpp::Time stamp_;
  double posDes_{0.0}, velDes_{0.0}, kp_{0.0}, kd_{0.0}, ff_{0.0};
};

struct ImuData {
  gazebo::physics::LinkPtr linkPtr_;
  std::string name_;
  double ori_[4] = {0.0, 0.0, 0.0, 1.0};
  double oriCov_[9] = {0.0};
  double angularVel_[3] = {0.0};
  double angularVelCov_[9] = {0.0};
  double linearAcc_[3] = {0.0};
  double linearAccCov_[9] = {0.0};
};

class LeggedHWSim : public gazebo_ros2_control::GazeboSystemInterface {
 public:
  LeggedHWSim() = default;
  virtual ~LeggedHWSim() = default;

  // Gazebo-Specific Initialization lifecycle hook
  bool initSim(
    rclcpp::Node::SharedPtr & model_nh,
    gazebo::physics::ModelPtr parent_model,
    const hardware_interface::HardwareInfo & hardware_info,
    sdf::ElementPtr sdf) override;

  // Standard ros2_control Lifecycle hooks
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & system_info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

 private:
  gazebo::physics::ModelPtr model_;
  gazebo::physics::ContactManager* contactManager_{nullptr};

  std::vector<gazebo::physics::JointPtr> sim_joints_;

  // Command input buffers (populated by WBC controllers)
  std::vector<double> hw_commands_positions_;
  std::vector<double> hw_commands_velocities_;
  std::vector<double> hw_commands_kps_;
  std::vector<double> hw_commands_kds_;
  std::vector<double> hw_commands_feedforward_torques_;

  // State feedback buffers (read by Estimation nodes)
  std::vector<double> hw_states_positions_;
  std::vector<double> hw_states_velocities_;
  std::vector<double> hw_states_torques_;

  // External structures for simulated feedback tracking
  std::vector<ImuData> imuDatas_;
  std::unordered_map<std::string, double> name2contact_;
  std::unordered_map<std::string, std::deque<HybridJointCommand>> cmdBuffer_;

  double delay_{0.0};
};

}  // namespace legged