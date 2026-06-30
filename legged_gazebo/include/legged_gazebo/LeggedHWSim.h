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

// ROS 2 and Gazebo Harmonic bindings
#include <gz_ros2_control/gz_system_interface.hpp>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>

// Gazebo Sim headers
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/math/Vector3.hh>
#include <sdf/JointAxis.hh>

namespace sim = gz::sim;

namespace legged {

struct HybridJointCommand {
  rclcpp::Time stamp_;
  double posDes_{0.0}, velDes_{0.0}, kp_{0.0}, kd_{0.0}, ff_{0.0};
};

struct ImuData {
  sim::Link link_;
  std::string name_;
  double ori_[4] = {0.0, 0.0, 0.0, 1.0};
  double oriCov_[9] = {0.0};
  double angularVel_[3] = {0.0};
  double angularVelCov_[9] = {0.0};
  double linearAcc_[3] = {0.0};
  double linearAccCov_[9] = {0.0};
};

class LeggedHWSim : public gz_ros2_control::GazeboSimSystemInterface {
 public:
  LeggedHWSim() = default;
  virtual ~LeggedHWSim() = default;

  // Gazebo-Specific Initialization lifecycle hook
  bool initSim(
    rclcpp::Node::SharedPtr & model_nh,
    std::map<std::string, sim::Entity> & joints,
    const hardware_interface::HardwareInfo & hardware_info,
    sim::EntityComponentManager & _ecm,
    unsigned int update_rate) override;

  // Standard ros2_control Lifecycle hooks
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & system_info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

 private:
  sim::EntityComponentManager* ecm_{nullptr};
  sim::Model model_;

  std::vector<sim::Joint> sim_joints_;
  std::vector<gz::math::Vector3d> sim_joint_axes_;
  std::vector<sdf::JointType> sim_joint_types_;

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