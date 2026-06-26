//
// Created by qiayuan on 2/10/21.
// Refactored for ROS 2 gazebo_ros2_control
//

#include "legged_gazebo/LeggedHWSim.h"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace legged {

hardware_interface::CallbackReturn LeggedHWSim::on_init(const hardware_interface::HardwareInfo & system_info) {
  if (hardware_interface::SystemInterface::on_init(system_info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Allocate data structures matching the configured joints
  for (const auto & joint : info_.joints) {
    hw_states_positions_.push_back(0.0);
    hw_states_velocities_.push_back(0.0);
    hw_states_torques_.push_back(0.0);

    hw_commands_positions_.push_back(0.0);
    hw_commands_velocities_.push_back(0.0);
    hw_commands_kps_.push_back(0.0);
    hw_commands_kds_.push_back(0.0);
    hw_commands_feedforward_torques_.push_back(0.0);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

bool LeggedHWSim::initSim(
  rclcpp::Node::SharedPtr & model_nh,
  gazebo::physics::ModelPtr parent_model,
  const hardware_interface::HardwareInfo & /*hardware_info*/,
  sdf::ElementPtr /*sdf*/) {
  
  model_ = parent_model;
  
  // Resolve model joint references inside the Gazebo Classic engine context
  sim_joints_.clear();
  for (const auto & joint_info : info_.joints) {
    auto sim_joint = model_->GetJoint(joint_info.name);
    if (!sim_joint) {
      RCLCPP_ERROR(model_nh->get_logger(), "Joint %s not found in gazebo model simulation context!", joint_info.name.c_str());
      return false;
    }
    sim_joints_.push_back(sim_joint);
    cmdBuffer_[joint_info.name] = std::deque<HybridJointCommand>();
  }

  // Parse simulated actuator delay properties out of the ros2_control hardware parameters
  if (info_.hardware_parameters.find("delay") != info_.hardware_parameters.end()) {
    delay_ = std::stod(info_.hardware_parameters.at("delay"));
  } else {
    model_nh->get_parameter_or("gazebo.delay", delay_, 0.0);
  }

  // Map contact sensor name requirements
  std::vector<std::string> contact_names = {"RF_FOOT", "LF_FOOT", "RH_FOOT", "LH_FOOT"};
  for (const auto & name : contact_names) {
    name2contact_[name] = 0.0;
  }

  RCLCPP_INFO(model_nh->get_logger(), "Debug: Listing all links in the Gazebo model:");
  for (const auto & link : model_->GetLinks()) {
    RCLCPP_INFO(model_nh->get_logger(), "Link name: '%s'", link->GetName().c_str());
  }

  // Parse IMU link context — root link is "base_inertia" (the actual
  // inertial link). "base" is a virtual zero-mass link that Gazebo merges.
  // "base_link" does not exist in this robot model at all.
  std::vector<std::string> imu_names = {"base_imu"};
  for (const auto & name : imu_names) {
    auto linkPtr = model_->GetLink("base");
    if (linkPtr) {
      ImuData imu;
      imu.linkPtr_ = linkPtr;
      imu.name_ = name;
      imuDatas_.push_back(imu);
    } else {
      RCLCPP_WARN(model_nh->get_logger(), "IMU link 'base' not found in Gazebo model — IMU data will not be available");
    }
  }

  contactManager_ = model_->GetWorld()->Physics()->GetContactManager();
  if (contactManager_) {
    contactManager_->SetNeverDropContacts(true);
  }

  return true;
}

std::vector<hardware_interface::StateInterface> LeggedHWSim::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocities_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_states_torques_[i]));
  }

  for (auto & imu : imuDatas_) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(imu.name_, "orientation.x", &imu.ori_[0]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(imu.name_, "orientation.y", &imu.ori_[1]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(imu.name_, "orientation.z", &imu.ori_[2]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(imu.name_, "orientation.w", &imu.ori_[3]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(imu.name_, "angular_velocity.x", &imu.angularVel_[0]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(imu.name_, "angular_velocity.y", &imu.angularVel_[1]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(imu.name_, "angular_velocity.z", &imu.angularVel_[2]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(imu.name_, "linear_acceleration.x", &imu.linearAcc_[0]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(imu.name_, "linear_acceleration.y", &imu.linearAcc_[1]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(imu.name_, "linear_acceleration.z", &imu.linearAcc_[2]));
  }

  for (auto & state : name2contact_) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(state.first, "contact", &state.second));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> LeggedHWSim::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;

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

hardware_interface::return_type LeggedHWSim::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period) {
  // Read kinematic positions and derive simulator velocity tracking inputs
  for (size_t i = 0; i < sim_joints_.size(); ++i) {
    double position = sim_joints_[i]->Position(0);
    hw_states_velocities_[i] = (position - hw_states_positions_[i]) / period.seconds();
    hw_states_positions_[i] = position;
    hw_states_torques_[i] = sim_joints_[i]->GetForce(0);
  }

  // Compute simulated IMU vector data via core Gazebo physics APIs
  for (auto & imu : imuDatas_) {
    auto pose = imu.linkPtr_->WorldPose();
    imu.ori_[0] = pose.Rot().X();
    imu.ori_[1] = pose.Rot().Y();
    imu.ori_[2] = pose.Rot().Z();
    imu.ori_[3] = pose.Rot().W();

    auto rate = imu.linkPtr_->RelativeAngularVel();
    imu.angularVel_[0] = rate.X();
    imu.angularVel_[1] = rate.Y();
    imu.angularVel_[2] = rate.Z();

    ignition::math::Vector3d gravity = {0.0, 0.0, -9.81};
    auto accel = imu.linkPtr_->RelativeLinearAccel() - pose.Rot().RotateVectorReverse(gravity);
    imu.linearAcc_[0] = accel.X();
    imu.linearAcc_[1] = accel.Y();
    imu.linearAcc_[2] = accel.Z();
  }

  // Extract contact collision mappings from the ContactManager
  for (auto & state : name2contact_) {
    state.second = 0.0;
  }
  if (contactManager_) {
    for (const auto & contact : contactManager_->GetContacts()) {
      if (contact) {
        if (contact->collision1 && contact->collision1->GetLink()) {
          std::string linkName1 = contact->collision1->GetLink()->GetName();
          if (name2contact_.find(linkName1) != name2contact_.end()) {
            name2contact_[linkName1] = 1.0;
          }
        }
        if (contact->collision2 && contact->collision2->GetLink()) {
          std::string linkName2 = contact->collision2->GetLink()->GetName();
          if (name2contact_.find(linkName2) != name2contact_.end()) {
            name2contact_[linkName2] = 1.0;
          }
        }
      }
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type LeggedHWSim::write(const rclcpp::Time & time, const rclcpp::Duration & /*period*/) {
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const std::string & name = info_.joints[i].name;
    auto & buffer = cmdBuffer_[name];

    // Reset loop command handler queue parameters on runtime changes
    if (buffer.empty()) {
      HybridJointCommand initial_cmd{.stamp_ = time, .posDes_ = hw_commands_positions_[i], .velDes_ = hw_commands_velocities_[i], .kp_ = hw_commands_kps_[i], .kd_ = hw_commands_kds_[i], .ff_ = hw_commands_feedforward_torques_[i]};
      buffer.push_front(initial_cmd);
    }

    // Retain commands lagging behind the specified artificial transmission delay period
    while (buffer.size() > 1 && (buffer.back().stamp_ + rclcpp::Duration::from_seconds(delay_)) < time) {
      buffer.pop_back();
    }

    HybridJointCommand current_cmd{.stamp_ = time, .posDes_ = hw_commands_positions_[i], .velDes_ = hw_commands_velocities_[i], .kp_ = hw_commands_kps_[i], .kd_ = hw_commands_kds_[i], .ff_ = hw_commands_feedforward_torques_[i]};
    buffer.push_front(current_cmd);

    // Apply standard hybrid torque equations straight onto the joint forces
    const auto & cmd = buffer.back();
    double current_pos = sim_joints_[i]->Position(0);
    double current_vel = sim_joints_[i]->GetVelocity(0);
    
    double torque = cmd.kp_ * (cmd.posDes_ - current_pos) + cmd.kd_ * (cmd.velDes_ - current_vel) + cmd.ff_;
    sim_joints_[i]->SetForce(0, torque);
  }

  return hardware_interface::return_type::OK;
}

}  // namespace legged

// Export macro so pluginlib and gazebo_ros2_control can recognize this component assembly
PLUGINLIB_EXPORT_CLASS(legged::LeggedHWSim, gazebo_ros2_control::GazeboSystemInterface)