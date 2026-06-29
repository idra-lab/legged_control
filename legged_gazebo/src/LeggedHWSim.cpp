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

  // Pre-allocate IMU data slots from the sensor declarations in the URDF <ros2_control> block.
  // export_state_interfaces() is called BEFORE initSim(), so we must create entries here
  // (with null linkPtr_) so the resource_manager gets valid pointer registrations.
  // initSim() will later fill in the linkPtr_ for each entry.
  for (const auto & sensor : info_.sensors) {
    ImuData imu;
    imu.name_ = sensor.name;
    imu.linkPtr_ = nullptr;
    imuDatas_.push_back(imu);
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
      RCLCPP_ERROR(model_nh->get_logger(),
        "Joint '%s' not found in Gazebo model — check URDF joint names match SDF",
        joint_info.name.c_str());
      // Store null but keep going so we can see ALL missing joints
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

  // Attach Gazebo LinkPtrs to the pre-allocated imuDatas_ entries.
  // on_init() already created the entries; here we just resolve the physics link.
  for (auto & imu : imuDatas_) {
    auto linkPtr = model_->GetLink(imu.name_);
    if (!linkPtr) {
      linkPtr = model_->GetLink("base_inertia");
    }
    if (!linkPtr) {
      linkPtr = model_->GetLink("base");
    }
    if (linkPtr) {
      imu.linkPtr_ = linkPtr;
    } else {
      RCLCPP_WARN(model_nh->get_logger(),
        "IMU link '%s' not found in Gazebo model — IMU data will be zeros", imu.name_.c_str());
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
  if (sim_joints_.empty()) return hardware_interface::return_type::OK;

  for (size_t i = 0; i < sim_joints_.size(); ++i) {
    if (!sim_joints_[i]) continue;   // ← null-check every entry

    hw_states_positions_[i] = sim_joints_[i]->Position(0);
    hw_states_velocities_[i] = sim_joints_[i]->GetVelocity(0);
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
    // CORREZIONE 2: Usa GetContactCount() per scorrere in modo sicuro solo i contatti validi
    unsigned int contact_count = contactManager_->GetContactCount();
    const auto & contacts = contactManager_->GetContacts();

    for (unsigned int i = 0; i < contact_count; ++i) {
      const auto & contact = contacts[i];
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
  if (sim_joints_.empty()) return hardware_interface::return_type::OK;

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    if (!sim_joints_[i]) continue;   // ← null-check every entry

    const std::string & name = info_.joints[i].name;
    auto & buffer = cmdBuffer_[name];

    if (buffer.empty()) {
      HybridJointCommand initial_cmd{
        .stamp_ = time,
        .posDes_ = hw_commands_positions_[i],
        .velDes_ = hw_commands_velocities_[i],
        .kp_ = hw_commands_kps_[i],
        .kd_ = hw_commands_kds_[i],
        .ff_ = hw_commands_feedforward_torques_[i]
      };
      buffer.push_front(initial_cmd);
    }

    while (buffer.size() > 1 &&
           (buffer.back().stamp_ + rclcpp::Duration::from_seconds(delay_)) < time) {
      buffer.pop_back();
    }

    HybridJointCommand current_cmd{
      .stamp_ = time,
      .posDes_ = hw_commands_positions_[i],
      .velDes_ = hw_commands_velocities_[i],
      .kp_ = hw_commands_kps_[i],
      .kd_ = hw_commands_kds_[i],
      .ff_ = hw_commands_feedforward_torques_[i]
    };
    buffer.push_front(current_cmd);

    const auto & cmd = buffer.back();
    double current_pos = sim_joints_[i]->Position(0);
    double current_vel = sim_joints_[i]->GetVelocity(0);
    double torque = cmd.kp_ * (cmd.posDes_ - current_pos)
                  + cmd.kd_ * (cmd.velDes_ - current_vel)
                  + cmd.ff_;
    sim_joints_[i]->SetForce(0, torque);
  }

  return hardware_interface::return_type::OK;
}

}  // namespace legged

// Export macro so pluginlib and gazebo_ros2_control can recognize this component assembly
PLUGINLIB_EXPORT_CLASS(legged::LeggedHWSim, gazebo_ros2_control::GazeboSystemInterface)