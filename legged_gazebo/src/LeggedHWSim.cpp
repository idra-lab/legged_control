//
// Created by qiayuan on 2/10/21.
// Refactored for ROS 2 gazebo_ros2_control
//

#include "legged_gazebo/LeggedHWSim.h"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <gz/sim/components/JointForce.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointTransmittedWrench.hh>
#include <gz/sim/components/JointType.hh>
#include <gz/sim/components/JointAxis.hh>
#include <gz/sim/components/ContactSensorData.hh>
#include <sdf/JointAxis.hh>

namespace legged {

hardware_interface::CallbackReturn LeggedHWSim::on_init(const hardware_interface::HardwareInfo & system_info) {
  std::cout << "[LeggedHWSim::on_init] Instance address: " << this << std::endl;
  if (hardware_interface::SystemInterface::on_init(system_info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Allocate data structures matching the configured joints
  hw_states_positions_.clear();
  hw_states_velocities_.clear();
  hw_states_torques_.clear();
  hw_commands_positions_.clear();
  hw_commands_velocities_.clear();
  hw_commands_kps_.clear();
  hw_commands_kds_.clear();
  hw_commands_feedforward_torques_.clear();

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

  // Pre-allocate IMU data slots if they weren't already created in initSim().
  if (imuDatas_.empty()) {
    for (const auto & sensor : info_.sensors) {
      ImuData imu;
      imu.name_ = sensor.name;
      imu.link_ = sim::Link(gz::sim::kNullEntity);
      imuDatas_.push_back(imu);
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

bool LeggedHWSim::initSim(
  rclcpp::Node::SharedPtr & model_nh,
  std::map<std::string, sim::Entity> & joints,
  const hardware_interface::HardwareInfo & hardware_info,
  sim::EntityComponentManager & _ecm,
  unsigned int /*update_rate*/) {

  std::cout << "[LeggedHWSim::initSim] initSim called on instance: " << this << std::endl;
  std::cout << "  - URDF joints count: " << hardware_info.joints.size() << std::endl;
  std::cout << "  - Gazebo map joints count: " << joints.size() << std::endl;
  ecm_ = &_ecm;

  sim::Entity modelEntity = gz::sim::kNullEntity;
  if (!joints.empty()) {
    sim::Joint firstJoint(joints.begin()->second);
    auto parentModel = firstJoint.ParentModel(_ecm);
    if (parentModel) {
      modelEntity = parentModel->Entity();
    }
  }
  model_ = sim::Model(modelEntity);

  sim_joints_.clear();
  sim_joint_axes_.clear();
  sim_joint_types_.clear();
  for (const auto & joint_info : hardware_info.joints) {
    auto it = joints.find(joint_info.name);
    if (it == joints.end()) {
      RCLCPP_ERROR(model_nh->get_logger(),
        "Joint '%s' not found in Gazebo joints map",
        joint_info.name.c_str());
      return false;
    }
    sim::Entity simjoint = it->second;
    sim_joints_.push_back(sim::Joint(simjoint));
    cmdBuffer_[joint_info.name] = std::deque<HybridJointCommand>();

    // Cache axis and type for effort projection in read()
    auto* typeComp = _ecm.Component<gz::sim::components::JointType>(simjoint);
    sim_joint_types_.push_back(typeComp ? typeComp->Data() : sdf::JointType::REVOLUTE);

    auto* axisComp = _ecm.Component<gz::sim::components::JointAxis>(simjoint);
    gz::math::Vector3d axis{0, 0, 1};
    if (axisComp) {
      auto xyz = axisComp->Data().Xyz();
      axis = {xyz[0], xyz[1], xyz[2]};
    }
    sim_joint_axes_.push_back(axis);
  }

  std::cout << "  - Populated sim_joints_ count: " << sim_joints_.size() << std::endl;

  for (auto & joint : sim_joints_) {
    joint.EnablePositionCheck(_ecm, true);
    joint.EnableVelocityCheck(_ecm, true);
    joint.EnableTransmittedWrenchCheck(_ecm, true);
    // Also explicitly create the JointTransmittedWrench component if absent
    if (!_ecm.EntityHasComponentType(joint.Entity(),
        gz::sim::components::JointTransmittedWrench().TypeId())) {
      _ecm.CreateComponent(joint.Entity(), gz::sim::components::JointTransmittedWrench());
    }
  }

  // Parse simulated actuator delay properties out of the ros2_control hardware parameters
  if (hardware_info.hardware_parameters.find("delay") != hardware_info.hardware_parameters.end()) {
    delay_ = std::stod(hardware_info.hardware_parameters.at("delay"));
  } else {
    model_nh->get_parameter_or("gazebo.delay", delay_, 0.0);
  }

  // Map contact sensor name requirements
  std::vector<std::string> contact_names = {"RF_FOOT", "LF_FOOT", "RH_FOOT", "LH_FOOT"};
  for (const auto & name : contact_names) {
    name2contact_[name] = 0.0;
  }

  // Pre-allocate IMU data slots from the sensor declarations in the URDF <ros2_control> block.
  if (imuDatas_.empty()) {
    for (const auto & sensor : hardware_info.sensors) {
      ImuData imu;
      imu.name_ = sensor.name;
      imu.link_ = sim::Link(gz::sim::kNullEntity);
      imuDatas_.push_back(imu);
    }
  }

  // Attach Link entities to the pre-allocated imuDatas_ entries.
  for (auto & imu : imuDatas_) {
    // Try sensor name first, then well-known base link names
    for (const auto & linkName : {imu.name_, std::string("base_inertia"), std::string("trunk"), std::string("base")}) {
      sim::Entity linkEntity = model_.LinkByName(_ecm, linkName);
      if (linkEntity != gz::sim::kNullEntity) {
        imu.link_ = sim::Link(linkEntity);
        imu.link_.EnableVelocityChecks(_ecm, true);
        imu.link_.EnableAccelerationChecks(_ecm, true);
        RCLCPP_INFO(model_nh->get_logger(),
          "IMU '%s' bound to link '%s'", imu.name_.c_str(), linkName.c_str());
        break;
      }
    }
    if (imu.link_.Entity() == gz::sim::kNullEntity) {
      RCLCPP_WARN(model_nh->get_logger(),
        "IMU link '%s' not found in Gazebo model — IMU data will be zeros", imu.name_.c_str());
    }
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
  static int throttle_count = 0;
  bool print_debug = (throttle_count++ % 1000 == 0);
  print_debug = false;

  if (sim_joints_.empty() || !ecm_) {
    if (print_debug) {
      std::cout << "[LeggedHWSim::read] Instance " << this << " WARNING: sim_joints_ empty or ecm_ null. Size: " << sim_joints_.size() << ", ecm_ ptr: " << ecm_ << std::endl;
    }
    return hardware_interface::return_type::OK;
  }

  for (size_t i = 0; i < sim_joints_.size(); ++i) {
    auto posOpt = sim_joints_[i].Position(*ecm_);
    if (print_debug && i == 0) {
      std::cout << "[LeggedHWSim::read] Joint " << info_.joints[i].name 
                << " has_pos_val: " << posOpt.has_value() 
                << " (empty: " << (posOpt ? posOpt->empty() : true) << ")";
      if (posOpt && !posOpt->empty()) {
        std::cout << " val: " << (*posOpt)[0];
      }
      std::cout << std::endl;
    }
    if (posOpt && !posOpt->empty()) {
      hw_states_positions_[i] = (*posOpt)[0];
    }
    auto velOpt = sim_joints_[i].Velocity(*ecm_);
    if (velOpt && !velOpt->empty()) {
      hw_states_velocities_[i] = (*velOpt)[0];
    }
    auto forceComp = ecm_->Component<gz::sim::components::JointTransmittedWrench>(
      sim_joints_[i].Entity());
    if (forceComp) {
      // Project wrench onto joint axis to get scalar effort
      gz::math::Vector3d axis = sim_joint_axes_[i];
      if (sim_joint_types_[i] == sdf::JointType::PRISMATIC) {
        hw_states_torques_[i] = forceComp->Data().force().x() * axis.X()
                              + forceComp->Data().force().y() * axis.Y()
                              + forceComp->Data().force().z() * axis.Z();
      } else {
        hw_states_torques_[i] = forceComp->Data().torque().x() * axis.X()
                              + forceComp->Data().torque().y() * axis.Y()
                              + forceComp->Data().torque().z() * axis.Z();
      }
    }
  }

  // Compute simulated IMU vector data via core Gazebo physics APIs
  for (auto & imu : imuDatas_) {
    if (imu.link_.Entity() == gz::sim::kNullEntity) continue;

    auto poseOpt = imu.link_.WorldPose(*ecm_);
    if (poseOpt) {
      auto pose = *poseOpt;
      imu.ori_[0] = pose.Rot().X();
      imu.ori_[1] = pose.Rot().Y();
      imu.ori_[2] = pose.Rot().Z();
      imu.ori_[3] = pose.Rot().W();

      auto worldAngVelOpt = imu.link_.WorldAngularVelocity(*ecm_);
      if (worldAngVelOpt) {
        auto relAngVel = pose.Rot().RotateVectorReverse(*worldAngVelOpt);
        imu.angularVel_[0] = relAngVel.X();
        imu.angularVel_[1] = relAngVel.Y();
        imu.angularVel_[2] = relAngVel.Z();
      }

      auto worldLinearAccOpt = imu.link_.WorldLinearAcceleration(*ecm_);
      if (worldLinearAccOpt) {
        gz::math::Vector3d gravity = {0.0, 0.0, -9.81};
        auto relLinearAcc = pose.Rot().RotateVectorReverse(*worldLinearAccOpt - gravity);
        imu.linearAcc_[0] = relLinearAcc.X();
        imu.linearAcc_[1] = relLinearAcc.Y();
        imu.linearAcc_[2] = relLinearAcc.Z();
      }
    }
  }

  // Extract contact collision mappings from the ECM
  for (auto & state : name2contact_) {
    state.second = 0.0;
    sim::Entity linkEntity = model_.LinkByName(*ecm_, state.first);
    if (linkEntity != gz::sim::kNullEntity) {
      sim::Link link(linkEntity);
      for (const auto & collisionEntity : link.Collisions(*ecm_)) {
        auto contactData = ecm_->Component<gz::sim::components::ContactSensorData>(collisionEntity);
        if (contactData && contactData->Data().contact_size() > 0) {
          state.second = 1.0;
          break;
        }
      }
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type LeggedHWSim::write(const rclcpp::Time & time, const rclcpp::Duration & /*period*/) {
  if (sim_joints_.empty() || !ecm_) return hardware_interface::return_type::OK;

  for (size_t i = 0; i < info_.joints.size(); ++i) {
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
    double current_pos = 0.0;
    auto posOpt = sim_joints_[i].Position(*ecm_);
    if (posOpt && !posOpt->empty()) {
      current_pos = (*posOpt)[0];
    }
    double current_vel = 0.0;
    auto velOpt = sim_joints_[i].Velocity(*ecm_);
    if (velOpt && !velOpt->empty()) {
      current_vel = (*velOpt)[0];
    }
    double torque = cmd.kp_ * (cmd.posDes_ - current_pos)
                  + cmd.kd_ * (cmd.velDes_ - current_vel)
                  + cmd.ff_;
    sim_joints_[i].SetForce(*ecm_, {torque});
  }

  return hardware_interface::return_type::OK;
}

}  // namespace legged

// Export macro so pluginlib and gz_ros2_control can recognize this component assembly
PLUGINLIB_EXPORT_CLASS(legged::LeggedHWSim, gz_ros2_control::GazeboSimSystemInterface)