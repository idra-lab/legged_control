//
// Created by qiayuan on 1/24/22.
// Refactored for ROS 2 Control SystemInterface
//

#include "legged_unitree_hw/UnitreeHW.h"

#ifdef UNITREE_SDK_3_3_1
#include "unitree_legged_sdk_3_3_1/unitree_joystick.h"
#elif UNITREE_SDK_3_8_0
#include "unitree_legged_sdk_3_8_0/joystick.h"
#endif

#include <pluginlib/class_list_macros.hpp>

namespace legged {

hardware_interface::CallbackReturn UnitreeHW::on_init(const hardware_interface::HardwareInfo & info) {
  // Trigger parent lifecycle memory allocations
  if (LeggedHW::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Create internal node context for publisher registrations
  node_ = rclcpp::Node::make_shared("legged_unitree_hw_node");

  // Load hardware parameters directly from the URDF's <ros2_control> block configuration map
  try {
    powerLimit_ = std::stoi(info_.hardware_parameters.at("power_limit"));
    contactThreshold_ = std::stod(info_.hardware_parameters.at("contact_threshold"));
  } catch (const std::exception & e) {
    RCLCPP_FATAL(node_->get_logger(), "Required parameter missing in ros2_control URDF config: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Dynamically map URDF configuration joint sequences to the Unitree SDK 0-11 sequence structure
  urdfToSdkIndex_.resize(info_.joints.size(), -1);
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const auto & joint_name = info_.joints[i].name;
    int leg_index = 0;
    int joint_index = 0;

    if (joint_name.find("RF") != std::string::npos) leg_index = UNITREE_LEGGED_SDK::FR_;
    else if (joint_name.find("LF") != std::string::npos) leg_index = UNITREE_LEGGED_SDK::FL_;
    else if (joint_name.find("RH") != std::string::npos) leg_index = UNITREE_LEGGED_SDK::RR_;
    else if (joint_name.find("LH") != std::string::npos) leg_index = UNITREE_LEGGED_SDK::RL_;
    else continue;

    if (joint_name.find("HAA") != std::string::npos) joint_index = 0;
    else if (joint_name.find("HFE") != std::string::npos) joint_index = 1;
    else if (joint_name.find("KFE") != std::string::npos) joint_index = 2;
    else continue;

    urdfToSdkIndex_[i] = leg_index * 3 + joint_index;
  }

#ifdef UNITREE_SDK_3_3_1
  udp_ = std::make_shared<UNITREE_LEGGED_SDK::UDP>(UNITREE_LEGGED_SDK::LOWLEVEL);
#elif UNITREE_SDK_3_8_0
  udp_ = std::make_shared<UNITREE_LEGGED_SDK::UDP>(UNITREE_LEGGED_SDK::LOWLEVEL, 8090, "192.168.123.10", 8007);
#endif

  udp_->InitCmdData(lowCmd_);

  std::string robot_type = info_.hardware_parameters.at("robot_type");
#ifdef UNITREE_SDK_3_3_1
  if (robot_type == "a1") {
    safety_ = std::make_shared<UNITREE_LEGGED_SDK::Safety>(UNITREE_LEGGED_SDK::LeggedType::A1);
  } else if (robot_type == "aliengo") {
    safety_ = std::make_shared<UNITREE_LEGGED_SDK::Safety>(UNITREE_LEGGED_SDK::LeggedType::Aliengo);
  }
#elif UNITREE_SDK_3_8_0
  if (robot_type == "go1") {
    safety_ = std::make_shared<UNITREE_LEGGED_SDK::Safety>(UNITREE_LEGGED_SDK::LeggedType::Go1);
  }
#endif
  else {
    RCLCPP_FATAL(node_->get_logger(), "Unknown robot type configured: %s", robot_type.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  joyPublisher_ = node_->create_publisher<sensor_msgs::msg::Joy>("/joy", 10);
  contactPublisher_ = node_->create_publisher<std_msgs::msg::Int16MultiArray>("/contact", 10);

  lastJoyPub_ = node_->now();
  lastContactPub_ = node_->now();

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type UnitreeHW::read(const rclcpp::Time & time, const rclcpp::Duration & /*period*/) {
  udp_->Recv();
  udp_->GetRecv(lowState_);

  // Populate standard joint state vector buffers
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    int sdk_idx = urdfToSdkIndex_[i];
    if (sdk_idx >= 0 && sdk_idx < 12) {
      hw_states_positions_[i] = lowState_.motorState[sdk_idx].q;
      hw_states_velocities_[i] = lowState_.motorState[sdk_idx].dq;
      hw_states_torques_[i] = lowState_.motorState[sdk_idx].tauEst;
    }
  }

  // Populate IMU state values directly into base LeggedHW buffer fields
  imu_orientation_[0] = lowState_.imu.quaternion[1]; // x
  imu_orientation_[1] = lowState_.imu.quaternion[2]; // y
  imu_orientation_[2] = lowState_.imu.quaternion[3]; // z
  imu_orientation_[3] = lowState_.imu.quaternion[0]; // w
  
  imu_angular_velocity_[0] = lowState_.imu.gyroscope[0];
  imu_angular_velocity_[1] = lowState_.imu.gyroscope[1];
  imu_angular_velocity_[2] = lowState_.imu.gyroscope[2];
  
  imu_linear_acceleration_[0] = lowState_.imu.accelerometer[0];
  imu_linear_acceleration_[1] = lowState_.imu.accelerometer[1];
  imu_linear_acceleration_[2] = lowState_.imu.accelerometer[2];

  updateJoystick(time);
  updateContact(time);

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type UnitreeHW::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
  // Map target values out of ros2_control buffers into the low-level vendor driver command struct
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    int sdk_idx = urdfToSdkIndex_[i];
    if (sdk_idx >= 0 && sdk_idx < 12) {
      lowCmd_.motorCmd[sdk_idx].q = static_cast<float>(hw_commands_positions_[i]);
      lowCmd_.motorCmd[sdk_idx].dq = static_cast<float>(hw_commands_velocities_[i]);
      lowCmd_.motorCmd[sdk_idx].Kp = static_cast<float>(hw_commands_kps_[i]);
      lowCmd_.motorCmd[sdk_idx].Kd = static_cast<float>(hw_commands_kds_[i]);
      lowCmd_.motorCmd[sdk_idx].tau = static_cast<float>(hw_commands_feedforward_torques_[i]);
    }
  }

  safety_->PositionLimit(lowCmd_);
  safety_->PowerProtect(lowCmd_, lowState_, powerLimit_);
  udp_->SetSend(lowCmd_);
  udp_->Send();

  return hardware_interface::return_type::OK;
}

void UnitreeHW::updateJoystick(const rclcpp::Time & time) {
  if ((time - lastJoyPub_).seconds() < (1.0 / 50.0)) {
    return;
  }
  lastJoyPub_ = time;

  xRockerBtnDataStruct keyData;
  std::memcpy(&keyData, &lowState_.wirelessRemote[0], 40);

  sensor_msgs::msg::Joy joyMsg;
  joyMsg.header.stamp = time;
  joyMsg.axes.push_back(-keyData.lx);
  joyMsg.axes.push_back(keyData.ly);
  joyMsg.axes.push_back(-keyData.rx);
  joyMsg.axes.push_back(keyData.ry);
  joyMsg.buttons.push_back(keyData.btn.components.X);
  joyMsg.buttons.push_back(keyData.btn.components.A);
  joyMsg.buttons.push_back(keyData.btn.components.B);
  joyMsg.buttons.push_back(keyData.btn.components.Y);
  joyMsg.buttons.push_back(keyData.btn.components.L1);
  joyMsg.buttons.push_back(keyData.btn.components.R1);
  joyMsg.buttons.push_back(keyData.btn.components.L2);
  joyMsg.buttons.push_back(keyData.btn.components.R2);
  joyMsg.buttons.push_back(keyData.btn.components.select);
  joyMsg.buttons.push_back(keyData.btn.components.start);
  joyPublisher_->publish(joyMsg);
}

void UnitreeHW::updateContact(const rclcpp::Time & time) {
  if ((time - lastContactPub_).seconds() < (1.0 / 50.0)) {
    return;
  }
  lastContactPub_ = time;

  std_msgs::msg::Int16MultiArray contactMsg;
  for (size_t i = 0; i < CONTACT_SENSOR_NAMES.size(); ++i) {
    contactMsg.data.push_back(lowState_.footForce[i]);
  }
  contactPublisher_->publish(contactMsg);
}

}  // namespace legged

// Register with pluginlib so ros2_control manager can discover this driver
PLUGINLIB_EXPORT_CLASS(legged::UnitreeHW, hardware_interface::SystemInterface)