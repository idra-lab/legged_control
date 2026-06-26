//
// Created by qiayuan on 1/24/22.
// Refactored for ROS 2 Control SystemInterface
//

#pragma once

#include <vector>
#include <string>
#include <memory>

#include <legged_hw/LeggedHW.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>

#ifdef UNITREE_SDK_3_3_1
#include "unitree_legged_sdk_3_3_1/safety.h"
#include "unitree_legged_sdk_3_3_1/udp.h"
#elif UNITREE_SDK_3_8_0
#include "unitree_legged_sdk_3_8_0/safety.h"
#include "unitree_legged_sdk_3_8_0/udp.h"
#endif

namespace legged {
const std::vector<std::string> CONTACT_SENSOR_NAMES = {"RF_FOOT", "LF_FOOT", "RH_FOOT", "LH_FOOT"};

class UnitreeHW : public LeggedHW {
 public:
  UnitreeHW() = default;
  virtual ~UnitreeHW() = default;

  // ROS 2 Control Lifecycle Methods
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  void updateJoystick(const rclcpp::Time & time);

  void updateContact(const rclcpp::Time & time);

 private:
  // Maps URDF joint list ordering to the low-level Unitree 0-11 SDK index structure
  std::vector<int> urdfToSdkIndex_;

  std::shared_ptr<UNITREE_LEGGED_SDK::UDP> udp_;
  std::shared_ptr<UNITREE_LEGGED_SDK::Safety> safety_;
  UNITREE_LEGGED_SDK::LowState lowState_{};
  UNITREE_LEGGED_SDK::LowCmd lowCmd_{};

  int powerLimit_{0};
  double contactThreshold_{0.0};

  // Internal node context required to handle topic publishing inside a plugin environment
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joyPublisher_;
  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr contactPublisher_;
  rclcpp::Time lastJoyPub_, lastContactPub_;
};

}  // namespace legged