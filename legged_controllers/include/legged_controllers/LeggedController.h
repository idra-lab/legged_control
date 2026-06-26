//
// Refactored for ROS 2 Control
//

#pragma once

#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <ocs2_msgs/msg/mpc_observation.hpp>

#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_core/misc/Benchmark.h>
#include <ocs2_legged_robot_ros/visualization/LeggedRobotVisualizer.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>

#include <legged_estimation/StateEstimateBase.h>
#include <legged_interface/LeggedInterface.h>
#include <legged_wbc/WbcBase.h>

#include "legged_controllers/SafetyChecker.h"
#include "legged_controllers/visualization/LeggedSelfCollisionVisualization.h"

namespace legged {
using namespace ocs2;
using namespace legged_robot;

struct HybridJointHandle {
  hardware_interface::LoanedStateInterface* positionState{nullptr};
  hardware_interface::LoanedStateInterface* velocityState{nullptr};
  hardware_interface::LoanedStateInterface* effortState{nullptr};
  hardware_interface::LoanedCommandInterface* positionCmd{nullptr};
  hardware_interface::LoanedCommandInterface* velocityCmd{nullptr};
  hardware_interface::LoanedCommandInterface* kpCmd{nullptr};
  hardware_interface::LoanedCommandInterface* kdCmd{nullptr};
  hardware_interface::LoanedCommandInterface* effortCmd{nullptr};

  double getPosition() const { return positionState->get_value(); }
  double getVelocity() const { return velocityState->get_value(); }
  double getEffort() const { return effortState->get_value(); }
  void setCommand(double pos, double vel, double kp, double kd, double effort) {
    positionCmd->set_value(pos);
    velocityCmd->set_value(vel);
    kpCmd->set_value(kp);
    kdCmd->set_value(kd);
    effortCmd->set_value(effort);
  }
};

struct ImuSensorHandle {
  std::vector<hardware_interface::LoanedStateInterface*> orientation; // size 4
  std::vector<hardware_interface::LoanedStateInterface*> angularVelocity; // size 3
  std::vector<hardware_interface::LoanedStateInterface*> linearAcceleration; // size 3
  
  const double* getOrientation() const {
    static double ori[4];
    for (size_t i = 0; i < 4; ++i) ori[i] = orientation[i]->get_value();
    return ori;
  }
  const double* getAngularVelocity() const {
    static double ang[3];
    for (size_t i = 0; i < 3; ++i) ang[i] = angularVelocity[i]->get_value();
    return ang;
  }
  const double* getLinearAcceleration() const {
    static double lin[3];
    for (size_t i = 0; i < 3; ++i) lin[i] = linearAcceleration[i]->get_value();
    return lin;
  }
  const double* getOrientationCovariance() const {
    static const double cov[9] = {0};
    return cov;
  }
  const double* getAngularVelocityCovariance() const {
    static const double cov[9] = {0};
    return cov;
  }
  const double* getLinearAccelerationCovariance() const {
    static const double cov[9] = {0};
    return cov;
  }
};

struct ContactSensorHandle {
  hardware_interface::LoanedStateInterface* contactState{nullptr};
  bool isContact() const {
    if (contactState) {
      return contactState->get_value() > 0.5;
    }
    return false;
  }
};

class LeggedController : public controller_interface::ControllerInterface {
 public:
  LeggedController() = default;
  ~LeggedController() override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

 protected:
  virtual void updateStateEstimation(const rclcpp::Time& time, const rclcpp::Duration& period);

  virtual void setupLeggedInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                    bool verbose);
  virtual void setupMpc();
  virtual void setupMrt();
  virtual void setupStateEstimate(const std::string& taskFile, bool verbose);

  // Interface
  std::shared_ptr<LeggedInterface> leggedInterface_;
  std::shared_ptr<PinocchioEndEffectorKinematics> eeKinematicsPtr_;
  std::vector<HybridJointHandle> hybridJointHandles_;
  std::vector<ContactSensorHandle> contactHandles_;
  ImuSensorHandle imuSensorHandle_;

  // State Estimation
  SystemObservation currentObservation_;
  vector_t measuredRbdState_;
  std::shared_ptr<StateEstimateBase> stateEstimate_;
  std::shared_ptr<CentroidalModelRbdConversions> rbdConversions_;

  // Whole Body Control
  std::shared_ptr<WbcBase> wbc_;
  std::shared_ptr<SafetyChecker> safetyChecker_;

  // Nonlinear MPC
  std::shared_ptr<MPC_BASE> mpc_;
  std::shared_ptr<MPC_MRT_Interface> mpcMrtInterface_;

  // Visualization
  std::shared_ptr<LeggedRobotVisualizer> robotVisualizer_;
  std::shared_ptr<LeggedSelfCollisionVisualization> selfCollisionVisualization_;
  
  // ROS 2 publishers and subscribers
  rclcpp::Publisher<ocs2_msgs::msg::MpcObservation>::SharedPtr observationPublisher_;
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr contactSub_;

  rclcpp::Node::SharedPtr ros2_node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::thread spin_thread_;

  contact_flag_t topicContacts_{};

 private:
  std::thread mpcThread_;
  std::atomic_bool controllerRunning_{}, mpcRunning_{};
  benchmark::RepeatedTimer mpcTimer_;
  benchmark::RepeatedTimer wbcTimer_;
};

class LeggedCheaterController : public LeggedController {
 protected:
  void setupStateEstimate(const std::string& taskFile, bool verbose) override;
};

}  // namespace legged
