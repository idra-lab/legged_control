//
// Refactored for ROS 2 Control
//

#pragma once

#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <mutex>

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

#include <opti_pessi_interface/OptiPessiInterface.h>
#include <opti_pessi_interface/SolverBackend.h>

// HybridJointHandle, ImuSensorHandle and ContactSensorHandle are reused from here: redefining them
// in namespace legged would clash with LeggedController.h in every translation unit that sees both.
#include "legged_controllers/LeggedController.h"
#include "legged_controllers/SafetyChecker.h"
#include "legged_controllers/visualization/LeggedSelfCollisionVisualization.h"

namespace legged {
using namespace ocs2;
using namespace legged_robot;

class OptiPessiController : public controller_interface::ControllerInterface {
 public:
  OptiPessiController() = default;
  ~OptiPessiController() override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

 protected:
  virtual void updateStateEstimation(const rclcpp::Time& time, const rclcpp::Duration& period);

  virtual void setupOptiPessiInterface(const std::string& optipessiFile, const std::string& scenarioFile,
                                       const std::string& libraryFolder, bool recompile, opti_pessi::SolverBackend backend);
  virtual void setupLeggedInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                    bool verbose);
  virtual void setupOptiPessiMpc();
  virtual void setupLeggedMpc();
  virtual void setupOptiPessiMrt();
  virtual void setupLeggedMrt();
  virtual void setupStateEstimate(const std::string& taskFile, bool verbose);

  /** Hands the current LIP phase to the Opti-Pessi MPC. Runs on the MPC thread, right before advanceMpc(). */
  void pushOptiPessiObservation();

  /**
   * 10-dof LIP state [cx, cy, theta, dcx, dcy, dtheta, p0x, p0y, p1x, p1y] of the measured robot:
   * c from pinocchio::centerOfMass, yaw and velocities from measuredRbdState_, stance feet of `phase`
   * (see opti_pessi::gaitPair) from forward kinematics.
   */
  vector_t measureLipState(size_t phase) const;

  // Interface
  std::shared_ptr<opti_pessi::OptiPessiInterface> optiPessiInterface_;
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
  std::shared_ptr<MPC_BASE> optiPessiMpc_;
  std::shared_ptr<MPC_BASE> leggedMpc_;
  std::shared_ptr<MPC_MRT_Interface> optiPessiMrtInterface_;
  std::shared_ptr<MPC_MRT_Interface> leggedMrtInterface_;

  // Opti-Pessi LIP loop, closed on the measured robot at every phase start (see measureLipState()).
  // update() writes the phase and its start state under the mutex, the MPC thread reads them.
  std::mutex optiPessiPhaseMutex_;
  size_t optiPessiPhase_ = 0;              // contact phases completed, i.e. the gait offset of the current solve
  vector_t optiPessiRobotState_;           // measured 10-dof LIP state at the start of the current phase
  scalar_t optiPessiPhaseElapsed_ = 0.0;   // seconds spent in the current phase
  bool optiPessiGoalReached_ = false;

  // Visualization
  std::shared_ptr<LeggedRobotVisualizer> robotVisualizer_;
  std::shared_ptr<LeggedSelfCollisionVisualization> selfCollisionVisualization_;

  // ROS 2 publishers and subscribers
  rclcpp::Publisher<ocs2_msgs::msg::MpcObservation>::SharedPtr optiPessiObservationPublisher_;
  rclcpp::Publisher<ocs2_msgs::msg::MpcObservation>::SharedPtr leggedObservationPublisher_;
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr contactSub_;

  rclcpp::Node::SharedPtr ros2_node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::thread spin_thread_;

  contact_flag_t topicContacts_{};

 private:
  std::thread optiPessiMpcThread_;
  std::thread leggedMpcThread_;
  std::atomic_bool controllerRunning_{}, mpcRunning_{};
  benchmark::RepeatedTimer optiPessiMpcTimer_;
  benchmark::RepeatedTimer leggedMpcTimer_;
  benchmark::RepeatedTimer wbcTimer_;
};

}  // namespace legged
