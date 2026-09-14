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
#include <visualization_msgs/msg/marker_array.hpp>

#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_core/misc/Benchmark.h>
#include <ocs2_legged_robot_ros/visualization/LeggedRobotVisualizer.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>

#include <legged_estimation/StateEstimateBase.h>
#include <legged_interface/LeggedInterface.h>
#include <legged_wbc/OptiPessiWbc.h>

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
   * (see opti_pessi::gaitPair) from forward kinematics. `footPositions` receives all four feet in
   * odom, indexed by opti_pessi::Foot.
   */
  vector_t measureLipState(size_t phase, std::array<vector3_t, 4>& footPositions) const;

  /**
   * Contact forces of the stance pair (foot 0, foot 1) of one LIP knot, in odom:
   *   f0 = (beta m ddcx, gamma m ddcy, (1 - alpha) m g),   f1 = ((1 - beta) m ddcx, (1 - gamma) m ddcy, alpha m g),
   * with ddc = omega^2 (c - z) and z = p0 + alpha (p1 - p0) the CoP.
   */
  static std::array<vector3_t, 2> computeContactForces(const vector_t& robotState, const vector_t& robotInput,
                                                       const opti_pessi::OptiPessiModelParameters& params);

  /**
   * Fills optiPessiFootReferences_ at `time` seconds into the current phase. The stance pair of the
   * phase holds its liftoff position and carries the knot-i forces; the other pair swings from its
   * liftoff position to the next footholds of robotInput along cubic splines with zero velocity at
   * both ends and an apex optiPessiSwingHeight_ above liftoff at mid-phase, landing with the knot-(i+1)
   * forces. The two pairs swap every phase.
   */
  void updateFootReferences(const vector_t& robotState, const vector_t& robotInput, const vector_t& nextRobotState,
                            const vector_t& nextRobotInput, scalar_t time);

  /**
   * Runs the LIP clock: evaluates the policy of the current phase into the foot and CoM references, and
   * at the end of a phase measures the robot for the next one. While the MPC has not solved the current
   * phase the references coast; once the goal is reached they stand.
   */
  void advanceOptiPessiPhase(const rclcpp::Time& time, const rclcpp::Duration& period);

  /**
   * Fills optiPessiComReference_ at `time` seconds into the current phase: closed-form LIP flow of
   * robotState with the CoP held at alpha, at height comHeight, and the yaw driven by the torque of the
   * tangential forces, as in opti_pessi::lipMap.
   */
  void updateComReference(const vector_t& robotState, const vector_t& robotInput, scalar_t time);

  /** End of phase: the swing pair lands with its touchdown forces, the old stance pair stays down unloaded. */
  void landSwingFeet();

  /** Every foot down where its reference is, loaded with m g / 4; the CoM at rest at comPosition, heading yaw. */
  void holdStance(const vector3_t& comPosition, scalar_t yaw);

  /** Runs OptiPessiWbc on the current references and writes the joint commands. False if the safety check fails. */
  bool updateWholeBodyControl(const rclcpp::Duration& period);

  /** Reference of the CoM and heading over the current phase, in odom. */
  struct ComReference {
    vector3_t position = vector3_t::Zero();
    vector3_t velocity = vector3_t::Zero();
    vector3_t acceleration = vector3_t::Zero();
    scalar_t yaw = 0.0;
    scalar_t yawRate = 0.0;
    scalar_t yawAcceleration = 0.0;
  };

  /** Reference of one foot over the current phase, in odom. */
  struct FootReference {
    bool contact = true;
    vector3_t position = vector3_t::Zero();
    vector3_t velocity = vector3_t::Zero();
    vector3_t force = vector3_t::Zero();           // contact force now, zero while swinging
    vector3_t touchdownForce = vector3_t::Zero();  // contact force at the end of the phase
  };

  /** Draws the active Opti-Pessi policy in odom: pessimistic CoM path per knot, obstacles, goal. */
  void publishOptiPessiPlan();

  /**
   * Draws the executed (optimistic) plan over the horizon like LeggedRobotVisualizer's optimized state
   * trajectory: continuous CoM path of the LIP flow, per-foot stance/swing paths, future footholds.
   */
  void publishOptiPessiTrajectories();

  /**
   * Swing foot at `time` in [0, duration]: cubic splines in x and y from liftoff to touchdown, and in z
   * through an apex swingHeight above the higher end at mid-swing; zero velocity at both ends.
   */
  static void evaluateSwing(const vector3_t& liftoff, const vector3_t& touchdown, scalar_t duration, scalar_t swingHeight,
                            scalar_t time, vector3_t& position, vector3_t& velocity);

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
  std::shared_ptr<OptiPessiWbc> wbc_;
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

  // Swing/stance references of the four feet, indexed by opti_pessi::Foot. Control thread only.
  std::array<vector3_t, 4> optiPessiLiftoffPositions_{};  // measured feet at the start of the current phase
  std::array<FootReference, 4> optiPessiFootReferences_{};
  scalar_t optiPessiSwingHeight_ = 0.08;                   // swing apex above liftoff [m]
  ComReference optiPessiComReference_{};                   // CoM and heading reference of the WBC. Control thread only.

  // Visualization
  std::shared_ptr<LeggedRobotVisualizer> robotVisualizer_;
  std::shared_ptr<LeggedSelfCollisionVisualization> selfCollisionVisualization_;

  // ROS 2 publishers and subscribers
  rclcpp::Publisher<ocs2_msgs::msg::MpcObservation>::SharedPtr optiPessiObservationPublisher_;
  rclcpp::Publisher<ocs2_msgs::msg::MpcObservation>::SharedPtr leggedObservationPublisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr optiPessiPlanPublisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr optiPessiTrajectoryPublisher_;
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
