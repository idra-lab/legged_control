//
// ros2_control plugin driving the quadruped with the Opti-Pessi LIP MPC instead of the centroidal NMPC.
//
// The planner decides ONE footstep per contact phase (next footholds, CoP alpha, force split beta/gamma,
// phase duration dt), not a continuous trajectory. This class turns that decision into the continuous
// CoM, yaw, foot and force references OptiPessiWbc tracks, on three threads:
//
//   control (update()) : state estimation, plan intake, LIP phase clock, references, WBC, command write-out
//   MPC     (optiPessiMpcThread_) : pushOptiPessiObservation() then advanceMpc(), at mpcDesiredFrequency_
//   spin    (spin_thread_) : goal / obstacle / contact callbacks
//
// The LIP loop closes ONCE PER PHASE: the 10-dof LIP state is measured at each phase boundary and handed
// to the MPC. Within a phase the only feedback is the WBC's PD terms. It owns both models -- LeggedInterface
// for the whole-body side (URDF, Pinocchio, estimator, WBC) and OptiPessiInterface for the planner -- but
// the centroidal SqpMpc of LeggedInterface is deliberately not built.
//

#pragma once

#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <limits>
#include <mutex>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <ocs2_msgs/msg/mpc_observation.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "legged_controllers/msg/obstacle_array.hpp"

#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_core/misc/Benchmark.h>
#include <ocs2_legged_robot_ros/visualization/LeggedRobotVisualizer.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>

#include <legged_estimation/StateEstimateBase.h>
#include <legged_interface/LeggedInterface.h>
#include <legged_wbc/OptiPessiWbc.h>

#include <opti_pessi_interface/ObstacleDetour.h>
#include <opti_pessi_interface/OptiPessiInterface.h>
#include <opti_pessi_interface/OptiPessiMpc.h>
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
   * Subscribes the goal (/opti_pessi/goal, visualization_msgs/MarkerArray: position of its first ADD marker) and the
   * obstacles (/opti_pessi/obstacles, legged_controllers/ObstacleArray), both in odom. Callbacks run on the spin thread.
   */
  void setupOptiPessiReferenceSubscribers();

  /**
   * Hands the latest goal and obstacles to the Opti-Pessi reference manager, from the MPC thread so they cannot change
   * under a solve. The OCP has a fixed number of obstacle slots (scenario obstacles.numObstacles): with more obstacles
   * than slots the ones whose keep-out disks come closest to the CoM of `robotState` take them, and free slots hold a
   * point obstacle parked far from the robot.
   */
  void pushOptiPessiReferences(const vector_t& robotState);

  /** Latest goal received and its sequence number (increments with every goal). False before the first goal. */
  bool getOptiPessiGoal(vector_t& goal, size_t& sequence);

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

  /**
   * Copies a plan just published by OptiPessiMpc into optiPessiPlan_ if it belongs to this walk and is newer than the
   * stored one: solved for the current phase from the state it started from (a saturated plan does not replace an
   * accepted one of the same phase), or for a later earlier phase than the stored one.
   */
  void storeAcceptedPlan(const opti_pessi::OptiPessiMpc::Plan& plan);

  /**
   * No usable plan and the robot nearly at rest: stops the MPC, lowers the swinging feet where they are, holds the CoM
   * at comHeight above the measured one and hands over to standUp()'s settling stage, which restarts from phase 0.
   */
  void restartFromStance();

  /** Runs OptiPessiWbc on the current references and writes the joint commands. False if the safety check fails. */
  bool updateWholeBodyControl(const rclcpp::Duration& period);

  /**
   * Stand-up stage run after activation: all feet down, CoM height ramped from the measured one to comHeight over
   * optiPessiStandUpDuration_, then optiPessiStandSettleDuration_ of holding. At its end phase 0 is measured and
   * the MPC starts.
   */
  void standUp(const rclcpp::Duration& period);

  /** Whole-body CoM of the measured robot, in odom. */
  vector3_t measureCenterOfMass() const;

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
  size_t optiPessiReachedGoalSequence_ = 0;  // goal sequence optiPessiGoalReached_ refers to. Control thread only.

  // Goal and obstacles as last received on their topics, in odom. Written by the spin thread, read by the control and
  // MPC threads, under the mutex.
  struct ObstacleObservation {
    scalar_t x = 0.0;
    scalar_t y = 0.0;
    opti_pessi::ObstacleType type = opti_pessi::ObstacleType::Human;
  };
  std::mutex optiPessiReferenceMutex_;
  vector_t optiPessiGoal_;               // empty until the first goal arrives
  size_t optiPessiGoalSequence_ = 0;     // increments with every goal received
  std::vector<ObstacleObservation> optiPessiObstacles_;

  // Detour goal around an obstacle blocking the line to the goal (see opti_pessi_interface/ObstacleDetour.h). MPC thread
  // only (pushOptiPessiReferences()).
  std::unique_ptr<opti_pessi::ObstacleDetour> optiPessiDetour_;

  /**
   * Latest Opti-Pessi plan, in robot coordinates. A phase starts on it at once -- shifted by the phases completed since
   * it was solved if its horizon is trustworthy, otherwise as a capture-point stop -- and switches to its own plan when
   * the MPC thread delivers one. Control thread only.
   */
  struct AcceptedPlan {
    bool valid = false;
    size_t phase = 0;              // phase the plan was solved for
    vector_t startState;           // measured 10-dof LIP state it was solved from
    std::vector<vector_t> inputs;  // robot inputs of knots 0..N-1
    bool trustworthy = false;      // later knots may be applied shifted
    std::string source;            // OptiPessiMpc::Plan::source
  };
  AcceptedPlan optiPessiPlan_{};
  size_t optiPessiPlanSequence_ = 0;     // OptiPessiMpc::Plan::sequence of the last plan looked at
  // task.info: optiPessiController
  size_t optiPessiMaxPlanShift_ = 1;     // knots a trustworthy plan may be shifted before the capture-point stop takes over
  scalar_t optiPessiRestartSpeed_ = 0.1;  // CoM speed below which a phase without a usable plan restarts from stance [m/s]

  // Swing/stance references of the four feet, indexed by opti_pessi::Foot. Control thread only.
  std::array<vector3_t, 4> optiPessiLiftoffPositions_{};  // measured feet at the start of the current phase
  std::array<FootReference, 4> optiPessiFootReferences_{};
  scalar_t optiPessiSwingHeight_ = 0.08;                   // swing apex above liftoff [m]
  ComReference optiPessiComReference_{};                   // CoM and heading reference of the WBC. Control thread only.

  // Stand-up stage after activation (see standUp()). Control thread only.
  bool optiPessiStandingUp_ = false;
  scalar_t optiPessiStandElapsed_ = 0.0;
  scalar_t optiPessiStandStartHeight_ = 0.0;     // CoM height measured at activation [m]
  scalar_t optiPessiStandUpDuration_ = 1.5;      // CoM height ramp [s]
  scalar_t optiPessiStandSettleDuration_ = 0.5;  // hold at comHeight before phase 0 is measured [s]

  // Per-phase diagnostics, logged at the end of each phase. Control thread only.
  scalar_t optiPessiWaitTime_ = 0.0;         // sim seconds of the phase run without its own policy (shifted plan, or none yet)
  size_t optiPessiQpFailuresAtPhaseStart_ = 0;

  /** Posture and contact statistics over one phase, accumulated every WBC tick. */
  struct PhaseDiagnostics {
    scalar_t pitchMin = std::numeric_limits<scalar_t>::max();
    scalar_t pitchMax = std::numeric_limits<scalar_t>::lowest();
    scalar_t rollMin = std::numeric_limits<scalar_t>::max();
    scalar_t rollMax = std::numeric_limits<scalar_t>::lowest();
    scalar_t baseHeightMin = std::numeric_limits<scalar_t>::max();
    feet_array_t<size_t> stanceWithoutContact{};  // ticks the reference stands on a foot the sensors see in the air
    feet_array_t<size_t> swingWithContact{};      // ticks the reference swings a foot the sensors see on the ground
    scalar_t centroidalResidualSum = 0.0;         // sum over ticks of |achieved - requested CoM acceleration| in xy
    size_t numTicks = 0;
    feet_array_t<size_t> frictionSaturated{};     // ticks a reference stance foot's WBC force sits on the friction pyramid
    scalar_t durationMin = std::numeric_limits<scalar_t>::max();     // phase duration u(DT) over the policies applied in the phase
    scalar_t durationMax = std::numeric_limits<scalar_t>::lowest();
    vector_t firstFootholds;                      // footholds of the first policy applied in the phase
    scalar_t footholdDrift = 0.0;                 // largest move of any foothold coordinate since then [m]
    bool fallback = false;                        // a capture-point stop ran in the phase
    size_t policyUpdates = 0;                     // new MPC policies loaded during the phase
  };
  PhaseDiagnostics optiPessiPhaseDiagnostics_{};

  // Visualization
  std::shared_ptr<LeggedRobotVisualizer> robotVisualizer_;
  std::shared_ptr<LeggedSelfCollisionVisualization> selfCollisionVisualization_;

  // ROS 2 publishers and subscribers
  rclcpp::Publisher<ocs2_msgs::msg::MpcObservation>::SharedPtr optiPessiObservationPublisher_;
  rclcpp::Publisher<ocs2_msgs::msg::MpcObservation>::SharedPtr leggedObservationPublisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr optiPessiPlanPublisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr optiPessiTrajectoryPublisher_;
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr contactSub_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr optiPessiGoalSub_;
  rclcpp::Subscription<legged_controllers::msg::ObstacleArray>::SharedPtr optiPessiObstacleSub_;

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
