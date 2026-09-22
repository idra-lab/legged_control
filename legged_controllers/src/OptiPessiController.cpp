//
// Opti-Pessi controller: LIP phase clock, reference synthesis and MPC hand-off. See OptiPessiController.h
// for the thread layout and the once-per-phase loop closure.
//

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "legged_controllers/OptiPessiController.h"

#include "legged_controllers/HardwareCommandWriter.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_core/thread_support/SetThreadPriority.h>
#include <ocs2_legged_robot/foot_planner/CubicSpline.h>
#include <ocs2_legged_robot/foot_planner/SplineCpg.h>
#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>
#include <ocs2_legged_robot_ros/gait/GaitReceiver.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_ros_interfaces/visualization/VisualizationHelpers.h>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <ocs2_sqp/SqpMpc.h>
#include <ocs2_ipm/IpmMpc.h>

#include <angles/angles.h>
#include <legged_estimation/FromTopiceEstimate.h>
#include <legged_estimation/LinearKalmanFilter.h>
#include <legged_wbc/OptiPessiWbc.h>
#include <pluginlib/class_list_macros.hpp>

#include <opti_pessi_interface/LipKinematics.h>
#include <opti_pessi_interface/OptiPessiInterface.h>
#include <opti_pessi_interface/OptiPessiMpc.h>
#include <opti_pessi_interface/initialization/OptiPessiInitializer.h>

#include <algorithm>
#include <cstdio>
#include <numeric>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

using namespace std;

namespace legged {

// Obstacle.msg type codes are opti_pessi::ObstacleType values.
static_assert(::legged_controllers::msg::Obstacle::HUMAN == static_cast<uint8_t>(opti_pessi::ObstacleType::Human));
static_assert(::legged_controllers::msg::Obstacle::CAR == static_cast<uint8_t>(opti_pessi::ObstacleType::Car));
static_assert(opti_pessi::kNumObstacleTypes == 2);

controller_interface::CallbackReturn OptiPessiController::on_init() {
  auto node = this->get_node();

  // Declare parameters safely
  if (!node->has_parameter("urdfFile")) {
    node->declare_parameter<std::string>("urdfFile", "");
  }
  if (!node->has_parameter("taskFile")) {
    node->declare_parameter<std::string>("taskFile", "");
  }
  if (!node->has_parameter("referenceFile")) {
    node->declare_parameter<std::string>("referenceFile", "");
  }
  if (!node->has_parameter("optipessiFile")) {
    node->declare_parameter<std::string>("optipessiFile", "");
  }
  if (!node->has_parameter("scenarioFile")) {
    node->declare_parameter<std::string>("scenarioFile", "");
  }
  if (!node->has_parameter("libraryFolder")) {
    node->declare_parameter<std::string>("libraryFolder", "");
  }
  if (!node->has_parameter("recompile")) {
    node->declare_parameter<bool>("recompile", false);
  }
  if (!node->has_parameter("backend")) {
    node->declare_parameter<std::string>("backend", "Ipm");
  }
  if (!node->has_parameter("obstacleDetour")) {
    node->declare_parameter<bool>("obstacleDetour", true);
  }

  std::string urdfFile = node->get_parameter("urdfFile").as_string();
  std::string taskFile = node->get_parameter("taskFile").as_string();
  std::string referenceFile = node->get_parameter("referenceFile").as_string();
  std::string optipessiFile = node->get_parameter("optipessiFile").as_string();
  std::string scenarioFile = node->get_parameter("scenarioFile").as_string();
  std::string libraryFolder = node->get_parameter("libraryFolder").as_string();
  bool recompile = node->get_parameter("recompile").as_bool();
  std::string backendStr = node->get_parameter("backend").as_string();
  optiPessiDetourEnabled_ = node->get_parameter("obstacleDetour").as_bool();
  opti_pessi::SolverBackend backend = opti_pessi::SolverBackend::Ipm;
  if (backendStr == "Ipm") {
    backend = opti_pessi::SolverBackend::Ipm;
  } else if (backendStr == "Sqp") {
    backend = opti_pessi::SolverBackend::Sqp;
  }

  bool verbose = true;
  loadData::loadCppDataType(taskFile, "legged_robot_interface.verbose", verbose);

  // Helper node for OCS2 ROS 2 communication
  ros2_node_ = std::make_shared<rclcpp::Node>("opti_pessi_controller_ros2_node");
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(ros2_node_);
  spin_thread_ = std::thread([this]() {
    executor_->spin();
  });

  // LeggedInterface first: the Opti-Pessi model takes its mass and inertia from the URDF.
  setupLeggedInterface(taskFile, urdfFile, referenceFile, verbose);
  setupOptiPessiInterface(optipessiFile, scenarioFile, libraryFolder, recompile, backend);
  // setupLeggedMpc();
  setupOptiPessiMpc();
  // setupLeggedMrt();
  setupOptiPessiMrt();

  // Visualization
  CentroidalModelPinocchioMapping pinocchioMapping(leggedInterface_->getCentroidalModelInfo());
  eeKinematicsPtr_ = std::make_shared<PinocchioEndEffectorKinematics>(leggedInterface_->getPinocchioInterface(), pinocchioMapping,
                                                                      leggedInterface_->modelSettings().contactNames3DoF);
  robotVisualizer_ = std::make_shared<LeggedRobotVisualizer>(leggedInterface_->getPinocchioInterface(),
                                                             leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_, ros2_node_);
  selfCollisionVisualization_ = std::make_shared<LeggedSelfCollisionVisualization>(leggedInterface_->getPinocchioInterface(),
                                                                         leggedInterface_->getGeometryInterface(), pinocchioMapping);
  executor_->add_node(selfCollisionVisualization_);

  // State estimation
  setupStateEstimate(taskFile, verbose);

  // Whole body control
  wbc_ = std::make_shared<OptiPessiWbc>(leggedInterface_->getPinocchioInterface(), leggedInterface_->getCentroidalModelInfo(),
                                        *eeKinematicsPtr_);
  wbc_->loadTasksSetting(taskFile, verbose);

  // Safety Checker
  safetyChecker_ = std::make_shared<SafetyChecker>(leggedInterface_->getCentroidalModelInfo());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn OptiPessiController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration OptiPessiController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  std::vector<std::string> joint_names{"LF_HAA", "LF_HFE", "LF_KFE", "LH_HAA", "LH_HFE", "LH_KFE",
                                       "RF_HAA", "RF_HFE", "RF_KFE", "RH_HAA", "RH_HFE", "RH_KFE"};
  for (const auto& joint : joint_names) {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
    config.names.push_back(joint + "/kp");
    config.names.push_back(joint + "/kd");
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
  }
  return config;
}

controller_interface::InterfaceConfiguration OptiPessiController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  std::vector<std::string> joint_names{"LF_HAA", "LF_HFE", "LF_KFE", "LH_HAA", "LH_HFE", "LH_KFE",
                                       "RF_HAA", "RF_HFE", "RF_KFE", "RH_HAA", "RH_HFE", "RH_KFE"};
  for (const auto& joint : joint_names) {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
  }
  
  // IMU sensor
  std::string imu_name = "base_imu";
  config.names.push_back(imu_name + "/orientation.x");
  config.names.push_back(imu_name + "/orientation.y");
  config.names.push_back(imu_name + "/orientation.z");
  config.names.push_back(imu_name + "/orientation.w");
  config.names.push_back(imu_name + "/angular_velocity.x");
  config.names.push_back(imu_name + "/angular_velocity.y");
  config.names.push_back(imu_name + "/angular_velocity.z");
  config.names.push_back(imu_name + "/linear_acceleration.x");
  config.names.push_back(imu_name + "/linear_acceleration.y");
  config.names.push_back(imu_name + "/linear_acceleration.z");

  // Contact sensors
  std::vector<std::string> contact_names{"LF_FOOT", "LH_FOOT", "RF_FOOT", "RH_FOOT"};
  for (const auto& contact : contact_names) {
    config.names.push_back(contact + "/contact");
  }

  return config;
}

controller_interface::CallbackReturn OptiPessiController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
  auto node = this->get_node();
  
  // Clear and resize handles
  hybridJointHandles_.clear();
  contactHandles_.clear();

  std::vector<std::string> joint_names{"LF_HAA", "LF_HFE", "LF_KFE", "LH_HAA", "LH_HFE", "LH_KFE",
                                       "RF_HAA", "RF_HFE", "RF_KFE", "RH_HAA", "RH_HFE", "RH_KFE"};
                                       
  for (const auto& joint : joint_names) {
    HybridJointHandle joint_handle;
    for (auto& state_interface : state_interfaces_) {
      if (state_interface.get_prefix_name() == joint) {
        if (state_interface.get_interface_name() == hardware_interface::HW_IF_POSITION) {
          joint_handle.positionState = &state_interface;
        } else if (state_interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY) {
          joint_handle.velocityState = &state_interface;
        } else if (state_interface.get_interface_name() == hardware_interface::HW_IF_EFFORT) {
          joint_handle.effortState = &state_interface;
        }
      }
    }
    for (auto& command_interface : command_interfaces_) {
      if (command_interface.get_prefix_name() == joint) {
        if (command_interface.get_interface_name() == hardware_interface::HW_IF_POSITION) {
          joint_handle.positionCmd = &command_interface;
        } else if (command_interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY) {
          joint_handle.velocityCmd = &command_interface;
        } else if (command_interface.get_interface_name() == "kp") {
          joint_handle.kpCmd = &command_interface;
        } else if (command_interface.get_interface_name() == "kd") {
          joint_handle.kdCmd = &command_interface;
        } else if (command_interface.get_interface_name() == hardware_interface::HW_IF_EFFORT) {
          joint_handle.effortCmd = &command_interface;
        }
      }
    }
    if (!joint_handle.positionState || !joint_handle.velocityState || !joint_handle.effortState ||
        !joint_handle.positionCmd || !joint_handle.velocityCmd || !joint_handle.kpCmd || !joint_handle.kdCmd || !joint_handle.effortCmd) {
      RCLCPP_ERROR(node->get_logger(), "Failed to bind all interfaces for joint %s", joint.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    hybridJointHandles_.push_back(joint_handle);
  }

  // Bind IMU interfaces
  std::string imu_name = "base_imu";
  imuSensorHandle_.orientation.clear();
  imuSensorHandle_.angularVelocity.clear();
  imuSensorHandle_.linearAcceleration.clear();
  imuSensorHandle_.orientation.resize(4, nullptr);
  imuSensorHandle_.angularVelocity.resize(3, nullptr);
  imuSensorHandle_.linearAcceleration.resize(3, nullptr);

  for (auto& state_interface : state_interfaces_) {
    if (state_interface.get_prefix_name() == imu_name) {
      if (state_interface.get_interface_name() == "orientation.x") imuSensorHandle_.orientation[0] = &state_interface;
      else if (state_interface.get_interface_name() == "orientation.y") imuSensorHandle_.orientation[1] = &state_interface;
      else if (state_interface.get_interface_name() == "orientation.z") imuSensorHandle_.orientation[2] = &state_interface;
      else if (state_interface.get_interface_name() == "orientation.w") imuSensorHandle_.orientation[3] = &state_interface;
      else if (state_interface.get_interface_name() == "angular_velocity.x") imuSensorHandle_.angularVelocity[0] = &state_interface;
      else if (state_interface.get_interface_name() == "angular_velocity.y") imuSensorHandle_.angularVelocity[1] = &state_interface;
      else if (state_interface.get_interface_name() == "angular_velocity.z") imuSensorHandle_.angularVelocity[2] = &state_interface;
      else if (state_interface.get_interface_name() == "linear_acceleration.x") imuSensorHandle_.linearAcceleration[0] = &state_interface;
      else if (state_interface.get_interface_name() == "linear_acceleration.y") imuSensorHandle_.linearAcceleration[1] = &state_interface;
      else if (state_interface.get_interface_name() == "linear_acceleration.z") imuSensorHandle_.linearAcceleration[2] = &state_interface;
    }
  }
  
  for (size_t i = 0; i < 4; ++i) {
    if (!imuSensorHandle_.orientation[i]) {
      RCLCPP_ERROR(node->get_logger(), "Failed to bind IMU orientation interface %zu", i);
      return controller_interface::CallbackReturn::ERROR;
    }
  }
  for (size_t i = 0; i < 3; ++i) {
    if (!imuSensorHandle_.angularVelocity[i] || !imuSensorHandle_.linearAcceleration[i]) {
      RCLCPP_ERROR(node->get_logger(), "Failed to bind IMU angular velocity/linear acceleration interface %zu", i);
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  // Bind Contact sensor interfaces
  bool contacts_exported = false;
  std::vector<std::string> contact_names{"LF_FOOT", "LH_FOOT", "RF_FOOT", "RH_FOOT"};
  for (const auto& contact : contact_names) {
    ContactSensorHandle contact_handle;
    for (auto& state_interface : state_interfaces_) {
      if (state_interface.get_prefix_name() == contact && state_interface.get_interface_name() == "contact") {
        contact_handle.contactState = &state_interface;
        contacts_exported = true;
        break;
      }
    }
    contactHandles_.push_back(contact_handle);
  }

  if (!contacts_exported) {
    RCLCPP_INFO(node->get_logger(), "Contact state interfaces not found. Subscribing to /contact topic...");
    auto contactCallback = [this](const std_msgs::msg::Int16MultiArray::SharedPtr msg) {
      if (msg->data.size() >= 4) {
        for (size_t i = 0; i < 4; ++i) {
          topicContacts_[i] = msg->data[i] > 40;
        }
      }
    };
    contactSub_ = ros2_node_->create_subscription<std_msgs::msg::Int16MultiArray>("/contact", 10, contactCallback);
  }

  // Initial state observation
  currentObservation_.state.setZero(leggedInterface_->getCentroidalModelInfo().stateDim);
  updateStateEstimation(node->get_clock()->now(), rclcpp::Duration::from_seconds(0.002));
  currentObservation_.input.setZero(leggedInterface_->getCentroidalModelInfo().inputDim);
  currentObservation_.mode = ModeNumber::STANCE;

  // Stand up before walking: the robot usually lies where Gazebo dropped it, with no controller holding it.
  // The WBC holds all four feet and ramps the CoM height up to comHeight (see standUp()); the MPC starts once
  // the robot stands, from the LIP state measured then.
  const vector_t robotState = measureLipState(0, optiPessiLiftoffPositions_);
  const vector3_t com = measureCenterOfMass();
  RCLCPP_INFO(node->get_logger(), "[OptiPessi] measured initial state: c=(%.3f, %.3f) theta=%.3f v=(%.3f, %.3f) dtheta=%.3f "
              "p0=(%.3f, %.3f) p1=(%.3f, %.3f)",
              robotState(0), robotState(1), robotState(2), robotState(3), robotState(4), robotState(5), robotState(6),
              robotState(7), robotState(8), robotState(9));
  // measuredRbdState_ = [yaw, pitch, roll, base position, ...]; feet indexed by opti_pessi::Foot (FL, FR, RL, RR).
  RCLCPP_INFO(node->get_logger(), "[OptiPessi] measured initial posture: baseZ=%.3f comZ=%.3f pitch=%.3f roll=%.3f "
              "feetZ=(%.3f, %.3f, %.3f, %.3f)",
              measuredRbdState_(5), com.z(), measuredRbdState_(1), measuredRbdState_(2), optiPessiLiftoffPositions_[0].z(),
              optiPessiLiftoffPositions_[1].z(), optiPessiLiftoffPositions_[2].z(), optiPessiLiftoffPositions_[3].z());

  for (size_t i = 0; i < optiPessiFootReferences_.size(); ++i) {
    optiPessiFootReferences_[i].position = optiPessiLiftoffPositions_[i];
  }
  holdStance(vector3_t(robotState(opti_pessi::RobotX::CX), robotState(opti_pessi::RobotX::CY), com.z()),
             robotState(opti_pessi::RobotX::TH));
  optiPessiStandStartHeight_ = com.z();
  optiPessiStandElapsed_ = 0.0;
  optiPessiStandingUp_ = true;
  optiPessiGoalReached_ = false;
  optiPessiStopping_ = false;
  optiPessiFailedSolveStops_ = 0;
  optiPessiStandFootprintValid_ = false;
  optiPessiRecoveryStep_ = 0;
  optiPessiPhaseDiagnostics_ = PhaseDiagnostics();
  optiPessiQpFailuresAtPhaseStart_ = wbc_->getNumQpFailures();
  mpcRunning_ = false;
  // A policy left from an earlier activation must not count as the first policy of this one, nor warm-start its solves.
  optiPessiMrtInterface_->reset();
  optiPessiMpcResetRequested_ = true;
  optiPessiPlan_ = AcceptedPlan();
  RCLCPP_INFO(node->get_logger(), "[OptiPessi] standing up: CoM height %.3f -> %.3f in %.1f s, then %.1f s settling",
              optiPessiStandStartHeight_, optiPessiInterface_->modelParameters().comHeight, optiPessiStandUpDuration_,
              optiPessiStandSettleDuration_);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn OptiPessiController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
  mpcRunning_ = false;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type OptiPessiController::update(const rclcpp::Time& time, const rclcpp::Duration& period) {
  // State Estimate
  updateStateEstimation(time, period);

  // Visualization of the measured robot (odom -> base TF, joint states, feet) with the Opti-Pessi
  // contact schedule and reference forces as its mode and input, i.e. LeggedController's current state.
  // Foot indices of opti_pessi::Foot match contactNames3DoF (LF, RF, LH, RH). There is no centroidal
  // plan, so the policy and command are empty: publishOptiPessiTrajectories() draws the plan instead.
  SystemObservation visualizedObservation = currentObservation_;
  contact_flag_t referenceContacts{};
  for (size_t i = 0; i < optiPessiFootReferences_.size(); ++i) {
    referenceContacts[i] = optiPessiFootReferences_[i].contact;
    visualizedObservation.input.segment<3>(3 * i) = optiPessiFootReferences_[i].force;
  }
  visualizedObservation.mode = stanceLeg2ModeNumber(referenceContacts);
  robotVisualizer_->update(visualizedObservation, PrimalSolution(), CommandData());
  selfCollisionVisualization_->update(currentObservation_);

  // Load the latest MPC policy
  // Plans come from the MPC itself (see OptiPessiMpc): the MRT policy is the solver's last iterate, for drawing only.
  const auto& optiPessiMpc = static_cast<const opti_pessi::OptiPessiMpc&>(*optiPessiMpc_);
  if (optiPessiMpc.getPlanSequence() != optiPessiPlanSequence_) {
    const opti_pessi::OptiPessiMpc::Plan plan = optiPessiMpc.getLatestPlan();
    optiPessiPlanSequence_ = plan.sequence;
    ++optiPessiPhaseDiagnostics_.policyUpdates;
    handleMpcPlan(plan);
  }
  if (optiPessiMrtInterface_->updatePolicy()) {
    publishOptiPessiPlan();
    publishOptiPessiTrajectories();
  }

  if (optiPessiStandingUp_) {
    standUp(period);
  } else {
    advanceOptiPessiPhase(time, period);
  }

  // Whole body control every tick, also while standing up or while the LIP clock is held.
  if (!updateWholeBodyControl(period)) {
    return controller_interface::return_type::ERROR;
  }
  return controller_interface::return_type::OK;
}

void OptiPessiController::advanceOptiPessiPhase(const rclcpp::Time& time, const rclcpp::Duration& period) {
  // Standing at the goal: holdStance() has set the references once. A new goal away from the robot restarts the walk
  // from stance (standUp() measures phase 0 again).
  if (optiPessiGoalReached_) {
    vector_t goal;
    size_t goalSequence = 0;
    if (getOptiPessiGoal(goal, goalSequence) && goalSequence != optiPessiReachedGoalSequence_) {
      optiPessiReachedGoalSequence_ = goalSequence;
      const scalar_t distance = (optiPessiRobotState_.head(2) - goal).norm();
      if (distance >= optiPessiInterface_->modelParameters().goalTolerance) {
        RCLCPP_INFO(this->get_node()->get_logger(), "[OptiPessi] new goal (%.3f, %.3f) %.3f m away, walking again", goal(0), goal(1),
                    distance);
        optiPessiGoalReached_ = false;
        restartFromStance();
      }
    }
    return;
  }

  // No plan yet (the MPC starts when standUp() ends, and restartFromStance() clears the plan): keep standing. A failed
  // solve has no step to wait for: the recovery starts from the robot standing here.
  if (!optiPessiPlan_.valid || optiPessiPlan_.phase > optiPessiPhase_) {
    if (optiPessiStopping_) {
      const vector_t robotState = measureLipState(optiPessiPhase_, optiPessiLiftoffPositions_);
      {
        std::lock_guard<std::mutex> lock(optiPessiPhaseMutex_);
        optiPessiRobotState_ = robotState;
      }
      optiPessiPhaseElapsed_ = 0.0;
      optiPessiPhaseDuration_ = 0.0;
      continueRecovery(robotState);
      return;
    }
    optiPessiWaitTime_ += period.seconds();
    return;
  }

  // Every solve starts at knot 0 (see opti_pessi_interface/definitions.h). Until the MPC thread delivers this phase's
  // own plan, the latest plan runs shifted by the phases completed since it was solved (its last knot past the
  // horizon), from the state measured at the start of this phase. Only accepted solves are stored (handleMpcPlan()), and
  // the plan is applied as it comes, with no checks except on its duration (below). The LIP clock never waits for a
  // solve. The input lasts u(DT) seconds.
  const auto& params = optiPessiInterface_->modelParameters();
  const size_t offset = optiPessiPhase_ - optiPessiPlan_.phase;
  const size_t lastKnot = optiPessiPlan_.inputs.size() - 1;
  const vector_t robotState = offset == 0 ? optiPessiPlan_.startState : optiPessiRobotState_;
  vector_t appliedInput = optiPessiPlan_.inputs[std::min(offset, lastKnot)];
  if (offset > 0) {
    optiPessiWaitTime_ += period.seconds();
  }

  // A plan arriving mid-phase may bring touchdown forward, but not closer than optiPessiMinLandingTime_ from now (or
  // than the previous touchdown, if that was sooner). Without this, a late plan with a shorter dt than the time already
  // spent ends the phase at once and the swing feet become the stance pair in mid-air. The floor never moves touchdown
  // later than the previous tick had it, so re-solves cannot stretch a phase forever. At phase start
  // optiPessiPhaseDuration_ is 0 and the floor is 0. The stretched dt is the one applied everywhere below.
  const scalar_t plannedDuration = appliedInput(opti_pessi::RobotU::DT);
  const scalar_t landingFloor =
      optiPessiPhaseElapsed_ + std::min(optiPessiPhaseDuration_ - optiPessiPhaseElapsed_, optiPessiMinLandingTime_);
  const scalar_t phaseDuration = std::max(plannedDuration, landingFloor);
  optiPessiPhaseDuration_ = phaseDuration;
  appliedInput(opti_pessi::RobotU::DT) = phaseDuration;

  // Plan stability over the phase: the MPC keeps re-solving it, and each new policy can move dt and the footholds.
  {
    PhaseDiagnostics& diagnostics = optiPessiPhaseDiagnostics_;
    diagnostics.durationMin = std::min(diagnostics.durationMin, plannedDuration);
    diagnostics.durationMax = std::max(diagnostics.durationMax, plannedDuration);
    diagnostics.durationStretch = std::max(diagnostics.durationStretch, phaseDuration - plannedDuration);
    const vector_t footholds = appliedInput.head(4);
    if (diagnostics.firstFootholds.size() == 0) {
      diagnostics.firstFootholds = footholds;
    }
    diagnostics.footholdDrift = std::max(diagnostics.footholdDrift, (footholds - diagnostics.firstFootholds).cwiseAbs().maxCoeff());
  }

  // The next knot is the stance the swing feet land in, so its forces are their touchdown forces.
  const vector_t nextState = opti_pessi::lipMapScalar(robotState, appliedInput, params.omega(), params.mass, params.inertia);
  const vector_t nextInput = optiPessiPlan_.inputs[std::min(offset + 1, lastKnot)];

  optiPessiPhaseElapsed_ += period.seconds();
  updateFootReferences(robotState, appliedInput, nextState, nextInput, optiPessiPhaseElapsed_);
  updateComReference(robotState, appliedInput, optiPessiPhaseElapsed_);
  if (optiPessiRecoveryStep_ > 0) {
    // Recovery steps hold the heading instead of following the yaw torque of their hand-set force split.
    optiPessiComReference_.yaw = optiPessiRecoveryYaw_;
    optiPessiComReference_.yawRate = 0.0;
    optiPessiComReference_.yawAcceleration = 0.0;
  }
  if (optiPessiPhaseElapsed_ < phaseDuration) {
    return;
  }

  // Phase over: measure the robot for the next phase's stance pair and hand it to the MPC thread.
  const vector_t successorState = measureLipState(optiPessiPhase_ + 1, optiPessiLiftoffPositions_);

  // Diagnostics of the phase just ended, in sim time: how long the LIP clock waited for its policy, how far the
  // measured robot ended from the LIP prediction (plan realizable?) and from the WBC references (tracking?), and
  // how many WBC QPs failed during it.
  {
    using opti_pessi::RobotX;
    const vector_t& predicted = nextState;
    const vector_t lipError = predicted - successorState;
    const size_t qpFailures = wbc_->getNumQpFailures();
    RCLCPP_INFO(this->get_node()->get_logger(),
                "[OptiPessi] phase %zu diag: t=%.3f shifted=%.3f solve=%.1fms | LIP-measured c=(%.3f, %.3f) dc=(%.3f, %.3f) th=%.3f dth=%.3f "
                "p0=(%.3f, %.3f) p1=(%.3f, %.3f) | ref-measured com=(%.3f, %.3f) yaw=%.3f yawRate=%.3f | measured |dc|=%.3f dth=%.3f | "
                "WBC QP failures=%zu",
                optiPessiPhase_, time.seconds(), optiPessiWaitTime_, optiPessiMpcTimer_.getLastIntervalInMilliseconds(), lipError(RobotX::CX),
                lipError(RobotX::CY), lipError(RobotX::DCX), lipError(RobotX::DCY), lipError(RobotX::TH), lipError(RobotX::DTH),
                lipError(RobotX::P0X), lipError(RobotX::P0Y), lipError(RobotX::P1X), lipError(RobotX::P1Y),
                optiPessiComReference_.position.x() - successorState(RobotX::CX),
                optiPessiComReference_.position.y() - successorState(RobotX::CY), optiPessiComReference_.yaw - successorState(RobotX::TH),
                optiPessiComReference_.yawRate - successorState(RobotX::DTH), successorState.segment(RobotX::DCX, 2).norm(),
                successorState(RobotX::DTH), qpFailures - optiPessiQpFailuresAtPhaseStart_);
    optiPessiQpFailuresAtPhaseStart_ = qpFailures;
    optiPessiWaitTime_ = 0.0;

    // Foot order of the contact counters: LF, RF, LH, RH (contact_flag_t).
    const PhaseDiagnostics& d = optiPessiPhaseDiagnostics_;
    RCLCPP_INFO(this->get_node()->get_logger(),
                "[OptiPessi] phase %zu posture: pitch=[%.3f, %.3f] (negative = nose up) roll=[%.3f, %.3f] baseZ min=%.3f | ticks "
                "stance-without-contact LF/RF/LH/RH=%zu/%zu/%zu/%zu, swing-with-contact=%zu/%zu/%zu/%zu",
                optiPessiPhase_, d.pitchMin, d.pitchMax, d.rollMin, d.rollMax, d.baseHeightMin, d.stanceWithoutContact[0],
                d.stanceWithoutContact[1], d.stanceWithoutContact[2], d.stanceWithoutContact[3], d.swingWithContact[0],
                d.swingWithContact[1], d.swingWithContact[2], d.swingWithContact[3]);
    RCLCPP_INFO(this->get_node()->get_logger(),
                "[OptiPessi] phase %zu wbc: mean |achieved - requested CoM acceleration| xy=%.3f m/s^2 | ticks at friction pyramid "
                "(mu=%.2f) LF/RF/LH/RH=%zu/%zu/%zu/%zu of %zu",
                optiPessiPhase_, d.numTicks > 0 ? d.centroidalResidualSum / static_cast<scalar_t>(d.numTicks) : 0.0,
                wbc_->getFrictionCoefficient(), d.frictionSaturated[0], d.frictionSaturated[1], d.frictionSaturated[2],
                d.frictionSaturated[3], d.numTicks);
    {
      // The swing pair of the phase just ended, at the moment it becomes the stance pair: height above its touchdown
      // reference and what the contact sensors say, plus how much the plan moved during the swing.
      const char* const kFootNames[] = {"LF", "RF", "LH", "RH"};
      const contact_flag_t sensorFlags = modeNumber2StanceLeg(currentObservation_.mode);  // handle order LF, LH, RF, RH
      const contact_flag_t measuredContacts{sensorFlags[0], sensorFlags[2], sensorFlags[1], sensorFlags[3]};
      const auto swing = opti_pessi::gaitPair(static_cast<int>(optiPessiPhase_) + 1);
      const auto s0 = static_cast<size_t>(swing[0]);
      const auto s1 = static_cast<size_t>(swing[1]);
      RCLCPP_INFO(this->get_node()->get_logger(),
                  "[OptiPessi] phase %zu touchdown: %s height=%.3f contact=%d, %s height=%.3f contact=%d | dt seen=[%.3f, %.3f] "
                  "stretched=%.3f foothold drift=%.3f policy updates=%zu",
                  optiPessiPhase_, kFootNames[s0], optiPessiLiftoffPositions_[s0].z() - optiPessiFootReferences_[s0].position.z(),
                  measuredContacts[s0] ? 1 : 0, kFootNames[s1],
                  optiPessiLiftoffPositions_[s1].z() - optiPessiFootReferences_[s1].position.z(), measuredContacts[s1] ? 1 : 0,
                  d.durationMin, d.durationMax, d.durationStretch, d.footholdDrift, d.policyUpdates);
    }
    optiPessiPhaseDiagnostics_ = PhaseDiagnostics();

    // Plan quality: the applied knot as the OCP bounds it (StageInequalityConstraint rows on x_1), and the latest
    // MPC solve. A predicted successor beyond the limits means the applied policy was not a feasible plan.
    using vector2_t = Eigen::Matrix<scalar_t, 2, 1>;
    const vector2_t dcBody = opti_pessi::applyR01(predicted(RobotX::TH), vector2_t(predicted(RobotX::DCX), predicted(RobotX::DCY)));
    const auto solve = std::static_pointer_cast<opti_pessi::OptiPessiMpc>(optiPessiMpc_)->getLastSolveStatistics();
    RCLCPP_INFO(this->get_node()->get_logger(),
                "[OptiPessi] phase %zu plan: predicted dcBody=(%.3f, %.3f) dth=%.3f (limits %.2f, %.2f, %.2f) | latest solve: "
                "gaitOffset=%d warmStart=%s iterations=%zu cost=%.3e source=%s pessiScale=%.2f trustworthy=%d dynRes=%.3e "
                "appliedViol=%.3e horizonViol=%.3e | applied %s of phase %zu (shift %zu, %s)",
                optiPessiPhase_, dcBody(0), dcBody(1), predicted(RobotX::DTH), params.dcxMax, params.dcyMax, params.dthetaMax,
                solve.gaitOffset, solve.warmStart, solve.numIterations, solve.performance.cost, solve.source, solve.pessiScale,
                solve.trustworthy ? 1 : 0, solve.dynamicsResidual, solve.appliedViolation, solve.horizonViolation,
                "plan", optiPessiPlan_.phase, offset, optiPessiPlan_.source.c_str());

    // Obstacle clearance at the end of the phase: distance from each obstacle centre to the convex hull of the four hips
    // and the landed feet, measured and as the LIP predicted, against the type radius the OCP keeps out.
    std::vector<ObstacleObservation> obstacles;
    {
      std::lock_guard<std::mutex> lock(optiPessiReferenceMutex_);
      obstacles = optiPessiObstacles_;
    }
    const auto hullClearance = [&](const vector_t& state, const vector2_t& centre) {
      std::array<vector2_t, 6> points{};
      for (size_t f = 0; f < 4; ++f) {
        points[f] = state.head<2>() + opti_pessi::applyR(state(RobotX::TH), opti_pessi::hipOf(params, static_cast<opti_pessi::Foot>(f)));
      }
      points[4] = state.segment<2>(RobotX::P0X);
      points[5] = state.segment<2>(RobotX::P1X);
      scalar_t best = -1e9;
      constexpr int kNumDirections = 64;
      for (int k = 0; k < kNumDirections; ++k) {
        const scalar_t angle = 2.0 * M_PI * static_cast<scalar_t>(k) / kNumDirections;
        const vector2_t a(std::cos(angle), std::sin(angle));
        scalar_t support = -1e9;
        for (const vector2_t& p : points) {
          support = std::max(support, a.dot(p));
        }
        best = std::max(best, a.dot(centre) - support);
      }
      return best;
    };
    for (size_t j = 0; j < obstacles.size(); ++j) {
      const vector2_t centre(obstacles[j].x, obstacles[j].y);
      const opti_pessi::ObstacleTypeModel& model = opti_pessi::obstacleTypeOf(params, obstacles[j].type);
      RCLCPP_INFO(this->get_node()->get_logger(),
                  "[OptiPessi] phase %zu obstacle %zu: centre=(%.3f, %.3f) r=%.2f vmax=%.2f | hull clearance measured=%.3f "
                  "predicted=%.3f (inside keep-out if < r)",
                  optiPessiPhase_, j, centre.x(), centre.y(), model.radius, model.maxSpeed, hullClearance(successorState, centre),
                  hullClearance(predicted, centre));
    }
  }
  {
    std::lock_guard<std::mutex> lock(optiPessiPhaseMutex_);
    optiPessiRobotState_ = successorState;
    ++optiPessiPhase_;
  }
  optiPessiPhaseElapsed_ -= phaseDuration;
  optiPessiPhaseDuration_ = 0.0;
  landSwingFeet();

  RCLCPP_INFO(this->get_node()->get_logger(), "[OptiPessi] phase %zu: dt=%.3f c=(%.3f, %.3f) theta=%.3f v=(%.3f, %.3f)",
              optiPessiPhase_, phaseDuration, successorState(opti_pessi::RobotX::CX), successorState(opti_pessi::RobotX::CY),
              successorState(opti_pessi::RobotX::TH), successorState(opti_pessi::RobotX::DCX), successorState(opti_pessi::RobotX::DCY));

  // Publish the LIP state at each phase start
  SystemObservation lipObservation;
  lipObservation.time = time.seconds();
  lipObservation.state = successorState;
  lipObservation.input = appliedInput;
  lipObservation.mode = optiPessiPhase_;
  optiPessiObservationPublisher_->publish(ros_msg_conversions::createObservationMsg(lipObservation));

  // A solve failed: the step just ended has landed, so all four feet are down. The recovery runs to its end wherever the
  // goal is.
  if (optiPessiStopping_) {
    continueRecovery(successorState);
    return;
  }

  vector_t goal;
  size_t goalSequence = 0;
  if (getOptiPessiGoal(goal, goalSequence) && (successorState.head(2) - goal).norm() < params.goalTolerance) {
    optiPessiGoalReached_ = true;
    optiPessiReachedGoalSequence_ = goalSequence;
    holdStance(optiPessiComReference_.position, optiPessiComReference_.yaw);
    RCLCPP_INFO(this->get_node()->get_logger(), "[OptiPessi] goal reached after %zu phases", optiPessiPhase_);
  }
}

bool OptiPessiController::updateWholeBodyControl(const rclcpp::Duration& period) {
  // opti_pessi::Foot and contactNames3DoF share the foot order (LF, RF, LH, RH).
  WbcReference reference;
  for (size_t i = 0; i < optiPessiFootReferences_.size(); ++i) {
    const FootReference& foot = optiPessiFootReferences_[i];
    reference.contact[i] = foot.contact;
    reference.footPosition[i] = foot.position;
    reference.footVelocity[i] = foot.velocity;
    reference.footForce[i] = foot.force;
  }
  reference.comPosition = optiPessiComReference_.position;
  reference.comVelocity = optiPessiComReference_.velocity;
  reference.comAcceleration = optiPessiComReference_.acceleration;
  reference.yaw = optiPessiComReference_.yaw;
  reference.yawRate = optiPessiComReference_.yawRate;
  reference.yawAcceleration = optiPessiComReference_.yawAcceleration;

  wbcTimer_.startTimer();
  const vector_t x = wbc_->update(reference, measuredRbdState_);
  wbcTimer_.endTimer();

  // Posture and contact statistics of the current phase, logged at its end by advanceOptiPessiPhase().
  // measuredRbdState_ = [yaw, pitch, roll, base position, ...]; the contact flags are the estimator's (sensors).
  PhaseDiagnostics& diagnostics = optiPessiPhaseDiagnostics_;
  diagnostics.pitchMin = std::min(diagnostics.pitchMin, measuredRbdState_(1));
  diagnostics.pitchMax = std::max(diagnostics.pitchMax, measuredRbdState_(1));
  diagnostics.rollMin = std::min(diagnostics.rollMin, measuredRbdState_(2));
  diagnostics.rollMax = std::max(diagnostics.rollMax, measuredRbdState_(2));
  diagnostics.baseHeightMin = std::min(diagnostics.baseHeightMin, measuredRbdState_(5));
  diagnostics.centroidalResidualSum += wbc_->getLastCentroidalResidual().head<2>().norm();
  ++diagnostics.numTicks;
  {
    // x = [qdd, F, tau]: a stance foot is saturated when either tangential component reaches the pyramid edge.
    const size_t forceOffset = leggedInterface_->getCentroidalModelInfo().generalizedCoordinatesNum;
    const scalar_t mu = wbc_->getFrictionCoefficient();
    for (size_t i = 0; i < reference.contact.size(); ++i) {
      const vector3_t force = x.segment<3>(forceOffset + 3 * i);
      if (reference.contact[i] && std::max(std::abs(force.x()), std::abs(force.y())) >= 0.95 * mu * force.z()) {
        ++diagnostics.frictionSaturated[i];
      }
    }
  }
  // The estimator's flags follow the hardware handle order (LF, LH, RF, RH), not contact_flag_t (LF, RF, LH, RH).
  const contact_flag_t sensorFlags = modeNumber2StanceLeg(currentObservation_.mode);
  const contact_flag_t measuredContacts{sensorFlags[0], sensorFlags[2], sensorFlags[1], sensorFlags[3]};
  for (size_t i = 0; i < measuredContacts.size(); ++i) {
    if (reference.contact[i] && !measuredContacts[i]) {
      ++diagnostics.stanceWithoutContact[i];
    } else if (!reference.contact[i] && measuredContacts[i]) {
      ++diagnostics.swingWithContact[i];
    }
  }

  // No centroidal plan to take joint targets from. writeHardwareCommand() uses kp = 0, so only the torque
  // and the velocity target act: the target is the measured joint velocity advanced by one step of the WBC
  // joint accelerations. The position target (unused) is the measured one.
  const auto& info = leggedInterface_->getCentroidalModelInfo();
  const size_t nq = info.generalizedCoordinatesNum;
  vector_t desiredInput = vector_t::Zero(info.inputDim);
  desiredInput.head(3 * info.numThreeDofContacts) = x.segment(nq, 3 * info.numThreeDofContacts);
  desiredInput.tail(info.actuatedDofNum) =
      measuredRbdState_.segment(nq + 6, info.actuatedDofNum) + period.seconds() * x.segment(6, info.actuatedDofNum);

  return writeHardwareCommand(hybridJointHandles_, info, *safetyChecker_, currentObservation_, currentObservation_.state, desiredInput, x,
                              this->get_node()->get_logger(), "[OptiPessi Controller] Safety check failed!");
}

vector_t OptiPessiController::measureLipState(size_t phase, std::array<vector3_t, 4>& footPositions) const {
  using opti_pessi::RobotX;
  const auto& info = leggedInterface_->getCentroidalModelInfo();
  const size_t nq = info.generalizedCoordinatesNum;
  const vector_t& rbd = measuredRbdState_;  // [zyx, basePos, q, angularVelWorld, baseLinearVelWorld, dq]

  const vector3_t zyx = rbd.head<3>();
  const vector3_t angularVelWorld = rbd.segment<3>(nq);

  // Kinematics on a copy, so the shared model data is never written here.
  PinocchioInterface pinocchioInterface = leggedInterface_->getPinocchioInterface();
  const auto& model = pinocchioInterface.getModel();
  auto& data = pinocchioInterface.getData();
  vector_t qPino = vector_t::Zero(nq);
  qPino.head<3>() = rbd.segment<3>(3);
  qPino.segment<3>(3) = zyx;
  qPino.tail(info.actuatedDofNum) = rbd.segment(6, info.actuatedDofNum);
  // Pinocchio velocity as in WbcBase::updateMeasured(): base linear velocity, ZYX Euler rates, joint velocities.
  vector_t vPino = vector_t::Zero(nq);
  vPino.head<3>() = rbd.segment<3>(nq + 3);
  vPino.segment<3>(3) = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<scalar_t>(zyx, angularVelWorld);
  vPino.tail(info.actuatedDofNum) = rbd.segment(nq + 6, info.actuatedDofNum);
  // Whole-body CoM and its velocity (the swinging legs move the CoM relative to the base), then the feet.
  const vector3_t com = pinocchio::centerOfMass(model, data, qPino, vPino);
  const vector3_t comVelocity = data.vcom[0];
  pinocchio::updateFramePlacements(model, data);

  vector_t lipState = vector_t::Zero(RobotX::DIM);
  lipState(RobotX::CX) = com(0);
  lipState(RobotX::CY) = com(1);
  lipState(RobotX::TH) = rbd(0);
  lipState(RobotX::DCX) = comVelocity(0);
  lipState(RobotX::DCY) = comVelocity(1);
  lipState(RobotX::DTH) = vPino(3);

  // Indexed by opti_pessi::Foot (FL, FR, RL, RR).
  const std::array<std::string, 4> footFrames{"LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"};
  for (size_t i = 0; i < footFrames.size(); ++i) {
    footPositions[i] = data.oMf[model.getFrameId(footFrames[i])].translation();
  }
  const auto stance = opti_pessi::gaitPair(static_cast<int>(phase));
  lipState.segment(RobotX::P0X, 2) = footPositions[static_cast<size_t>(stance[0])].head<2>();
  lipState.segment(RobotX::P1X, 2) = footPositions[static_cast<size_t>(stance[1])].head<2>();
  return lipState;
}

std::array<vector3_t, 2> OptiPessiController::computeContactForces(const vector_t& robotState, const vector_t& robotInput,
                                                                   const opti_pessi::OptiPessiModelParameters& params) {
  using opti_pessi::RobotU;
  using opti_pessi::RobotX;
  using vector2_t = Eigen::Matrix<scalar_t, 2, 1>;
  const vector2_t c(robotState(RobotX::CX), robotState(RobotX::CY));
  const vector2_t p0(robotState(RobotX::P0X), robotState(RobotX::P0Y));
  const vector2_t p1(robotState(RobotX::P1X), robotState(RobotX::P1Y));
  const scalar_t alpha = robotInput(RobotU::ALPHA);
  const scalar_t beta = robotInput(RobotU::BETA);
  const scalar_t gamma = robotInput(RobotU::GAMMA);

  // Tangential: the same beta/gamma split of m * ddc as the OCP. Normal: m * g shared so that the CoP sits at alpha.
  vector2_t f0, f1;
  opti_pessi::computeTangentialForces(c, p0, p1, alpha, beta, gamma, params.omega(), params.mass, f0, f1);
  std::array<vector3_t, 2> forces;
  forces[0] << f0, (1.0 - alpha) * params.mass * params.gravity;
  forces[1] << f1, alpha * params.mass * params.gravity;
  return forces;
}

void OptiPessiController::updateFootReferences(const vector_t& robotState, const vector_t& robotInput, const vector_t& nextRobotState,
                                               const vector_t& nextRobotInput, scalar_t time) {
  using opti_pessi::RobotU;
  const auto& params = optiPessiInterface_->modelParameters();
  const scalar_t phaseDuration = robotInput(RobotU::DT);
  time = std::max(0.0, std::min(time, phaseDuration));

  // Stance pair of this phase: stays where it was measured at phase start.
  const auto stance = opti_pessi::gaitPair(static_cast<int>(optiPessiPhase_));
  const auto stanceForces = computeContactForces(robotState, robotInput, params);
  for (size_t k = 0; k < 2; ++k) {
    const auto foot = static_cast<size_t>(stance[k]);
    FootReference& reference = optiPessiFootReferences_[foot];
    reference.contact = true;
    reference.position = optiPessiLiftoffPositions_[foot];
    reference.velocity.setZero();
    reference.force = stanceForces[k];
    reference.touchdownForce = stanceForces[k];
  }

  // Swing pair: the stance pair of the next phase, landing on the footholds of the applied input
  // (flat ground: touchdown at liftoff height).
  const auto swing = opti_pessi::gaitPair(static_cast<int>(optiPessiPhase_) + 1);
  const auto touchdownForces = computeContactForces(nextRobotState, nextRobotInput, params);
  for (size_t k = 0; k < 2; ++k) {
    const auto foot = static_cast<size_t>(swing[k]);
    const vector3_t& liftoff = optiPessiLiftoffPositions_[foot];
    const vector3_t touchdown(robotInput(RobotU::P0X + 2 * static_cast<int>(k)), robotInput(RobotU::P0Y + 2 * static_cast<int>(k)),
                              liftoff.z());

    FootReference& reference = optiPessiFootReferences_[foot];
    reference.contact = false;
    evaluateSwing(liftoff, touchdown, phaseDuration, optiPessiSwingHeight_, time, reference.position, reference.velocity);
    reference.force.setZero();
    reference.touchdownForce = touchdownForces[k];
  }
}

void OptiPessiController::updateComReference(const vector_t& robotState, const vector_t& robotInput, scalar_t time) {
  using opti_pessi::RobotU;
  using opti_pessi::RobotX;
  using vector2_t = Eigen::Matrix<scalar_t, 2, 1>;
  const auto& params = optiPessiInterface_->modelParameters();
  const scalar_t w = params.omega();
  time = std::max(0.0, std::min(time, robotInput(RobotU::DT)));

  const vector2_t c(robotState(RobotX::CX), robotState(RobotX::CY));
  const vector2_t dc(robotState(RobotX::DCX), robotState(RobotX::DCY));
  const vector2_t p0(robotState(RobotX::P0X), robotState(RobotX::P0Y));
  const vector2_t p1(robotState(RobotX::P1X), robotState(RobotX::P1Y));
  const scalar_t alpha = robotInput(RobotU::ALPHA);
  const vector2_t cop = opti_pessi::computeCop(p0, p1, alpha);

  // Closed-form flow of ddc = omega^2 (c - z) with the CoP z held at alpha.
  const scalar_t ch = std::cosh(w * time);
  const scalar_t sh = std::sinh(w * time);
  const vector2_t com = ch * c + (sh / w) * dc + (1.0 - ch) * cop;
  const vector2_t comVelocity = (w * sh) * (c - cop) + ch * dc;
  const vector2_t comAcceleration = (w * w) * (com - cop);
  optiPessiComReference_.position << com, params.comHeight;
  optiPessiComReference_.velocity << comVelocity, 0.0;
  optiPessiComReference_.acceleration << comAcceleration, 0.0;

  // Yaw: lipMap applies the phase-start torque of the tangential forces over the whole phase. This is its
  // continuous form; at phase end it differs from lipMap's forward-Euler yaw by dt^2 tau / (2 I).
  vector2_t f0, f1;
  opti_pessi::computeTangentialForces(c, p0, p1, alpha, robotInput(RobotU::BETA), robotInput(RobotU::GAMMA), w, params.mass, f0, f1);
  const scalar_t yawAcceleration = opti_pessi::yawTorque(c, p0, p1, f0, f1) / params.inertia;
  optiPessiComReference_.yaw = robotState(RobotX::TH) + time * robotState(RobotX::DTH) + 0.5 * time * time * yawAcceleration;
  optiPessiComReference_.yawRate = robotState(RobotX::DTH) + time * yawAcceleration;
  optiPessiComReference_.yawAcceleration = yawAcceleration;
}

void OptiPessiController::standUp(const rclcpp::Duration& period) {
  using opti_pessi::RobotX;
  const auto& params = optiPessiInterface_->modelParameters();
  optiPessiStandElapsed_ += period.seconds();

  // CoM height: cubic from the height measured at activation to comHeight, zero velocity at both ends.
  const scalar_t duration = optiPessiStandUpDuration_;
  const scalar_t s = std::min(optiPessiStandElapsed_ / duration, 1.0);
  const scalar_t rise = params.comHeight - optiPessiStandStartHeight_;
  optiPessiComReference_.position.z() = optiPessiStandStartHeight_ + (3.0 - 2.0 * s) * s * s * rise;
  optiPessiComReference_.velocity.z() = s < 1.0 ? 6.0 * s * (1.0 - s) * rise / duration : 0.0;
  optiPessiComReference_.acceleration.z() = s < 1.0 ? (6.0 - 12.0 * s) * rise / (duration * duration) : 0.0;
  if (optiPessiStandElapsed_ < optiPessiStandUpDuration_ + optiPessiStandSettleDuration_) {
    return;
  }
  // Walking starts once there is a goal to walk to.
  {
    vector_t goal;
    size_t goalSequence = 0;
    if (!getOptiPessiGoal(goal, goalSequence)) {
      RCLCPP_INFO_THROTTLE(this->get_node()->get_logger(), *this->get_node()->get_clock(), 5000,
                           "[OptiPessi] standing, waiting for a goal on /opti_pessi/goal");
      return;
    }
  }

  // Standing: phase 0 starts from the robot measured now, and the MPC thread starts solving it.
  const vector_t robotState = measureLipState(0, optiPessiLiftoffPositions_);
  for (size_t i = 0; i < optiPessiFootReferences_.size(); ++i) {
    optiPessiFootReferences_[i].position = optiPessiLiftoffPositions_[i];
  }
  holdStance(vector3_t(robotState(RobotX::CX), robotState(RobotX::CY), params.comHeight), robotState(RobotX::TH));

  // The stance a failed solve recovers to (continueRecovery()). Only the first stand-up of an activation records it: a
  // later one starts from wherever the walk before it stopped.
  if (!optiPessiStandFootprintValid_) {
    for (size_t i = 0; i < optiPessiStandFootprint_.size(); ++i) {
      optiPessiStandFootprint_[i] =
          opti_pessi::applyR01(robotState(RobotX::TH), optiPessiLiftoffPositions_[i].head<2>() - robotState.segment<2>(RobotX::CX));
    }
    optiPessiStandFootprintValid_ = true;
  }
  {
    std::lock_guard<std::mutex> lock(optiPessiPhaseMutex_);
    optiPessiRobotState_ = robotState;
    optiPessiPhase_ = 0;
  }
  optiPessiPhaseElapsed_ = 0.0;
  optiPessiPhaseDuration_ = 0.0;
  optiPessiPlan_ = AcceptedPlan();  // a plan from before a restart was solved for other feet

  const PhaseDiagnostics& d = optiPessiPhaseDiagnostics_;
  const size_t qpFailures = wbc_->getNumQpFailures();
  RCLCPP_INFO(this->get_node()->get_logger(),
              "[OptiPessi] stood up: comZ=%.3f (reference %.3f) pitch=[%.3f, %.3f] roll=[%.3f, %.3f] WBC QP failures=%zu | phase 0 "
              "starts from c=(%.3f, %.3f) theta=%.3f v=(%.3f, %.3f) dtheta=%.3f",
              measureCenterOfMass().z(), params.comHeight, d.pitchMin, d.pitchMax, d.rollMin, d.rollMax,
              qpFailures - optiPessiQpFailuresAtPhaseStart_, robotState(RobotX::CX), robotState(RobotX::CY), robotState(RobotX::TH),
              robotState(RobotX::DCX), robotState(RobotX::DCY), robotState(RobotX::DTH));
  optiPessiPhaseDiagnostics_ = PhaseDiagnostics();
  optiPessiQpFailuresAtPhaseStart_ = qpFailures;
  optiPessiWaitTime_ = 0.0;

  optiPessiStandingUp_ = false;
  mpcRunning_ = true;
}

void OptiPessiController::handleMpcPlan(const opti_pessi::OptiPessiMpc::Plan& plan) {
  // During the stand-up or a restart the phase is about to be reset: nothing solved now belongs to it. While stopping,
  // the step in progress lands on the plan it started with.
  if (optiPessiStandingUp_ || optiPessiStopping_) {
    return;
  }

  // A plan for this phase must start from the state this phase started from (its start state is an exact copy of the
  // observation), which rejects a solve that was in flight across a restart.
  const bool forThisPhase = plan.phase == optiPessiPhase_ && optiPessiRobotState_.size() == plan.startState.size() &&
                            (plan.startState - optiPessiRobotState_).cwiseAbs().maxCoeff() < 1e-9;
  const bool newerEarlierPhase = plan.phase < optiPessiPhase_ && (!optiPessiPlan_.valid || plan.phase > optiPessiPlan_.phase);
  if (!forThisPhase && !newerEarlierPhase) {
    return;
  }

  // A failed solve is never executed, and the solver is not left to warm-start from it: the robot returns to its stand-up
  // stance (continueRecovery()) and the MPC restarts cold. Standing at the goal nothing is executed, so it changes nothing
  // there.
  if (plan.failed) {
    if (optiPessiGoalReached_) {
      return;
    }
    mpcRunning_ = false;
    optiPessiStopping_ = true;
    ++optiPessiFailedSolveStops_;
    const auto solve = std::static_pointer_cast<opti_pessi::OptiPessiMpc>(optiPessiMpc_)->getLastSolveStatistics();
    RCLCPP_WARN(this->get_node()->get_logger(),
                "[OptiPessi] solve of phase %zu failed (gaitOffset=%d warmStart=%s iterations=%zu dynRes=%.3e appliedViol=%.3e "
                "horizonViol=%.3e): back to the stand-up stance %s (stop %zu)",
                plan.phase, solve.gaitOffset, solve.warmStart, solve.numIterations, solve.dynamicsResidual, solve.appliedViolation,
                solve.horizonViolation, optiPessiPlan_.valid ? "once the current step lands" : "now", optiPessiFailedSolveStops_);
    return;
  }

  optiPessiPlan_.valid = true;
  optiPessiPlan_.phase = plan.phase;
  optiPessiPlan_.startState = plan.startState;
  optiPessiPlan_.inputs = plan.inputs;
  optiPessiPlan_.trustworthy = plan.trustworthy;
  optiPessiPlan_.source = plan.source;
}

void OptiPessiController::restartFromStance() {
  using opti_pessi::RobotX;
  const auto& params = optiPessiInterface_->modelParameters();
  mpcRunning_ = false;
  optiPessiStopping_ = false;
  optiPessiRecoveryStep_ = 0;
  optiPessiPlan_ = AcceptedPlan();
  optiPessiMrtInterface_->reset();
  // The MPC thread resets the solver before its next solve: phase 0 of the new walk starts cold.
  optiPessiMpcResetRequested_ = true;

  // Swinging feet go straight down where they are and the stance pair carries the weight until standUp() puts every
  // foot in contact.
  std::array<vector3_t, 4> feet{};
  const vector_t lipState = measureLipState(optiPessiPhase_, feet);
  size_t numStance = 0;
  for (const FootReference& reference : optiPessiFootReferences_) {
    numStance += reference.contact ? 1 : 0;
  }
  for (size_t i = 0; i < optiPessiFootReferences_.size(); ++i) {
    FootReference& reference = optiPessiFootReferences_[i];
    reference.velocity.setZero();
    if (reference.contact) {
      reference.force << 0.0, 0.0, params.mass * params.gravity / static_cast<scalar_t>(std::max<size_t>(numStance, 1));
    } else {
      reference.position << feet[i].head<2>(), optiPessiLiftoffPositions_[i].z();
      reference.force.setZero();
    }
    reference.touchdownForce = reference.force;
  }
  optiPessiComReference_ = ComReference();
  optiPessiComReference_.position << lipState(RobotX::CX), lipState(RobotX::CY), params.comHeight;
  optiPessiComReference_.yaw = lipState(RobotX::TH);

  // Settling stage only (no height ramp): standUp() re-measures the robot and starts over from phase 0.
  optiPessiStandStartHeight_ = params.comHeight;
  optiPessiStandElapsed_ = optiPessiStandUpDuration_;
  optiPessiStandingUp_ = true;
}

void OptiPessiController::continueRecovery(const vector_t& robotState) {
  using opti_pessi::RobotU;
  using opti_pessi::RobotX;
  using vector2_t = Eigen::Matrix<scalar_t, 2, 1>;
  const auto& params = optiPessiInterface_->modelParameters();
  const auto logger = this->get_node()->get_logger();
  constexpr scalar_t kFootprintTolerance = 0.03;  // [m] largest foot error that counts as standing on the footprint

  if (optiPessiRecoveryStep_ == 0) {
    optiPessiRecoveryYaw_ = robotState(RobotX::TH);
    // Already on the footprint (a failure right after standing up): nothing to step. It is fitted to the feet by its
    // centre, at the current heading.
    vector2_t centre = vector2_t::Zero();
    for (size_t i = 0; i < optiPessiStandFootprint_.size(); ++i) {
      centre += optiPessiLiftoffPositions_[i].head<2>() - opti_pessi::applyR(optiPessiRecoveryYaw_, optiPessiStandFootprint_[i]);
    }
    centre /= static_cast<scalar_t>(optiPessiStandFootprint_.size());
    scalar_t footprintError = 0.0;
    for (size_t i = 0; i < optiPessiStandFootprint_.size(); ++i) {
      const vector2_t foot = centre + opti_pessi::applyR(optiPessiRecoveryYaw_, optiPessiStandFootprint_[i]);
      footprintError = std::max(footprintError, (optiPessiLiftoffPositions_[i].head<2>() - foot).norm());
    }
    if (!optiPessiStandFootprintValid_ || footprintError < kFootprintTolerance) {
      optiPessiRecoveryCom_ = optiPessiStandFootprintValid_ ? centre : vector2_t(robotState.segment<2>(RobotX::CX));
      optiPessiRecoveryStep_ = 2;
    }
  }

  // Both steps have landed: stand over the footprint and restart the MPC cold.
  if (optiPessiRecoveryStep_ == 2) {
    restartFromStance();
    optiPessiComReference_.position.head<2>() = optiPessiRecoveryCom_;
    optiPessiComReference_.yaw = optiPessiRecoveryYaw_;
    RCLCPP_WARN(logger, "[OptiPessi] recovered: standing at (%.3f, %.3f) yaw %.3f in the stand-up stance, restarting the MPC cold",
                optiPessiRecoveryCom_.x(), optiPessiRecoveryCom_.y(), optiPessiRecoveryYaw_);
    return;
  }

  // One diagonal step. On the stance segment the CoP only moves the capture point xi = c + dc / omega along the segment,
  // xi(t) - z = e^(omega t) (xi(0) - z), so it sits where xi projects onto the segment and only the part of xi off the
  // segment grows.
  const scalar_t w = params.omega();
  const scalar_t duration = std::min(std::max(params.dtCost0, params.dtMin), params.dtMax);
  const vector2_t p0 = robotState.segment<2>(RobotX::P0X);
  const vector2_t p1 = robotState.segment<2>(RobotX::P1X);
  const vector2_t capturePoint = robotState.segment<2>(RobotX::CX) + robotState.segment<2>(RobotX::DCX) / w;
  const vector2_t segment = p1 - p0;
  scalar_t alpha = 0.5;
  if (segment.squaredNorm() > 1e-6) {
    alpha = (capturePoint - p0).dot(segment) / segment.squaredNorm();
  }
  alpha = std::min(std::max(alpha, params.alphaReduction), 1.0 - params.alphaReduction);
  const vector2_t cop = opti_pessi::computeCop(p0, p1, alpha);

  // The first step centres the footprint on the capture point it ends with. The second step, standing on the footprint's
  // diagonal through it, holds the capture point there and the CoM comes to rest on it.
  if (optiPessiRecoveryStep_ == 0) {
    optiPessiRecoveryCom_ = cop + std::exp(w * duration) * (capturePoint - cop);
    RCLCPP_WARN(logger, "[OptiPessi] recovery: two steps of %.3f s back to the stand-up stance centred at (%.3f, %.3f) yaw %.3f, "
                "%.3f m from the CoM", duration, optiPessiRecoveryCom_.x(), optiPessiRecoveryCom_.y(), optiPessiRecoveryYaw_,
                (optiPessiRecoveryCom_ - robotState.segment<2>(RobotX::CX)).norm());
  }

  // The swing pair lands on its footprint spots, in the order updateFootReferences() reads them.
  vector_t input = vector_t::Zero(RobotU::DIM);
  const auto swing = opti_pessi::gaitPair(static_cast<int>(optiPessiPhase_) + 1);
  for (size_t k = 0; k < 2; ++k) {
    input.segment<2>(RobotU::P0X + 2 * static_cast<int>(k)) =
        optiPessiRecoveryCom_ + opti_pessi::applyR(optiPessiRecoveryYaw_, optiPessiStandFootprint_[static_cast<size_t>(swing[k])]);
  }
  input(RobotU::ALPHA) = alpha;
  input(RobotU::DT) = duration;
  input(RobotU::BETA) = 0.5;
  input(RobotU::GAMMA) = 0.5;

  optiPessiPlan_ = AcceptedPlan();
  optiPessiPlan_.valid = true;
  optiPessiPlan_.phase = optiPessiPhase_;
  optiPessiPlan_.startState = robotState;
  optiPessiPlan_.inputs = {input};
  optiPessiPlan_.source = "recovery";
  ++optiPessiRecoveryStep_;
}

vector3_t OptiPessiController::measureCenterOfMass() const {
  const auto& info = leggedInterface_->getCentroidalModelInfo();
  const vector_t& rbd = measuredRbdState_;  // [zyx, basePos, q, ...]

  // Kinematics on a copy, so the shared model data is never written here.
  PinocchioInterface pinocchioInterface = leggedInterface_->getPinocchioInterface();
  vector_t qPino = vector_t::Zero(info.generalizedCoordinatesNum);
  qPino.head<3>() = rbd.segment<3>(3);
  qPino.segment<3>(3) = rbd.head<3>();
  qPino.tail(info.actuatedDofNum) = rbd.segment(6, info.actuatedDofNum);
  const vector3_t com = pinocchio::centerOfMass(pinocchioInterface.getModel(), pinocchioInterface.getData(), qPino);
  return com;
}

/** Phase boundary: the swing pair becomes stance with its touchdown forces, the old stance pair is unloaded. */
void OptiPessiController::landSwingFeet() {
  for (FootReference& reference : optiPessiFootReferences_) {
    if (reference.contact) {
      reference.force.setZero();
    } else {
      reference.contact = true;
      reference.velocity.setZero();
      reference.force = reference.touchdownForce;
    }
  }
}

/** All four feet in contact carrying m g / 4, CoM and yaw held: the stand-up and goal-reached posture. */
void OptiPessiController::holdStance(const vector3_t& comPosition, scalar_t yaw) {
  const auto& params = optiPessiInterface_->modelParameters();
  for (FootReference& reference : optiPessiFootReferences_) {
    reference.contact = true;
    reference.velocity.setZero();
    reference.force << 0.0, 0.0, 0.25 * params.mass * params.gravity;
    reference.touchdownForce = reference.force;
  }
  // Built aside and assigned last: callers may pass optiPessiComReference_.position itself, which resetting the member
  // first would zero before it is read (the goal-reached call did, sending the CoM reference to the odom origin).
  ComReference reference;
  reference.position = comPosition;
  reference.yaw = yaw;
  optiPessiComReference_ = reference;
}

void OptiPessiController::publishOptiPessiPlan() {
  using visualization_msgs::msg::Marker;
  const auto& params = optiPessiInterface_->modelParameters();
  const auto stamp = ros2_node_->get_clock()->now();  // same clock as the odom -> base TF of robotVisualizer_

  auto makeMarker = [&](const std::string& ns, int id, int32_t type, float r, float g, float b, float a) {
    Marker marker;
    marker.header.frame_id = "odom";
    marker.header.stamp = stamp;
    marker.ns = ns;
    marker.id = id;
    marker.type = type;
    marker.action = Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;
    return marker;
  };
  auto point = [](scalar_t x, scalar_t y, scalar_t z) {
    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
  };

  visualization_msgs::msg::MarkerArray markers;

  // Clears the obstacles of the previous draw: there may be fewer now.
  Marker clear;
  clear.header.frame_id = "odom";
  clear.header.stamp = stamp;
  clear.action = Marker::DELETEALL;
  markers.markers.push_back(clear);

  // Obstacles and goal as last received (odom), each obstacle at the keep-out radius of its type: humans orange, cars
  // blue. The reference manager is not read here, the MPC thread writes it.
  std::vector<ObstacleObservation> obstacles;
  vector_t goalPosition;
  vector_t detourGoalPosition;
  {
    std::lock_guard<std::mutex> lock(optiPessiReferenceMutex_);
    obstacles = optiPessiObstacles_;
    goalPosition = optiPessiGoal_;
    detourGoalPosition = optiPessiDetourGoal_;
  }
  for (size_t j = 0; j < obstacles.size(); ++j) {
    const bool car = obstacles[j].type == opti_pessi::ObstacleType::Car;
    Marker disk = makeMarker("obstacles", static_cast<int>(j), Marker::CYLINDER, car ? 0.2F : 0.9F, car ? 0.4F : 0.5F, car ? 0.9F : 0.1F,
                             0.6F);
    disk.pose.position = point(obstacles[j].x, obstacles[j].y, 0.25);
    disk.scale.x = disk.scale.y = 2.0 * opti_pessi::obstacleTypeOf(params, obstacles[j].type).radius;
    disk.scale.z = 0.5;
    markers.markers.push_back(disk);
  }

  if (goalPosition.size() == 2) {
    Marker goal = makeMarker("goal", 0, Marker::SPHERE, 1.0F, 0.85F, 0.0F, 1.0F);
    goal.pose.position = point(goalPosition(0), goalPosition(1), 0.05);
    goal.scale.x = goal.scale.y = goal.scale.z = 0.1;
    markers.markers.push_back(goal);
  }

  // The actual local goal tracked by the OCP while an obstacle detour is active (cyan).
  if (detourGoalPosition.size() == 2) {
    Marker detourGoal = makeMarker("detour_goal", 0, Marker::SPHERE, 0.0F, 0.9F, 0.9F, 1.0F);
    detourGoal.pose.position = point(detourGoalPosition(0), detourGoalPosition(1), 0.05);
    detourGoal.scale.x = detourGoal.scale.y = detourGoal.scale.z = 0.1;
    markers.markers.push_back(detourGoal);

    Marker detourLine = makeMarker("detour_goal", 1, Marker::LINE_STRIP, 0.1F, 0.4F, 1.0F, 0.9F);
    detourLine.scale.x = 0.01;
    detourLine.points.push_back(getPointMsg(measureCenterOfMass()));
    detourLine.points.push_back(detourGoal.pose.position);
    markers.markers.push_back(detourLine);
  }

  optiPessiPlanPublisher_->publish(markers);
}

void OptiPessiController::publishOptiPessiTrajectories() {
  using opti_pessi::RobotU;
  using opti_pessi::RobotX;
  using visualization_msgs::msg::Marker;
  using vector2_t = Eigen::Matrix<scalar_t, 2, 1>;
  const auto& params = optiPessiInterface_->modelParameters();
  const PrimalSolution& policy = optiPessiMrtInterface_->getPolicy();
  const size_t phase = optiPessiMrtInterface_->getCommand().mpcInitObservation_.mode;

  // Knot 0 swings from the feet measured at the start of optiPessiPhase_, so a policy solved for
  // another phase has no liftoff positions to start from.
  if (phase != optiPessiPhase_ || policy.stateTrajectory_.size() < 2 || policy.inputTrajectory_.empty()) {
    return;
  }
  const size_t numPhases = std::min(policy.stateTrajectory_.size() - 1, policy.inputTrajectory_.size());
  constexpr size_t samplesPerPhase = 10;
  const scalar_t w = params.omega();

  feet_array_t<std::vector<geometry_msgs::msg::Point>> feetPoints;
  std::vector<geometry_msgs::msg::Point> comPoints;
  std::vector<Marker> comOrientations;
  visualization_msgs::msg::Marker footholds;
  footholds.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  footholds.scale.x = footholds.scale.y = footholds.scale.z = robotVisualizer_->footMarkerDiameter_;
  footholds.ns = "Future footholds";
  footholds.pose.orientation = getOrientationMsg({1., 0., 0., 0.});

  vector_t previousState = opti_pessi::extractRobotState(policy.stateTrajectory_.front());
  for (size_t i = 0; i < numPhases; ++i) {
    const vector_t x = opti_pessi::extractRobotState(policy.stateTrajectory_[i]);
    const vector_t u = opti_pessi::extractRobotInput(policy.inputTrajectory_[i]);
    const scalar_t dt = u(RobotU::DT);
    const auto stance = opti_pessi::gaitPair(static_cast<int>(phase + i));
    const auto swing = opti_pessi::gaitPair(static_cast<int>(phase + i + 1));

    // Flat ground: every foot stays at the height measured at the start of the current phase. The swing
    // pair of knot i stood on the footholds of knot i - 1 (same pair order); at knot 0 it was measured.
    std::array<vector3_t, 2> stancePositions, liftoffs, touchdowns;
    for (size_t k = 0; k < 2; ++k) {
      const int offset = 2 * static_cast<int>(k);
      const scalar_t stanceZ = optiPessiLiftoffPositions_[static_cast<size_t>(stance[k])].z();
      const scalar_t swingZ = optiPessiLiftoffPositions_[static_cast<size_t>(swing[k])].z();
      stancePositions[k] << x(RobotX::P0X + offset), x(RobotX::P0Y + offset), stanceZ;
      if (i == 0) {
        liftoffs[k] = optiPessiLiftoffPositions_[static_cast<size_t>(swing[k])];
      } else {
        liftoffs[k] << previousState(RobotX::P0X + offset), previousState(RobotX::P0Y + offset), swingZ;
      }
      touchdowns[k] << u(RobotU::P0X + offset), u(RobotU::P0Y + offset), swingZ;
      footholds.points.push_back(getPointMsg(touchdowns[k]));
      footholds.colors.push_back(getColor(robotVisualizer_->feetColorMap_[static_cast<size_t>(swing[k])]));
    }

    // Closed-form LIP flow with the CoP held at alpha, as in opti_pessi::lipMap.
    const vector2_t c(x(RobotX::CX), x(RobotX::CY));
    const vector2_t dc(x(RobotX::DCX), x(RobotX::DCY));
    const vector2_t p0(x(RobotX::P0X), x(RobotX::P0Y));
    const vector2_t p1(x(RobotX::P1X), x(RobotX::P1Y));
    const vector2_t cop = opti_pessi::computeCop(p0, p1, u(RobotU::ALPHA));

    // Use the same continuous yaw reference as updateComReference().
    vector2_t f0, f1;
    opti_pessi::computeTangentialForces(c, p0, p1, u(RobotU::ALPHA), u(RobotU::BETA), u(RobotU::GAMMA), w, params.mass, f0, f1);
    const scalar_t yawAcceleration = opti_pessi::yawTorque(c, p0, p1, f0, f1) / params.inertia;

    for (size_t s = (i == 0 ? 0 : 1); s <= samplesPerPhase; ++s) {
      const scalar_t t = dt * static_cast<scalar_t>(s) / static_cast<scalar_t>(samplesPerPhase);
      const scalar_t ch = std::cosh(w * t);
      const scalar_t sh = std::sinh(w * t);
      const vector2_t com = ch * c + (sh / w) * dc + (1.0 - ch) * cop;
      comPoints.push_back(getPointMsg(vector3_t(com(0), com(1), params.comHeight)));

      // Sparse arrows keep the CoM path readable: start, then midpoint and end of each phase.
      if (s % (samplesPerPhase / 2) == 0) {
        const scalar_t yaw = x(RobotX::TH) + t * x(RobotX::DTH) + 0.5 * t * t * yawAcceleration;
        Marker orientation;
        orientation.ns = "CoM Orientation";
        orientation.type = Marker::ARROW;
        orientation.action = Marker::ADD;
        orientation.pose.position = comPoints.back();
        orientation.pose.orientation.z = std::sin(0.5 * yaw);
        orientation.pose.orientation.w = std::cos(0.5 * yaw);
        orientation.scale.x = 0.18;
        orientation.scale.y = 0.025;
        orientation.scale.z = 0.025;
        orientation.color = getColor(Color::red);
        comOrientations.push_back(std::move(orientation));
      }

      for (size_t k = 0; k < 2; ++k) {
        feetPoints[static_cast<size_t>(stance[k])].push_back(getPointMsg(stancePositions[k]));
        vector3_t position, velocity;
        evaluateSwing(liftoffs[k], touchdowns[k], dt, optiPessiSwingHeight_, t, position, velocity);
        feetPoints[static_cast<size_t>(swing[k])].push_back(getPointMsg(position));
      }
    }
    previousState = x;
  }

  visualization_msgs::msg::MarkerArray markerArray;
  // Clear old arrows as well when the horizon becomes shorter.
  Marker clear;
  clear.action = Marker::DELETEALL;
  markerArray.markers.push_back(clear);
  for (size_t i = 0; i < feetPoints.size(); ++i) {
    markerArray.markers.emplace_back(
        getLineMsg(std::move(feetPoints[i]), robotVisualizer_->feetColorMap_[i], robotVisualizer_->trajectoryLineWidth_));
    markerArray.markers.back().ns = "EE Trajectories";
  }
  markerArray.markers.emplace_back(getLineMsg(std::move(comPoints), Color::red, robotVisualizer_->trajectoryLineWidth_));
  markerArray.markers.back().ns = "CoM Trajectory";
  markerArray.markers.push_back(std::move(footholds));
  for (auto& orientation : comOrientations) {
    markerArray.markers.push_back(std::move(orientation));
  }

  // Same clock as the odom -> base TF of robotVisualizer_.
  assignHeader(markerArray.markers.begin(), markerArray.markers.end(), getHeaderMsg("odom", ros2_node_->get_clock()->now()));
  assignIncreasingId(markerArray.markers.begin(), markerArray.markers.end());
  optiPessiTrajectoryPublisher_->publish(markerArray);
}

/** Swing foot at `time`: cubic in x and y with zero end velocities, SplineCpg in z through the apex. */
void OptiPessiController::evaluateSwing(const vector3_t& liftoff, const vector3_t& touchdown, scalar_t duration, scalar_t swingHeight,
                                        scalar_t time, vector3_t& position, vector3_t& velocity) {
  const CubicSpline splineX({0.0, liftoff.x(), 0.0}, {duration, touchdown.x(), 0.0});
  const CubicSpline splineY({0.0, liftoff.y(), 0.0}, {duration, touchdown.y(), 0.0});
  const SplineCpg splineZ({0.0, liftoff.z(), 0.0}, std::max(liftoff.z(), touchdown.z()) + swingHeight, {duration, touchdown.z(), 0.0});
  position << splineX.position(time), splineY.position(time), splineZ.position(time);
  velocity << splineX.velocity(time), splineY.velocity(time), splineZ.velocity(time);
}

void OptiPessiController::pushOptiPessiObservation() {
  size_t phase = 0;
  vector_t robotState;
  {
    std::lock_guard<std::mutex> lock(optiPessiPhaseMutex_);
    phase = optiPessiPhase_;
    robotState = optiPessiRobotState_;
  }

  // Time is always knot 0; the phase index travels in `mode` so update() can tell which phase a
  // policy was solved for. The gait offset is set here, on the thread that runs the solver, so it
  // cannot change under a solve in progress.
  SystemObservation observation;
  observation.time = 0.0;
  observation.state = opti_pessi::packInitialState(robotState);
  observation.input = vector_t::Zero(optiPessiInterface_->inputDim());
  observation.mode = phase;
  optiPessiInterface_->getOptiPessiReferenceManagerPtr()->setGaitOffset(static_cast<int>(phase));
  pushOptiPessiReferences(robotState);
  optiPessiMrtInterface_->setCurrentObservation(observation);
}

void OptiPessiController::pushOptiPessiReferences(const vector_t& robotState) {
  using vector2_t = Eigen::Matrix<scalar_t, 2, 1>;
  const auto& params = optiPessiInterface_->modelParameters();
  vector_t goal;
  std::vector<ObstacleObservation> seen;
  {
    std::lock_guard<std::mutex> lock(optiPessiReferenceMutex_);
    goal = optiPessiGoal_;
    seen = optiPessiObstacles_;
  }
  auto& referenceManager = *optiPessiInterface_->getOptiPessiReferenceManagerPtr();

  // Slots keep the message order, so a slot keeps its obstacle (and the hyperplanes warm-started for it) between solves.
  // With more obstacles than slots, the closest keep-out disks take them.
  const vector2_t com = robotState.head<2>();
  const size_t numSlots = static_cast<size_t>(params.numObstacles());
  std::vector<size_t> selected(seen.size());
  std::iota(selected.begin(), selected.end(), 0);
  if (seen.size() > numSlots) {
    const auto clearance = [&](size_t k) {
      return (vector2_t(seen[k].x, seen[k].y) - com).norm() - opti_pessi::obstacleTypeOf(params, seen[k].type).radius;
    };
    std::partial_sort(selected.begin(), selected.begin() + static_cast<std::ptrdiff_t>(numSlots), selected.end(),
                      [&](size_t a, size_t b) { return clearance(a) < clearance(b); });
    selected.resize(numSlots);
    std::sort(selected.begin(), selected.end());
    RCLCPP_WARN_THROTTLE(ros2_node_->get_logger(), *ros2_node_->get_clock(), 5000,
                         "[OptiPessi] %zu obstacles but %zu OCP slots (scenario obstacles.numObstacles): tracking the closest", seen.size(),
                         numSlots);
  }

  // A free slot holds a point obstacle with no speed bound, parked out of reach of the horizon.
  constexpr scalar_t kParkingDistance = 100.0;  // [m] from the CoM
  matrix_t positions(numSlots, 2);
  vector_t radii = vector_t::Zero(numSlots);
  vector_t maxSpeeds = vector_t::Zero(numSlots);
  for (size_t j = 0; j < numSlots; ++j) {
    if (j < selected.size()) {
      const ObstacleObservation& obstacle = seen[selected[j]];
      const opti_pessi::ObstacleTypeModel& model = opti_pessi::obstacleTypeOf(params, obstacle.type);
      positions.row(j) << obstacle.x, obstacle.y;
      radii(j) = model.radius;
      maxSpeeds(j) = model.maxSpeed;
    } else {
      positions.row(j) << com.x() + kParkingDistance, com.y();
    }
  }
  referenceManager.setObstacles(positions, radii, maxSpeeds);

  // The OCP tracks a detour goal while an obstacle blocks the straight line to the goal: its short horizon never pays
  // for walking around, and stalls at the grown keep-out otherwise. Goal-reached checks keep using the real goal.
  // With the obstacleDetour parameter false, the OCP tracks the goal itself.
  if (goal.size() == 2) {
    if (!optiPessiDetourEnabled_) {
      referenceManager.setGoal(goal);
      return;
    }
    const bool wasActive = optiPessiDetour_->active();
    const vector_t ocpGoal = optiPessiDetour_->detourGoal(robotState, goal, positions, radii, maxSpeeds);
    referenceManager.setGoal(ocpGoal);
    {
      std::lock_guard<std::mutex> lock(optiPessiReferenceMutex_);
      optiPessiDetourGoal_ = optiPessiDetour_->active() ? ocpGoal : vector_t();
    }
    if (optiPessiDetour_->active() != wasActive) {
      RCLCPP_INFO(ros2_node_->get_logger(), "[OptiPessi] obstacle detour %s: OCP goal (%.3f, %.3f), goal (%.3f, %.3f)",
                  optiPessiDetour_->active() ? "on" : "off", ocpGoal(0), ocpGoal(1), goal(0), goal(1));
    }
  }
}

bool OptiPessiController::getOptiPessiGoal(vector_t& goal, size_t& sequence) {
  std::lock_guard<std::mutex> lock(optiPessiReferenceMutex_);
  goal = optiPessiGoal_;
  sequence = optiPessiGoalSequence_;
  return goal.size() == 2;
}

void OptiPessiController::setupOptiPessiReferenceSubscribers() {
  using visualization_msgs::msg::Marker;
  using visualization_msgs::msg::MarkerArray;
  using ObstacleArray = ::legged_controllers::msg::ObstacleArray;

  // No TF lookup: both topics must already be in odom, the frame the LIP state is measured in.
  auto inOdom = [this](const std::string& frame, const char* topic) {
    if (frame.empty() || frame == "odom") {
      return true;
    }
    RCLCPP_WARN_THROTTLE(ros2_node_->get_logger(), *ros2_node_->get_clock(), 5000, "[OptiPessi] %s in frame '%s' ignored: expected odom",
                         topic, frame.c_str());
    return false;
  };

  // Goal: the first marker that is added (ADD, alias MODIFY), at its position. RViz draws the same topic.
  optiPessiGoalSub_ = ros2_node_->create_subscription<MarkerArray>(
      "/opti_pessi/goal", rclcpp::QoS(1), [this, inOdom](const MarkerArray::SharedPtr msg) {
        for (const Marker& marker : msg->markers) {
          if (marker.action != Marker::ADD || !inOdom(marker.header.frame_id, "/opti_pessi/goal")) {
            continue;
          }
          vector_t goal(2);
          goal << marker.pose.position.x, marker.pose.position.y;
          bool changed = false;
          {
            std::lock_guard<std::mutex> lock(optiPessiReferenceMutex_);
            changed = optiPessiGoal_.size() != 2 || !optiPessiGoal_.isApprox(goal);
            if (changed) {
              optiPessiGoal_ = goal;
              ++optiPessiGoalSequence_;
            }
          }
          if (changed) {
            RCLCPP_INFO(ros2_node_->get_logger(), "[OptiPessi] goal (%.3f, %.3f)", goal(0), goal(1));
          }
          return;
        }
      });

  optiPessiObstacleSub_ = ros2_node_->create_subscription<ObstacleArray>(
      "/opti_pessi/obstacles", rclcpp::QoS(1), [this, inOdom](const ObstacleArray::SharedPtr msg) {
        if (!inOdom(msg->header.frame_id, "/opti_pessi/obstacles")) {
          return;
        }
        std::vector<ObstacleObservation> obstacles;
        obstacles.reserve(msg->obstacles.size());
        for (const auto& obstacle : msg->obstacles) {
          if (obstacle.type >= opti_pessi::kNumObstacleTypes) {
            RCLCPP_WARN_THROTTLE(ros2_node_->get_logger(), *ros2_node_->get_clock(), 5000, "[OptiPessi] obstacle of unknown type %u ignored",
                                 static_cast<unsigned>(obstacle.type));
            continue;
          }
          obstacles.push_back({obstacle.position.x, obstacle.position.y, static_cast<opti_pessi::ObstacleType>(obstacle.type)});
        }
        std::lock_guard<std::mutex> lock(optiPessiReferenceMutex_);
        optiPessiObstacles_ = std::move(obstacles);
      });
}

void OptiPessiController::updateStateEstimation(const rclcpp::Time& time, const rclcpp::Duration& period) {
  vector_t jointPos(hybridJointHandles_.size()), jointVel(hybridJointHandles_.size());
  contact_flag_t contactFlag;
  Eigen::Quaternion<scalar_t> quat;
  vector3_t angularVel, linearAccel;
  matrix3_t orientationCovariance, angularVelCovariance, linearAccelCovariance;

  for (size_t i = 0; i < hybridJointHandles_.size(); ++i) {
    jointPos(i) = hybridJointHandles_[i].getPosition();
    jointVel(i) = hybridJointHandles_[i].getVelocity();
  }
  for (size_t i = 0; i < contactHandles_.size(); ++i) {
    if (contactHandles_[i].contactState) {
      contactFlag[i] = contactHandles_[i].isContact();
    } else {
      contactFlag[i] = topicContacts_[i];
    }
  }
  const double* ori = imuSensorHandle_.getOrientation();
  quat.coeffs() << ori[0], ori[1], ori[2], ori[3];
  
  const double* angVel = imuSensorHandle_.getAngularVelocity();
  angularVel << angVel[0], angVel[1], angVel[2];
  
  const double* linAcc = imuSensorHandle_.getLinearAcceleration();
  linearAccel << linAcc[0], linAcc[1], linAcc[2];

  orientationCovariance.setZero();
  angularVelCovariance.setZero();
  linearAccelCovariance.setZero();

  stateEstimate_->updateJointStates(jointPos, jointVel);
  stateEstimate_->updateContact(contactFlag);
  stateEstimate_->updateImu(quat, angularVel, linearAccel, orientationCovariance, angularVelCovariance, linearAccelCovariance);
  measuredRbdState_ = stateEstimate_->update(time, period);
  currentObservation_.time += period.seconds();
  scalar_t yawLast = currentObservation_.state(9);
  currentObservation_.state = rbdConversions_->computeCentroidalStateFromRbdModel(measuredRbdState_);
  currentObservation_.state(9) = yawLast + angles::shortest_angular_distance(yawLast, currentObservation_.state(9));
  currentObservation_.mode = stateEstimate_->getMode();
}

OptiPessiController::~OptiPessiController() {
  controllerRunning_ = false;
  if (optiPessiMpcThread_.joinable()) {
    optiPessiMpcThread_.join();
  }
  if (leggedMpcThread_.joinable()) {
    leggedMpcThread_.join();
  }

  executor_->cancel();
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }

  std::cerr << "\n### OptiPessi MPC Benchmarking";
  std::cerr << "\n###   Maximum : " << optiPessiMpcTimer_.getMaxIntervalInMilliseconds() << "[ms].";
  std::cerr << "\n###   Average : " << optiPessiMpcTimer_.getAverageInMilliseconds() << "[ms]." << std::endl;
  std::cerr << "\n### Legged MPC Benchmarking";
  std::cerr << "\n###   Maximum : " << leggedMpcTimer_.getMaxIntervalInMilliseconds() << "[ms].";
  std::cerr << "\n###   Average : " << leggedMpcTimer_.getAverageInMilliseconds() << "[ms]." << std::endl;
  std::cerr << "\n### WBC Benchmarking";
  std::cerr << "\n###   Maximum : " << wbcTimer_.getMaxIntervalInMilliseconds() << "[ms].";
  std::cerr << "\n###   Average : " << wbcTimer_.getAverageInMilliseconds() << "[ms].";
}

void OptiPessiController::setupOptiPessiInterface(const std::string& optipessiFile, const std::string& scenarioFile,
                                                  const std::string& libraryFolder, bool recompile, opti_pessi::SolverBackend backend) {
  optiPessiInterface_ = std::make_shared<opti_pessi::OptiPessiInterface>(optipessiFile, scenarioFile, libraryFolder, recompile, true);

  // LIP mass and yaw inertia of the simulated robot, not task.info's: the URDF's total mass, and its composite
  // inertia about the vertical axis through the CoM at the default stance and comHeight.
  const auto& info = leggedInterface_->getCentroidalModelInfo();
  PinocchioInterface pinocchioInterface = leggedInterface_->getPinocchioInterface();
  vector_t q = vector_t::Zero(info.generalizedCoordinatesNum);
  q(2) = optiPessiInterface_->modelParameters().comHeight;
  q.tail(info.actuatedDofNum) = centroidal_model::getJointAngles(leggedInterface_->getInitialState(), info);
  pinocchio::ccrba(pinocchioInterface.getModel(), pinocchioInterface.getData(), q, vector_t::Zero(info.generalizedCoordinatesNum));
  const scalar_t mass = info.robotMass;
  const scalar_t inertia = pinocchioInterface.getData().Ig.inertia().matrix()(2, 2);
  RCLCPP_INFO(this->get_node()->get_logger(), "[OptiPessi] LIP model from URDF: mass %.3f kg (task.info %.3f), yaw inertia %.4f kg m^2 (task.info %.4f)",
              mass, optiPessiInterface_->modelParameters().mass, inertia, optiPessiInterface_->modelParameters().inertia);
  optiPessiInterface_->setRobotModel(mass, inertia);

  // Both values are compiled into the CppAD libraries: one library folder per model, so a different model generates
  // its own libraries instead of loading stale ones.
  char modelFolder[64];
  std::snprintf(modelFolder, sizeof(modelFolder), "/m%.3f_I%.4f", mass, inertia);
  optiPessiInterface_->setupOptimalControlProblem(libraryFolder + modelFolder, recompile, backend);
  optiPessiDetour_ = std::make_unique<opti_pessi::ObstacleDetour>(optiPessiInterface_->modelParameters());
}

void OptiPessiController::setupLeggedInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                            bool verbose) {
  leggedInterface_ = std::make_shared<LeggedInterface>(taskFile, urdfFile, referenceFile);
  leggedInterface_->setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose);
  // The state estimator needs these whichever MPC is running, and only LeggedInterface has a whole-body model.
  rbdConversions_ = std::make_shared<CentroidalModelRbdConversions>(leggedInterface_->getPinocchioInterface(),
                                                                    leggedInterface_->getCentroidalModelInfo());
}

void OptiPessiController::setupOptiPessiMpc() {
  // Not IpmMpc: a new contact phase must warm-start from the previous plan shifted one knot, see OptiPessiMpc.h.
  optiPessiMpc_ = std::make_shared<opti_pessi::OptiPessiMpc>(optiPessiInterface_->mpcSettings(), optiPessiInterface_->ipmSettings(),
                                                             optiPessiInterface_->getOptimalControlProblem(),
                                                             optiPessiInterface_->getInitializer(),
                                                             optiPessiInterface_->getOptiPessiReferenceManagerPtr(),
                                                             optiPessiInterface_->modelParameters());

  const std::string robotName = "opti_pessi_robot";

  // Gait receiver
  // auto gaitReceiverPtr =
  //     std::make_shared<GaitReceiver>(ros2_node_, optiPessiInterface_->getSwitchedModelReferenceManagerPtr()->getGaitSchedule(), robotName);
  // ROS ReferenceManager
  auto rosReferenceManagerPtr = std::make_shared<RosReferenceManager>(robotName, optiPessiInterface_->getReferenceManagerPtr());
  rosReferenceManagerPtr->subscribe(ros2_node_);
  // optiPessiMpc_->getSolverPtr()->addSynchronizedModule(gaitReceiverPtr);
  optiPessiMpc_->getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);
  optiPessiObservationPublisher_ = ros2_node_->create_publisher<ocs2_msgs::msg::MpcObservation>(robotName + "_mpc_observation", 1);
  optiPessiPlanPublisher_ = ros2_node_->create_publisher<visualization_msgs::msg::MarkerArray>("/opti_pessi/plan", 1);
  optiPessiTrajectoryPublisher_ =
      ros2_node_->create_publisher<visualization_msgs::msg::MarkerArray>("/opti_pessi/optimizedStateTrajectory", 1);
  setupOptiPessiReferenceSubscribers();
}

void OptiPessiController::setupLeggedMpc() {
  leggedMpc_ = std::make_shared<SqpMpc>(leggedInterface_->mpcSettings(), leggedInterface_->sqpSettings(),
                                        leggedInterface_->getOptimalControlProblem(), leggedInterface_->getInitializer());

  const std::string robotName = "legged_robot";

  // Gait receiver
  auto gaitReceiverPtr =
      std::make_shared<GaitReceiver>(ros2_node_, leggedInterface_->getSwitchedModelReferenceManagerPtr()->getGaitSchedule(), robotName);
  // ROS ReferenceManager
  auto rosReferenceManagerPtr = std::make_shared<RosReferenceManager>(robotName, leggedInterface_->getReferenceManagerPtr());
  rosReferenceManagerPtr->subscribe(ros2_node_);
  leggedMpc_->getSolverPtr()->addSynchronizedModule(gaitReceiverPtr);
  leggedMpc_->getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);
  leggedObservationPublisher_ = ros2_node_->create_publisher<ocs2_msgs::msg::MpcObservation>(robotName + "_mpc_observation", 1);
}

void OptiPessiController::setupOptiPessiMrt() {
  optiPessiMrtInterface_ = std::make_shared<MPC_MRT_Interface>(*optiPessiMpc_);
  optiPessiMrtInterface_->initRollout(&optiPessiInterface_->getRollout());
  optiPessiMpcTimer_.reset();

  controllerRunning_ = true;
  optiPessiMpcThread_ = std::thread([&]() {
    while (controllerRunning_) {
      try {
        executeAndSleep(
            [&]() {
              if (mpcRunning_) {
                // Here and not in restartFromStance(): on this thread no solve can be in progress.
                if (optiPessiMpcResetRequested_.exchange(false)) {
                  optiPessiMpc_->reset();
                }
                pushOptiPessiObservation();
                optiPessiMpcTimer_.startTimer();
                optiPessiMrtInterface_->advanceMpc();
                optiPessiMpcTimer_.endTimer();
              }
            },
            optiPessiInterface_->mpcSettings().mpcDesiredFrequency_);
      } catch (const std::exception& e) {
        controllerRunning_ = false;
        RCLCPP_ERROR(ros2_node_->get_logger(), "[OptiPessi MPC thread] Error : %s", e.what());
      }
    }
  });
  setThreadPriority(optiPessiInterface_->ipmSettings().threadPriority, optiPessiMpcThread_);
}

void OptiPessiController::setupLeggedMrt() {
  leggedMrtInterface_ = std::make_shared<MPC_MRT_Interface>(*leggedMpc_);
  leggedMrtInterface_->initRollout(&leggedInterface_->getRollout());
  leggedMpcTimer_.reset();

  controllerRunning_ = true;
  leggedMpcThread_ = std::thread([&]() {
    while (controllerRunning_) {
      try {
        executeAndSleep(
            [&]() {
              if (mpcRunning_) {
                leggedMpcTimer_.startTimer();
                leggedMrtInterface_->advanceMpc();
                leggedMpcTimer_.endTimer();
              }
            },
            leggedInterface_->mpcSettings().mpcDesiredFrequency_);
      } catch (const std::exception& e) {
        controllerRunning_ = false;
        RCLCPP_ERROR(ros2_node_->get_logger(), "[Legged MPC thread] Error : %s", e.what());
      }
    }
  });
  setThreadPriority(leggedInterface_->sqpSettings().threadPriority, leggedMpcThread_);
}

void OptiPessiController::setupStateEstimate(const std::string& taskFile, bool verbose) {
  stateEstimate_ = std::make_shared<KalmanFilterEstimate>(ros2_node_, leggedInterface_->getPinocchioInterface(),
                                                          leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_);
  dynamic_cast<KalmanFilterEstimate&>(*stateEstimate_).loadSettings(taskFile, verbose);
  currentObservation_.time = 0;
}

}  // namespace legged

PLUGINLIB_EXPORT_CLASS(legged::OptiPessiController, controller_interface::ControllerInterface)
