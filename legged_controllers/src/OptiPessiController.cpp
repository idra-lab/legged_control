//
// Refactored for ROS 2 Control
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

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

using namespace std;

namespace legged {

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

  std::string urdfFile = node->get_parameter("urdfFile").as_string();
  std::string taskFile = node->get_parameter("taskFile").as_string();
  std::string referenceFile = node->get_parameter("referenceFile").as_string();
  std::string optipessiFile = node->get_parameter("optipessiFile").as_string();
  std::string scenarioFile = node->get_parameter("scenarioFile").as_string();
  std::string libraryFolder = node->get_parameter("libraryFolder").as_string();
  bool recompile = node->get_parameter("recompile").as_bool();
  std::string backendStr = node->get_parameter("backend").as_string();
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
  {
    boost::property_tree::ptree pt;
    boost::property_tree::read_info(taskFile, pt);
    loadData::loadPtreeValue(pt, optiPessiMaxPlanShift_, "optiPessiController.maxPlanShift", verbose);
  }
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
  optiPessiPhaseDiagnostics_ = PhaseDiagnostics();
  optiPessiQpFailuresAtPhaseStart_ = wbc_->getNumQpFailures();
  mpcRunning_ = false;
  // A policy left from an earlier activation must not count as the first policy of this one.
  optiPessiMrtInterface_->reset();
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
  if (optiPessiMrtInterface_->updatePolicy()) {
    ++optiPessiPhaseDiagnostics_.policyUpdates;
    storeAcceptedPlan();
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
  // Standing at the goal: holdStance() has set the references once.
  if (optiPessiGoalReached_) {
    return;
  }

  // No accepted plan yet (the MPC starts when standUp() ends): keep standing.
  if (!optiPessiPlan_.valid || optiPessiPlan_.phase > optiPessiPhase_) {
    optiPessiWaitTime_ += period.seconds();
    return;
  }

  // Every solve starts at knot 0 (see opti_pessi_interface/definitions.h). Until the MPC thread delivers this phase's
  // own policy, the latest accepted plan runs shifted by the phases completed since it was solved, from the state
  // measured at the start of this phase: the LIP clock never waits for a solve. The input lasts u(DT) seconds.
  const size_t offset = optiPessiPhase_ - optiPessiPlan_.phase;
  if (offset > optiPessiMaxPlanShift_ || offset >= optiPessiPlan_.inputs.size()) {
    RCLCPP_WARN(this->get_node()->get_logger(),
                "[OptiPessi] phase %zu: latest accepted plan is from phase %zu (max shift %zu), stopping to restart from stance",
                optiPessiPhase_, optiPessiPlan_.phase, optiPessiMaxPlanShift_);
    restartFromStance();
    return;
  }
  if (offset > 0) {
    optiPessiWaitTime_ += period.seconds();
  }
  const vector_t robotState = offset == 0 ? optiPessiPlan_.startState : optiPessiRobotState_;
  const vector_t appliedInput = optiPessiPlan_.inputs[offset];
  const scalar_t phaseDuration = appliedInput(opti_pessi::RobotU::DT);

  // Plan stability over the phase: the MPC keeps re-solving it, and each new policy can move dt and the footholds.
  {
    PhaseDiagnostics& diagnostics = optiPessiPhaseDiagnostics_;
    diagnostics.durationMin = std::min(diagnostics.durationMin, phaseDuration);
    diagnostics.durationMax = std::max(diagnostics.durationMax, phaseDuration);
    const vector_t footholds = appliedInput.head(4);
    if (diagnostics.firstFootholds.size() == 0) {
      diagnostics.firstFootholds = footholds;
    }
    diagnostics.footholdDrift = std::max(diagnostics.footholdDrift, (footholds - diagnostics.firstFootholds).cwiseAbs().maxCoeff());
  }

  // The next knot is the stance the swing feet land in, so its forces are their touchdown forces.
  const auto& params = optiPessiInterface_->modelParameters();
  const vector_t nextState = opti_pessi::lipMapScalar(robotState, appliedInput, params.omega(), params.mass, params.inertia);
  const vector_t& nextInput = optiPessiPlan_.inputs[std::min(offset + 1, optiPessiPlan_.inputs.size() - 1)];

  optiPessiPhaseElapsed_ += period.seconds();
  updateFootReferences(robotState, appliedInput, nextState, nextInput, optiPessiPhaseElapsed_);
  updateComReference(robotState, appliedInput, optiPessiPhaseElapsed_);
  if (optiPessiPhaseElapsed_ < phaseDuration) {
    return;
  }

  // Phase over: measure the robot for the next phase's stance pair and hand it to the MPC thread.
  const vector_t successorState = measureLipState(optiPessiPhase_ + 1, optiPessiLiftoffPositions_);
  if (opti_pessi::isInsane(successorState, appliedInput, params)) {
    RCLCPP_WARN(this->get_node()->get_logger(), "[OptiPessi] phase %zu: measured state or applied input outside the LIP bounds",
                optiPessiPhase_);
  }

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
                  "foothold drift=%.3f policy updates=%zu",
                  optiPessiPhase_, kFootNames[s0], optiPessiLiftoffPositions_[s0].z() - optiPessiFootReferences_[s0].position.z(),
                  measuredContacts[s0] ? 1 : 0, kFootNames[s1],
                  optiPessiLiftoffPositions_[s1].z() - optiPessiFootReferences_[s1].position.z(), measuredContacts[s1] ? 1 : 0,
                  d.durationMin, d.durationMax, d.footholdDrift, d.policyUpdates);
    }
    optiPessiPhaseDiagnostics_ = PhaseDiagnostics();

    // Plan quality: the applied knot as the OCP bounds it (StageInequalityConstraint rows on x_1), and the latest
    // MPC solve. A predicted successor beyond the limits means the applied policy was not a feasible plan.
    using vector2_t = Eigen::Matrix<scalar_t, 2, 1>;
    const vector2_t dcBody = opti_pessi::applyR01(predicted(RobotX::TH), vector2_t(predicted(RobotX::DCX), predicted(RobotX::DCY)));
    const auto solve = std::static_pointer_cast<opti_pessi::OptiPessiMpc>(optiPessiMpc_)->getLastSolveStatistics();
    RCLCPP_INFO(this->get_node()->get_logger(),
                "[OptiPessi] phase %zu plan: predicted dcBody=(%.3f, %.3f) dth=%.3f (limits %.2f, %.2f, %.2f) | latest solve: "
                "gaitOffset=%d warmStart=%s iterations=%zu cost=%.3e accepted=%d dynRes=%.3e appliedViol=%.3e horizonViol=%.3e | "
                "applied plan of phase %zu (shift %zu)",
                optiPessiPhase_, dcBody(0), dcBody(1), predicted(RobotX::DTH), params.dcxMax, params.dcyMax, params.dthetaMax,
                solve.gaitOffset, solve.warmStart, solve.numIterations, solve.performance.cost, solve.accepted ? 1 : 0,
                solve.dynamicsResidual, solve.appliedViolation, solve.horizonViolation, optiPessiPlan_.phase, offset);
  }
  {
    std::lock_guard<std::mutex> lock(optiPessiPhaseMutex_);
    optiPessiRobotState_ = successorState;
    ++optiPessiPhase_;
  }
  optiPessiPhaseElapsed_ -= phaseDuration;
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

  if ((successorState.head(2) - params.goal).norm() < params.goalTolerance) {
    optiPessiGoalReached_ = true;
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

  // Standing: phase 0 starts from the robot measured now, and the MPC thread starts solving it.
  const vector_t robotState = measureLipState(0, optiPessiLiftoffPositions_);
  for (size_t i = 0; i < optiPessiFootReferences_.size(); ++i) {
    optiPessiFootReferences_[i].position = optiPessiLiftoffPositions_[i];
  }
  holdStance(vector3_t(robotState(RobotX::CX), robotState(RobotX::CY), params.comHeight), robotState(RobotX::TH));
  {
    std::lock_guard<std::mutex> lock(optiPessiPhaseMutex_);
    optiPessiRobotState_ = robotState;
    optiPessiPhase_ = 0;
  }
  optiPessiPhaseElapsed_ = 0.0;
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

void OptiPessiController::storeAcceptedPlan() {
  // During the stand-up or a restart the phase is about to be reset: nothing solved now belongs to it.
  if (optiPessiStandingUp_) {
    return;
  }
  const PrimalSolution& policy = optiPessiMrtInterface_->getPolicy();
  const SystemObservation& observation = optiPessiMrtInterface_->getCommand().mpcInitObservation_;
  const size_t phase = observation.mode;
  const auto numKnots = static_cast<size_t>(optiPessiInterface_->modelParameters().N);
  if (policy.inputTrajectory_.size() < numKnots) {
    return;
  }

  // Only accepted solves reach the MRT (see OptiPessiMpc::run). One solved for this phase must also start from the
  // state this phase started from, which rejects a solve that was in flight across a restart. The observation is an
  // exact copy of that state; the solver's knot 0 is not (the interior point step need not close the x_0 defect).
  const vector_t startState = opti_pessi::extractRobotState(observation.state);
  const bool forThisPhase = phase == optiPessiPhase_ && optiPessiRobotState_.size() == startState.size() &&
                            (startState - optiPessiRobotState_).cwiseAbs().maxCoeff() < 1e-6;
  const bool newerEarlierPhase = phase < optiPessiPhase_ && (!optiPessiPlan_.valid || phase > optiPessiPlan_.phase);
  if (!forThisPhase && !newerEarlierPhase) {
    return;
  }

  optiPessiPlan_.valid = true;
  optiPessiPlan_.phase = phase;
  optiPessiPlan_.startState = startState;
  optiPessiPlan_.inputs.clear();
  for (size_t k = 0; k < numKnots; ++k) {
    optiPessiPlan_.inputs.push_back(opti_pessi::extractRobotInput(policy.inputTrajectory_[k]));
  }
}

void OptiPessiController::restartFromStance() {
  using opti_pessi::RobotX;
  const auto& params = optiPessiInterface_->modelParameters();
  mpcRunning_ = false;
  optiPessiPlan_ = AcceptedPlan();
  optiPessiMrtInterface_->reset();

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

void OptiPessiController::holdStance(const vector3_t& comPosition, scalar_t yaw) {
  const auto& params = optiPessiInterface_->modelParameters();
  for (FootReference& reference : optiPessiFootReferences_) {
    reference.contact = true;
    reference.velocity.setZero();
    reference.force << 0.0, 0.0, 0.25 * params.mass * params.gravity;
    reference.touchdownForce = reference.force;
  }
  optiPessiComReference_ = ComReference();
  optiPessiComReference_.position = comPosition;
  optiPessiComReference_.yaw = yaw;
}

void OptiPessiController::publishOptiPessiPlan() {
  using opti_pessi::RobotX;
  using visualization_msgs::msg::Marker;
  const auto& params = optiPessiInterface_->modelParameters();
  const PrimalSolution& policy = optiPessiMrtInterface_->getPolicy();
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

  // Planned CoM path of the pessimistic branch at the LIP height, one point per knot. The optimistic
  // branch, the one executed, is drawn with its feet by publishOptiPessiTrajectories().
  Marker pessimisticCom = makeMarker("pessimistic_com", 0, Marker::LINE_STRIP, 0.9F, 0.2F, 0.1F, 1.0F);
  pessimisticCom.scale.x = 0.01;
  for (const vector_t& x : policy.stateTrajectory_) {
    pessimisticCom.points.push_back(point(x(RobotX::DIM + RobotX::CX), x(RobotX::DIM + RobotX::CY), params.comHeight));
  }
  markers.markers.push_back(pessimisticCom);

  // Obstacles as the OCP currently sees them (scenario frame, drawn as-is in odom).
  const matrix_t& obstacles = optiPessiInterface_->getOptiPessiReferenceManagerPtr()->getObstacles();
  for (int j = 0; j < obstacles.rows(); ++j) {
    Marker disk = makeMarker("obstacles", j, Marker::CYLINDER, 0.5F, 0.5F, 0.5F, 0.6F);
    disk.pose.position = point(obstacles(j, 0), obstacles(j, 1), 0.25);
    disk.scale.x = disk.scale.y = 2.0 * params.obstacleRadius;
    disk.scale.z = 0.5;
    markers.markers.push_back(disk);
  }

  Marker goal = makeMarker("goal", 0, Marker::SPHERE, 1.0F, 0.85F, 0.0F, 1.0F);
  goal.pose.position = point(params.goal(0), params.goal(1), 0.05);
  goal.scale.x = goal.scale.y = goal.scale.z = 0.1;
  markers.markers.push_back(goal);

  optiPessiPlanPublisher_->publish(markers);
}

void OptiPessiController::publishOptiPessiTrajectories() {
  using opti_pessi::RobotU;
  using opti_pessi::RobotX;
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

    for (size_t s = (i == 0 ? 0 : 1); s <= samplesPerPhase; ++s) {
      const scalar_t t = dt * static_cast<scalar_t>(s) / static_cast<scalar_t>(samplesPerPhase);
      const scalar_t ch = std::cosh(w * t);
      const scalar_t sh = std::sinh(w * t);
      const vector2_t com = ch * c + (sh / w) * dc + (1.0 - ch) * cop;
      comPoints.push_back(getPointMsg(vector3_t(com(0), com(1), params.comHeight)));

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
  for (size_t i = 0; i < feetPoints.size(); ++i) {
    markerArray.markers.emplace_back(
        getLineMsg(std::move(feetPoints[i]), robotVisualizer_->feetColorMap_[i], robotVisualizer_->trajectoryLineWidth_));
    markerArray.markers.back().ns = "EE Trajectories";
  }
  markerArray.markers.emplace_back(getLineMsg(std::move(comPoints), Color::red, robotVisualizer_->trajectoryLineWidth_));
  markerArray.markers.back().ns = "CoM Trajectory";
  markerArray.markers.push_back(std::move(footholds));

  // Same clock as the odom -> base TF of robotVisualizer_.
  assignHeader(markerArray.markers.begin(), markerArray.markers.end(), getHeaderMsg("odom", ros2_node_->get_clock()->now()));
  assignIncreasingId(markerArray.markers.begin(), markerArray.markers.end());
  optiPessiTrajectoryPublisher_->publish(markerArray);
}

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
  optiPessiMrtInterface_->setCurrentObservation(observation);
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
