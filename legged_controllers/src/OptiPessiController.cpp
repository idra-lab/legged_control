//
// Refactored for ROS 2 Control
//

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "legged_controllers/OptiPessiController.h"

#include "legged_controllers/HardwareCommandWriter.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_core/thread_support/SetThreadPriority.h>
#include <ocs2_legged_robot_ros/gait/GaitReceiver.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <ocs2_sqp/SqpMpc.h>
#include <ocs2_ipm/IpmMpc.h>

#include <angles/angles.h>
#include <legged_estimation/FromTopiceEstimate.h>
#include <legged_estimation/LinearKalmanFilter.h>
#include <legged_wbc/HierarchicalWbc.h>
#include <legged_wbc/WeightedWbc.h>
#include <pluginlib/class_list_macros.hpp>

#include <opti_pessi_interface/OptiPessiInterface.h>
#include <opti_pessi_interface/initialization/OptiPessiInitializer.h>

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

  setupOptiPessiInterface(optipessiFile, scenarioFile, libraryFolder, recompile, backend);
  setupLeggedInterface(taskFile, urdfFile, referenceFile, verbose);
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
  wbc_ = std::make_shared<WeightedWbc>(leggedInterface_->getPinocchioInterface(), leggedInterface_->getCentroidalModelInfo(),
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

  // Opti-Pessi LIP loop: every phase starts from the measured robot.
  const vector_t robotState = measureLipState(0);
  RCLCPP_INFO(node->get_logger(), "[OptiPessi] measured initial state: c=(%.3f, %.3f) theta=%.3f v=(%.3f, %.3f) dtheta=%.3f "
              "p0=(%.3f, %.3f) p1=(%.3f, %.3f)",
              robotState(0), robotState(1), robotState(2), robotState(3), robotState(4), robotState(5), robotState(6),
              robotState(7), robotState(8), robotState(9));
  {
    std::lock_guard<std::mutex> lock(optiPessiPhaseMutex_);
    optiPessiRobotState_ = robotState;
    optiPessiPhase_ = 0;
  }
  optiPessiPhaseElapsed_ = 0.0;
  optiPessiGoalReached_ = false;

  RCLCPP_INFO(node->get_logger(), "Waiting for the initial Opti-Pessi policy ...");
  while (!optiPessiMrtInterface_->initialPolicyReceived() && rclcpp::ok()) {
    pushOptiPessiObservation();
    optiPessiMrtInterface_->advanceMpc();
    rclcpp::Rate(optiPessiInterface_->mpcSettings().mrtDesiredFrequency_).sleep();
  }
  RCLCPP_INFO(node->get_logger(), "Initial Opti-Pessi policy has been received.");

  mpcRunning_ = true;

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn OptiPessiController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
  mpcRunning_ = false;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type OptiPessiController::update(const rclcpp::Time& time, const rclcpp::Duration& period) {
  // State Estimate
  updateStateEstimation(time, period);

  // Load the latest MPC policy
  optiPessiMrtInterface_->updatePolicy();

  // A policy solved for an earlier phase would apply that phase's footholds: hold the LIP clock
  // until the MPC thread has caught up with the current phase.
  const CommandData& command = optiPessiMrtInterface_->getCommand();
  if (optiPessiGoalReached_ || command.mpcInitObservation_.mode != optiPessiPhase_) {
    return controller_interface::return_type::OK;
  }

  // Every solve starts at knot 0 (see opti_pessi_interface/definitions.h), so the input at t = 0 is
  // the one applied over this whole phase; it lasts u(DT) seconds.
  vector_t plannedState, plannedInput;
  size_t plannedMode = 0;
  optiPessiMrtInterface_->evaluatePolicy(0.0, command.mpcInitObservation_.state, plannedState, plannedInput, plannedMode);
  const vector_t appliedInput = opti_pessi::extractRobotInput(plannedInput);
  const scalar_t phaseDuration = appliedInput(opti_pessi::RobotU::DT);

  optiPessiPhaseElapsed_ += period.seconds();
  if (optiPessiPhaseElapsed_ < phaseDuration) {
    return controller_interface::return_type::OK;
  }

  // Phase over: measure the robot for the next phase's stance pair and hand it to the MPC thread.
  const auto& params = optiPessiInterface_->modelParameters();
  const vector_t successorState = measureLipState(optiPessiPhase_ + 1);
  if (opti_pessi::isInsane(successorState, appliedInput, params)) {
    RCLCPP_WARN(this->get_node()->get_logger(), "[OptiPessi] phase %zu: measured state or applied input outside the LIP bounds",
                optiPessiPhase_);
  }
  {
    std::lock_guard<std::mutex> lock(optiPessiPhaseMutex_);
    optiPessiRobotState_ = successorState;
    ++optiPessiPhase_;
  }
  optiPessiPhaseElapsed_ -= phaseDuration;

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
    RCLCPP_INFO(this->get_node()->get_logger(), "[OptiPessi] goal reached after %zu phases", optiPessiPhase_);
  }

  return controller_interface::return_type::OK;
}

vector_t OptiPessiController::measureLipState(size_t phase) const {
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
  pinocchio::forwardKinematics(model, data, qPino);
  pinocchio::updateFramePlacements(model, data);
  const vector3_t com = pinocchio::centerOfMass(model, data, qPino);

  vector_t lipState = vector_t::Zero(RobotX::DIM);
  lipState(RobotX::CX) = com(0);
  lipState(RobotX::CY) = com(1);
  lipState(RobotX::TH) = rbd(0);
  lipState(RobotX::DCX) = rbd(nq + 3);
  lipState(RobotX::DCY) = rbd(nq + 4);
  lipState(RobotX::DTH) = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<scalar_t>(zyx, angularVelWorld)(0);

  // Indexed by opti_pessi::Foot (FL, FR, RL, RR).
  const std::array<std::string, 4> footFrames{"LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"};
  const auto stance = opti_pessi::gaitPair(static_cast<int>(phase));
  lipState.segment(RobotX::P0X, 2) = data.oMf[model.getFrameId(footFrames[static_cast<size_t>(stance[0])])].translation().head<2>();
  lipState.segment(RobotX::P1X, 2) = data.oMf[model.getFrameId(footFrames[static_cast<size_t>(stance[1])])].translation().head<2>();
  return lipState;
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
  optiPessiInterface_->setupOptimalControlProblem(libraryFolder, recompile, backend);
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
  optiPessiMpc_ = std::make_shared<IpmMpc>(optiPessiInterface_->mpcSettings(), optiPessiInterface_->ipmSettings(),
                                          optiPessiInterface_->getOptimalControlProblem(), optiPessiInterface_->getInitializer());

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
