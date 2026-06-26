//
// Refactored for ROS 2 Control
//

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include "legged_controllers/LeggedController.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_core/thread_support/SetThreadPriority.h>
#include <ocs2_legged_robot_ros/gait/GaitReceiver.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <ocs2_sqp/SqpMpc.h>

#include <angles/angles.h>
#include <legged_estimation/FromTopiceEstimate.h>
#include <legged_estimation/LinearKalmanFilter.h>
#include <legged_wbc/HierarchicalWbc.h>
#include <legged_wbc/WeightedWbc.h>
#include <pluginlib/class_list_macros.hpp>

using namespace std;

namespace legged {

controller_interface::CallbackReturn LeggedController::on_init() {
  auto node = this->get_node();

  // Declare parameters
  node->declare_parameter<std::string>("urdfFile", "");
  node->declare_parameter<std::string>("taskFile", "");
  node->declare_parameter<std::string>("referenceFile", "");

  std::string urdfFile = node->get_parameter("urdfFile").as_string();
  std::string taskFile = node->get_parameter("taskFile").as_string();
  std::string referenceFile = node->get_parameter("referenceFile").as_string();

  bool verbose = true;
  loadData::loadCppDataType(taskFile, "legged_robot_interface.verbose", verbose);

  // Helper node for OCS2 ROS 2 communication
  ros2_node_ = std::make_shared<rclcpp::Node>("legged_controller_ros2_node");
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(ros2_node_);
  spin_thread_ = std::thread([this]() {
    executor_->spin();
  });

  setupLeggedInterface(taskFile, urdfFile, referenceFile, verbose);
  setupMpc();
  setupMrt();

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

controller_interface::CallbackReturn LeggedController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration LeggedController::command_interface_configuration() const {
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

controller_interface::InterfaceConfiguration LeggedController::state_interface_configuration() const {
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

controller_interface::CallbackReturn LeggedController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
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

  TargetTrajectories target_trajectories({currentObservation_.time}, {currentObservation_.state}, {currentObservation_.input});

  mpcMrtInterface_->setCurrentObservation(currentObservation_);
  mpcMrtInterface_->getReferenceManager().setTargetTrajectories(target_trajectories);
  RCLCPP_INFO(node->get_logger(), "Waiting for the initial policy ...");
  while (!mpcMrtInterface_->initialPolicyReceived() && rclcpp::ok()) {
    mpcMrtInterface_->advanceMpc();
    rclcpp::Rate(leggedInterface_->mpcSettings().mrtDesiredFrequency_).sleep();
  }
  RCLCPP_INFO(node->get_logger(), "Initial policy has been received.");

  mpcRunning_ = true;

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LeggedController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
  mpcRunning_ = false;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type LeggedController::update(const rclcpp::Time& time, const rclcpp::Duration& period) {
  // State Estimate
  updateStateEstimation(time, period);

  // Update the current state of the system
  mpcMrtInterface_->setCurrentObservation(currentObservation_);

  // Load the latest MPC policy
  mpcMrtInterface_->updatePolicy();

  // Evaluate the current policy
  vector_t optimizedState, optimizedInput;
  size_t plannedMode = 0;
  mpcMrtInterface_->evaluatePolicy(currentObservation_.time, currentObservation_.state, optimizedState, optimizedInput, plannedMode);

  // Whole body control
  currentObservation_.input = optimizedInput;

  wbcTimer_.startTimer();
  vector_t x = wbc_->update(optimizedState, optimizedInput, measuredRbdState_, plannedMode, period.seconds());
  wbcTimer_.endTimer();

  vector_t torque = x.tail(12);

  vector_t posDes = centroidal_model::getJointAngles(optimizedState, leggedInterface_->getCentroidalModelInfo());
  vector_t velDes = centroidal_model::getJointVelocities(optimizedInput, leggedInterface_->getCentroidalModelInfo());

  // Safety check, if failed, stop the controller
  if (!safetyChecker_->check(currentObservation_, optimizedState, optimizedInput)) {
    RCLCPP_ERROR(this->get_node()->get_logger(), "[Legged Controller] Safety check failed!");
    return controller_interface::return_type::ERROR;
  }

  for (size_t j = 0; j < leggedInterface_->getCentroidalModelInfo().actuatedDofNum; ++j) {
    hybridJointHandles_[j].setCommand(posDes(j), velDes(j), 0, 3, torque(j));
  }

  // Visualization
  robotVisualizer_->update(currentObservation_, mpcMrtInterface_->getPolicy(), mpcMrtInterface_->getCommand());
  selfCollisionVisualization_->update(currentObservation_);

  // Publish the observation
  observationPublisher_->publish(ros_msg_conversions::createObservationMsg(currentObservation_));

  return controller_interface::return_type::OK;
}

void LeggedController::updateStateEstimation(const rclcpp::Time& time, const rclcpp::Duration& period) {
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

LeggedController::~LeggedController() {
  controllerRunning_ = false;
  if (mpcThread_.joinable()) {
    mpcThread_.join();
  }
  
  executor_->cancel();
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }

  std::cerr << "\n### MPC Benchmarking";
  std::cerr << "\n###   Maximum : " << mpcTimer_.getMaxIntervalInMilliseconds() << "[ms].";
  std::cerr << "\n###   Average : " << mpcTimer_.getAverageInMilliseconds() << "[ms]." << std::endl;
  std::cerr << "\n### WBC Benchmarking";
  std::cerr << "\n###   Maximum : " << wbcTimer_.getMaxIntervalInMilliseconds() << "[ms].";
  std::cerr << "\n###   Average : " << wbcTimer_.getAverageInMilliseconds() << "[ms].";
}

void LeggedController::setupLeggedInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                            bool verbose) {
  leggedInterface_ = std::make_shared<LeggedInterface>(taskFile, urdfFile, referenceFile);
  leggedInterface_->setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose);
}

void LeggedController::setupMpc() {
  mpc_ = std::make_shared<SqpMpc>(leggedInterface_->mpcSettings(), leggedInterface_->sqpSettings(),
                                  leggedInterface_->getOptimalControlProblem(), leggedInterface_->getInitializer());
  rbdConversions_ = std::make_shared<CentroidalModelRbdConversions>(leggedInterface_->getPinocchioInterface(),
                                                                    leggedInterface_->getCentroidalModelInfo());

  const std::string robotName = "legged_robot";
  
  // Gait receiver
  auto gaitReceiverPtr =
      std::make_shared<GaitReceiver>(ros2_node_, leggedInterface_->getSwitchedModelReferenceManagerPtr()->getGaitSchedule(), robotName);
  // ROS ReferenceManager
  auto rosReferenceManagerPtr = std::make_shared<RosReferenceManager>(robotName, leggedInterface_->getReferenceManagerPtr());
  rosReferenceManagerPtr->subscribe(ros2_node_);
  mpc_->getSolverPtr()->addSynchronizedModule(gaitReceiverPtr);
  mpc_->getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);
  observationPublisher_ = ros2_node_->create_publisher<ocs2_msgs::msg::MpcObservation>(robotName + "_mpc_observation", 1);
}

void LeggedController::setupMrt() {
  mpcMrtInterface_ = std::make_shared<MPC_MRT_Interface>(*mpc_);
  mpcMrtInterface_->initRollout(&leggedInterface_->getRollout());
  mpcTimer_.reset();

  controllerRunning_ = true;
  mpcThread_ = std::thread([&]() {
    while (controllerRunning_) {
      try {
        executeAndSleep(
            [&]() {
              if (mpcRunning_) {
                mpcTimer_.startTimer();
                mpcMrtInterface_->advanceMpc();
                mpcTimer_.endTimer();
              }
            },
            leggedInterface_->mpcSettings().mpcDesiredFrequency_);
      } catch (const std::exception& e) {
        controllerRunning_ = false;
        RCLCPP_ERROR(ros2_node_->get_logger(), "[Ocs2 MPC thread] Error : %s", e.what());
      }
    }
  });
  setThreadPriority(leggedInterface_->sqpSettings().threadPriority, mpcThread_);
}

void LeggedController::setupStateEstimate(const std::string& taskFile, bool verbose) {
  stateEstimate_ = std::make_shared<KalmanFilterEstimate>(ros2_node_, leggedInterface_->getPinocchioInterface(),
                                                          leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_);
  dynamic_cast<KalmanFilterEstimate&>(*stateEstimate_).loadSettings(taskFile, verbose);
  currentObservation_.time = 0;
}

void LeggedCheaterController::setupStateEstimate(const std::string& /*taskFile*/, bool /*verbose*/) {
  stateEstimate_ = std::make_shared<FromTopicStateEstimate>(ros2_node_, leggedInterface_->getPinocchioInterface(),
                                                            leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_);
}

}  // namespace legged

PLUGINLIB_EXPORT_CLASS(legged::LeggedController, controller_interface::ControllerInterface)
PLUGINLIB_EXPORT_CLASS(legged::LeggedCheaterController, controller_interface::ControllerInterface)