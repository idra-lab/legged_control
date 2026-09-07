//
// OptiPessiController implementation. See the header for the design summary.
//

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>

#include "legged_controllers/OptiPessiController.h"

#include "legged_controllers/HardwareCommandWriter.h"

#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_centroidal_model/FactoryFunctions.h>
#include <ocs2_core/PreComputation.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_legged_robot/common/ModelSettings.h>
#include <ocs2_oc/oc_problem/OptimalControlProblem.h>

#include <angles/angles.h>
#include <legged_estimation/LinearKalmanFilter.h>
#include <legged_wbc/WeightedWbc.h>
#include <pluginlib/class_list_macros.hpp>

#include <opti_pessi_interface/LipKinematics.h>
#include <opti_pessi_interface/initialization/OptiPessiInitializer.h>

using namespace std;

namespace legged {

namespace {

// ===================================================================================================
// Task 13 sanity gate + capture-point fallback.
//
// This MIRRORS opti_pessi_interface/src/simulation/ClosedLoopSimulation.cpp's anonymous namespace
// (kDynamicsResidualTolerance, kAppliedViolationTolerance, appliedConstraintViolation, isInsane,
// fallbackInput, saturateRobotInput, extractSolve) as closely as the controller's single-attempt
// (no keep-out continuation, no warm start) use case allows.
//
// It is a DUPLICATE, not a reuse, because those pieces are file-local (anonymous namespace) in
// opti_pessi_interface, and this task is explicitly forbidden from editing opti_pessi_interface
// (it is being edited live by the human in another workstream) to promote them into a shared
// header. See task-13-report.md for the flag to do that promotion properly in a later cleanup, once
// opti_pessi_interface is stable again.
//
// Deliberately NOT ported: the keep-out (pessiScale) continuation retry and cross-solve warm
// starting that ClosedLoopSimulation.cpp uses to improve convergence -- see task-13-report.md for
// why those were judged out of scope for "background solve with sanity gates and fallback".
// ===================================================================================================

using opti_pessi::OptiPessiModelParameters;
using opti_pessi::RobotU;
using opti_pessi::RobotX;

/** Largest one-step dynamics residual accepted before the solve is treated as failed. */
constexpr scalar_t kDynamicsResidualTolerance = 0.02;

/** Largest violation of the APPLIED step's (knot 1's) own path inequalities before rejection. */
constexpr scalar_t kAppliedViolationTolerance = 0.05;

/**
 * Emergency step used when the solver produces nothing usable: a capture-point (deadbeat) stop.
 * Identical in intent and formula to ClosedLoopSimulation.cpp's fallbackInput() -- see the long
 * comment there for the DCM derivation and the documented limits of this fallback.
 */
vector_t fallbackInput(const OptiPessiModelParameters& params, const vector_t& robotState, int phaseIndex) {
  vector_t u = vector_t::Zero(RobotU::DIM);
  const auto next = opti_pessi::gaitPair(phaseIndex + 1);
  const vector_t c = robotState.head(2);
  const vector_t dc = robotState.segment(RobotX::DCX, 2);
  const scalar_t theta = robotState(RobotX::TH);
  const scalar_t w = params.omega();

  const vector_t dcm = c + dc / w;

  u.segment(RobotU::P0X, 2) = dcm + opti_pessi::applyR(theta, opti_pessi::hipOf(params, next[0]));
  u.segment(RobotU::P1X, 2) = dcm + opti_pessi::applyR(theta, opti_pessi::hipOf(params, next[1]));

  const vector_t p0 = robotState.segment(RobotX::P0X, 2);
  const vector_t p1 = robotState.segment(RobotX::P1X, 2);
  const vector_t d = p1 - p0;
  const scalar_t denominator = d.dot(d);
  scalar_t alpha = 0.5;
  if (denominator > 1e-9) {
    alpha = (dcm - p0).dot(d) / denominator;
  }
  u(RobotU::ALPHA) = std::min(std::max(alpha, params.alphaReduction), scalar_t(1) - params.alphaReduction);
  u(RobotU::DT) = params.dtMin;  // shortest phase: re-plan as soon as possible
  u(RobotU::BETA) = 0.5;
  u(RobotU::GAMMA) = 0.5;
  return u;
}

/** Saturates the free scalars of a (usually failed-solve) input to their bounds, footholds untouched. */
vector_t saturateRobotInput(vector_t u, const OptiPessiModelParameters& params) {
  u(RobotU::ALPHA) = std::min(std::max(u(RobotU::ALPHA), params.alphaReduction), scalar_t(1) - params.alphaReduction);
  u(RobotU::BETA) = std::min(std::max(u(RobotU::BETA), scalar_t(0)), scalar_t(1));
  u(RobotU::GAMMA) = std::min(std::max(u(RobotU::GAMMA), scalar_t(0)), scalar_t(1));
  u(RobotU::DT) = std::min(std::max(u(RobotU::DT), params.dtMin), params.dtMax);
  return u;
}

/** Guards against the solver returning a formally converged but physically nonsensical iterate. */
bool isInsane(const vector_t& robotState, const vector_t& robotInput, const OptiPessiModelParameters& params) {
  if (!robotState.allFinite() || !robotInput.allFinite()) {
    return true;
  }
  if (robotState.head(2).norm() > 10.0 || robotState.segment(RobotX::DCX, 2).norm() > params.dcxMax + 0.2 ||
      std::abs(robotState(RobotX::DTH)) > params.dthetaMax + 0.2) {
    return true;
  }
  if (robotInput(RobotU::DT) < params.dtMin - 0.05 || robotInput(RobotU::DT) > params.dtMax + 0.05) {
    return true;
  }
  return false;
}

/** Worst violation (as a positive number) of the problem's path inequalities at one knot. */
scalar_t appliedConstraintViolation(const ocs2::OptimalControlProblem& problem, scalar_t time, const vector_t& state,
                                    const vector_t& input) {
  if (problem.inequalityConstraintPtr == nullptr || problem.inequalityConstraintPtr->empty()) {
    return 0.0;
  }
  const ocs2::PreComputation preComputation;
  scalar_t worst = 0.0;
  for (const auto& g : problem.inequalityConstraintPtr->getValue(time, state, input, preComputation)) {
    if (g.size() > 0) {
      worst = std::max(worst, -std::min(scalar_t(0), g.minCoeff()));
    }
  }
  return worst;
}

/** Outcome of extracting and gating one solve's applied (knot-0) step. Mirrors SolveOutcome. */
struct SolveOutcome {
  bool ok{false};               // dynamics residual + applied violation both within tolerance, not insane
  bool haveAppliedInput{false}; // the solver returned SOME primal solution, even if `ok` is false
  vector_t appliedInput;        // meaningful only if haveAppliedInput
};

/** Mirrors ClosedLoopSimulation.cpp's extractSolve(), minus the horizon-wide plan-quality figures
 *  that only the (not-ported) keep-out continuation needs. */
SolveOutcome extractSolve(const ocs2::IpmSolver& solver, const ocs2::OptimalControlProblem& problem,
                          const OptiPessiModelParameters& params, const vector_t& robotState, scalar_t finalTime) {
  SolveOutcome out;
  ocs2::PrimalSolution solution;
  try {
    solution = solver.primalSolution(finalTime);
  } catch (const std::exception&) {
    return out;
  }
  if (solution.stateTrajectory_.size() < 2 || solution.inputTrajectory_.empty()) {
    return out;
  }

  out.appliedInput = opti_pessi::extractRobotInput(solution.inputTrajectory_.front());
  out.haveAppliedInput = true;
  const vector_t successorState =
      opti_pessi::lipMapScalar(robotState, out.appliedInput, params.omega(), params.mass, params.inertia);

  // Measured at knot 1, not knot 0 -- see the long comment at this same measurement in
  // ClosedLoopSimulation.cpp::extractSolve for why knot 0's path rows are (deliberately) inactive.
  scalar_t constraintViolation;
  if (solution.inputTrajectory_.size() > 1) {
    constraintViolation = appliedConstraintViolation(problem, 1.0, solution.stateTrajectory_[1], solution.inputTrajectory_[1]);
  } else {
    constraintViolation =
        appliedConstraintViolation(problem, 0.0, solution.stateTrajectory_.front(), solution.inputTrajectory_.front());
  }
  const vector_t solverSuccessor = opti_pessi::extractRobotState(solution.stateTrajectory_[1]);
  const scalar_t dynamicsResidual = (solverSuccessor - successorState).norm();

  if (!out.appliedInput.allFinite() || !successorState.allFinite() || isInsane(successorState, out.appliedInput, params)) {
    return out;  // ok stays false
  }
  out.ok = dynamicsResidual < kDynamicsResidualTolerance && constraintViolation < kAppliedViolationTolerance;
  return out;
}

}  // namespace

controller_interface::CallbackReturn OptiPessiController::on_init() {
  auto node = this->get_node();

  const auto declareStringParam = [&](const std::string& name, const std::string& defaultValue) {
    if (!node->has_parameter(name)) {
      node->declare_parameter<std::string>(name, defaultValue);
    }
    return node->get_parameter(name).as_string();
  };

  // Legged/quadruped model files -- same names, same meaning as LeggedController's parameters.
  const std::string urdfFile = declareStringParam("urdfFile", "");
  const std::string taskFile = declareStringParam("taskFile", "");
  const std::string referenceFile = declareStringParam("referenceFile", "");

  // Opti-Pessi OCP config files -- separate from the legged taskFile above: this is
  // opti_pessi_interface's own config/task.info + config/scenario_S*.info.
  const std::string optiPessiTaskFile = declareStringParam("optiPessiTaskFile", "");
  const std::string optiPessiScenarioFile = declareStringParam("optiPessiScenarioFile", "");
  const std::string optiPessiLibraryFolder = declareStringParam("optiPessiLibraryFolder", "/tmp/ocs2/opti_pessi");
  if (!node->has_parameter("optiPessiRecompileLibraries")) {
    node->declare_parameter<bool>("optiPessiRecompileLibraries", false);
  }
  const bool recompileLibraries = node->get_parameter("optiPessiRecompileLibraries").as_bool();

  bool verbose = true;
  loadData::loadCppDataType(taskFile, "legged_robot_interface.verbose", verbose);

  // Helper node for ROS 2 communication: KalmanFilterEstimate needs a spinning executor for its tf
  // listener and odometry subscription, and the control update() thread does not spin one itself.
  ros2_node_ = std::make_shared<rclcpp::Node>("opti_pessi_controller_ros2_node");
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(ros2_node_);
  spinThread_ = std::thread([this]() { executor_->spin(); });

  // Robot model, built directly from the URDF -- the same two factory calls
  // legged_interface::LeggedInterface::setupModel makes internally, without constructing the
  // quadruped's own nonlinear-MPC OptimalControlProblem this controller does not need.
  const auto modelSettings = loadModelSettings(taskFile, "model_settings", verbose);
  pinocchioInterfacePtr_ =
      std::make_unique<PinocchioInterface>(centroidal_model::createPinocchioInterface(urdfFile, modelSettings.jointNames));
  centroidalModelInfo_ = centroidal_model::createCentroidalModelInfo(
      *pinocchioInterfacePtr_, centroidal_model::loadCentroidalType(taskFile),
      centroidal_model::loadDefaultJointState(pinocchioInterfacePtr_->getModel().nq - 6, referenceFile), modelSettings.contactNames3DoF,
      modelSettings.contactNames6DoF);

  CentroidalModelPinocchioMapping pinocchioMapping(centroidalModelInfo_);
  eeKinematicsPtr_ =
      std::make_shared<PinocchioEndEffectorKinematics>(*pinocchioInterfacePtr_, pinocchioMapping, modelSettings.contactNames3DoF);

  rbdConversions_ = std::make_shared<CentroidalModelRbdConversions>(*pinocchioInterfacePtr_, centroidalModelInfo_);

  // State estimation -- same estimator LeggedController uses.
  auto kalmanFilter =
      std::make_shared<KalmanFilterEstimate>(ros2_node_, *pinocchioInterfacePtr_, centroidalModelInfo_, *eeKinematicsPtr_);
  kalmanFilter->loadSettings(taskFile, verbose);
  stateEstimate_ = kalmanFilter;

  // Whole body control
  wbc_ = std::make_shared<WeightedWbc>(*pinocchioInterfacePtr_, centroidalModelInfo_, *eeKinematicsPtr_);
  wbc_->loadTasksSetting(taskFile, verbose);

  // Safety checker
  safetyChecker_ = std::make_shared<SafetyChecker>(centroidalModelInfo_);

  // Opti-Pessi OCP interface. This constructs the full OptimalControlProblem (including CppAD
  // codegen the first time), exactly as LeggedInterface::setupOptimalControlProblem does for the
  // quadruped MPC -- Task 13 is what actually solves it on a background thread; here it is used
  // only for modelParameters() (comHeight, mass, dtCost0, ...) to seed the standing plan below.
  interface_ = std::make_shared<opti_pessi::OptiPessiInterface>(optiPessiTaskFile, optiPessiScenarioFile, optiPessiLibraryFolder,
                                                                 recompileLibraries, verbose);

  interface_->setupOptimalControlProblem(optiPessiLibraryFolder, recompileLibraries);

  // Background solver (Task 13). Built once here from the interface's own settings/problem/
  // initializer -- exactly the pattern OptiPessiInterface.h's class comment documents ("construct
  // an ocs2::IpmSolver from ipmSettings() / getOptimalControlProblem() / getInitializer()"). From
  // this point on solver_ is touched ONLY by solverThread_ (see the header's ownership comment).
  solver_ = std::make_unique<ocs2::IpmSolver>(interface_->ipmSettings(), interface_->getOptimalControlProblem(),
                                              interface_->getInitializer());

  legGeometry_ = opti_pessi_bridge::aliengoLegGeometry();
  synthesisSettings_.comHeight = interface_->modelParameters().comHeight;
  synthesisSettings_.mass = interface_->modelParameters().mass;
  synthesisSettings_.inertia = interface_->modelParameters().inertia;
  synthesisSettings_.gravity = interface_->modelParameters().gravity;
  // swingHeight has no counterpart in OptiPessiModelParameters; SynthesisSettings's own default
  // (0.1 m) is kept as-is.

  centroidalState_ = vector_t::Zero(centroidalModelInfo_.stateDim);
  centroidalInput_ = vector_t::Zero(centroidalModelInfo_.inputDim);

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

  // --- Hardware handle binding -----------------------------------------------------------------
  // Copied from LeggedController::on_activate verbatim: ros2_control hardware-interface plumbing,
  // not controller-specific design (see the RULING in task-12-brief.md and the header comment on
  // the handle structs above).
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
        !joint_handle.positionCmd || !joint_handle.velocityCmd || !joint_handle.kpCmd || !joint_handle.kdCmd ||
        !joint_handle.effortCmd) {
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

  // --- Opti-Pessi wiring -------------------------------------------------------------------------

  // Seed the measured centroidal state/input once so the standing plan below folds real sensor
  // data rather than zeros.
  updateStateEstimation(node->get_clock()->now(), rclcpp::Duration::from_seconds(0.002));

  // A standing plan: feet where they are, no motion, nominal phase duration. This lets the
  // controller hold a stance and drive the WBC before any solve has landed. See
  // task-12-report.md for a caveat: with no real previous-phase history, the "swing" pair's
  // take-off/landing points both collapse to the CURRENT stance pair's coordinates (StateFolder's
  // documented cold-start fallback), which is a placeholder for OCP warm-starting, not a faithful
  // "this is where the other two feet actually are" -- see the report for the consequence.
  history_ = opti_pessi_bridge::FoldedHistory{};
  opti_pessi_bridge::PhasePlan standing;
  standing.robotState =
      opti_pessi_bridge::foldToLipState(legGeometry_, centroidalState_, centroidalInput_, 0, opti_pessi_bridge::FoldedHistory{});
  standing.robotInput = vector_t::Zero(opti_pessi::RobotU::DIM);
  standing.robotInput(opti_pessi::RobotU::ALPHA) = 0.5;
  standing.robotInput(opti_pessi::RobotU::BETA) = 0.5;
  standing.robotInput(opti_pessi::RobotU::GAMMA) = 0.5;
  standing.robotInput(opti_pessi::RobotU::DT) = interface_->modelParameters().dtCost0;
  // Land the swing feet exactly where the (placeholder) previous footholds already are: no
  // stepping.
  standing.robotInput.segment(opti_pessi::RobotU::P0X, 2) = standing.robotState.segment(opti_pessi::RobotX::P0X, 2);
  standing.robotInput.segment(opti_pessi::RobotU::P1X, 2) = standing.robotState.segment(opti_pessi::RobotX::P1X, 2);
  standing.phaseIndex = 0;
  standing.valid = true;

  planBuffer_.setCurrent(standing);
  phaseClock_.reset(standing.robotInput(opti_pessi::RobotU::DT));

  // Start the background solver thread. solverRunning_ is set before the thread is created, so the
  // very first thing solverThreadLoop() observes when it takes the lock is a consistent state; no
  // request is pending yet (solveRequested_ was left false from construction / the previous
  // deactivate), so the thread simply blocks on the condition variable until update() posts one.
  solveRequested_ = false;
  solverRunning_ = true;
  solverThread_ = std::thread(&OptiPessiController::solverThreadLoop, this);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn OptiPessiController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
  stopSolverThread();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type OptiPessiController::update(const rclcpp::Time& time, const rclcpp::Duration& period) {
  updateStateEstimation(time, period);

  // Advance the phase clock; on a boundary, latch a pending plan if one is ready.
  const bool crossedBoundary = phaseClock_.advance(period.seconds());
  if (crossedBoundary) {
    // Carry this phase's stance feet and pose into the history the next solve needs.
    const auto currentPlan = planBuffer_.current();
    if (currentPlan.valid) {
      history_.previousFoot0 = currentPlan.robotState.segment(opti_pessi::RobotX::P0X, 2);
      history_.previousFoot1 = currentPlan.robotState.segment(opti_pessi::RobotX::P1X, 2);
      history_.previousCom = currentPlan.robotState.segment(opti_pessi::RobotX::CX, 2);
      history_.previousYaw = currentPlan.robotState(opti_pessi::RobotX::TH);
      history_.valid = true;
    }

    // Request a solve for the phase that is starting now. The control thread NEVER blocks here:
    // this only copies a 17-vector under solveInputMutex_ (see the header's ownership comment) and
    // notifies the solver thread, which picks it up whenever it next finishes its current solve --
    // mean solve time is ~1s against a ~0.2-0.25s phase, so most requests are overwritten by a later
    // one before the solver thread ever looks at them, and that is fine: only the latest matters.
    {
      std::lock_guard<std::mutex> lock(solveInputMutex_);
      solveInitialState_ = opti_pessi_bridge::foldToLipState(legGeometry_, centroidalState_, centroidalInput_,
                                                             phaseClock_.parity(), history_);
      solvePhaseIndex_ = phaseClock_.phaseIndex();
      solveRequested_ = true;
    }
    solveCv_.notify_one();

    if (planBuffer_.hasPending()) {
      auto next = planBuffer_.takePending();
      // A plan more than one phase old describes a robot that has already moved on. Executing it is
      // worse than holding the current phase's plan, so drop it.
      if (next.phaseIndex >= phaseClock_.phaseIndex() - 1) {
        planBuffer_.setCurrent(next);
        phaseClock_.setPhaseDuration(next.robotInput(opti_pessi::RobotU::DT));
      } else {
        RCLCPP_WARN(get_node()->get_logger(), "opti_pessi bridge: dropped a stale plan (phase %d, now %d)",
                    next.phaseIndex, phaseClock_.phaseIndex());
      }
    }
  }

  const auto plan = planBuffer_.current();
  if (!plan.valid) {
    return controller_interface::return_type::OK;  // nothing to track yet
  }

  const auto reference =
      opti_pessi_bridge::synthesize(legGeometry_, plan, phaseClock_.tau(), phaseClock_.parity(), synthesisSettings_);
  if (!reference.allFeetReachable) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                         "opti_pessi bridge: foot target outside the leg workspace, clamped");
  }

  const vector_t x = wbc_->update(reference.state, reference.input, measuredRbdState_, reference.mode, period.seconds());

  // Safety check is run against the MEASURED state (not the reference), same convention as
  // LeggedController.
  SystemObservation measuredObservation;
  measuredObservation.time = time.seconds();
  measuredObservation.state = centroidalState_;
  measuredObservation.input = centroidalInput_;

  if (!writeHardwareCommand(hybridJointHandles_, centroidalModelInfo_, *safetyChecker_, measuredObservation, reference.state,
                            reference.input, x, this->get_node()->get_logger(), "[OptiPessi Controller] Safety check failed!")) {
    return controller_interface::return_type::ERROR;
  }

  return controller_interface::return_type::OK;
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

  const scalar_t yawLast = centroidalState_.size() > 9 ? centroidalState_(9) : 0.0;
  centroidalState_ = rbdConversions_->computeCentroidalStateFromRbdModel(measuredRbdState_);
  centroidalState_(9) = yawLast + angles::shortest_angular_distance(yawLast, centroidalState_(9));
}

void OptiPessiController::solverThreadLoop() {
  while (true) {
    ocs2::vector_t initialState;
    int phaseIndex = 0;
    {
      std::unique_lock<std::mutex> lock(solveInputMutex_);
      // Predicate form: solveCv_.wait re-checks this itself, so a notify_one()/notify_all() that
      // lands between stopSolverThread()'s lock_guard releasing and this wait() starting is not
      // missed -- the predicate is re-evaluated under the lock before ever blocking.
      solveCv_.wait(lock, [this] { return solveRequested_.load() || !solverRunning_.load(); });
      if (!solverRunning_.load()) {
        break;
      }
      initialState = solveInitialState_;
      phaseIndex = solvePhaseIndex_;
      solveRequested_ = false;
    }  // Lock released here. The solve below -- the only slow part of this loop -- runs unlocked.

    const opti_pessi_bridge::PhasePlan plan = solveOnePhase(initialState, phaseIndex);
    planBuffer_.publish(plan);  // solverThread_ is PlanBuffer's sole writer; see PlanBuffer.h.
  }
}

opti_pessi_bridge::PhasePlan OptiPessiController::solveOnePhase(const ocs2::vector_t& initialState, int phaseIndex) {
  const auto& params = interface_->modelParameters();
  const auto referenceManagerPtr = interface_->getOptiPessiReferenceManagerPtr();

  // Scenario obstacles are static config for this task (no live obstacle perception exists yet in
  // this controller); goal is likewise the configured scenario goal. Both are pushed every solve
  // for consistency with getOptiPessiReferenceManagerPtr()'s documented per-iteration contract,
  // even though today they never change between solves.
  referenceManagerPtr->setGaitOffset(phaseIndex);
  referenceManagerPtr->setObstacles(params.obstaclePositions);
  referenceManagerPtr->setGoal(params.goal);
  referenceManagerPtr->setPessiScale(1.0);  // nominal keep-out; the continuation retry is not ported, see above

  const ocs2::vector_t augmentedInitialState = opti_pessi::packInitialState(initialState);
  const scalar_t finalTime = interface_->finalTime();

  SolveOutcome outcome;
  try {
    solver_->reset();
    solver_->run(0.0, augmentedInitialState, finalTime);
    outcome = extractSolve(*solver_, interface_->getOptimalControlProblem(), params, initialState, finalTime);
  } catch (const std::exception& e) {
    RCLCPP_WARN(get_node()->get_logger(), "opti_pessi bridge: solve for phase %d threw: %s", phaseIndex, e.what());
    // outcome stays default-constructed (ok=false, haveAppliedInput=false): falls through to the
    // capture-point fallback below, exactly as a solve that ran but produced nothing usable would.
  }

  ocs2::vector_t appliedInput;
  if (outcome.ok) {
    appliedInput = outcome.appliedInput;
  } else {
    // Prefer the failed solve's own first input, saturated to bounds (what the reference does: a
    // failed iterate is usually still informative) before giving up to the capture-point stop.
    bool usedSaturatedSolve = false;
    if (outcome.haveAppliedInput && outcome.appliedInput.size() == opti_pessi::RobotU::DIM &&
        outcome.appliedInput(opti_pessi::RobotU::DT) > 0.0) {
      const ocs2::vector_t candidate = saturateRobotInput(outcome.appliedInput, params);
      const ocs2::vector_t candidateSuccessor =
          opti_pessi::lipMapScalar(initialState, candidate, params.omega(), params.mass, params.inertia);
      if (!isInsane(candidateSuccessor, candidate, params)) {
        appliedInput = candidate;
        usedSaturatedSolve = true;
      }
    }
    if (!usedSaturatedSolve) {
      appliedInput = fallbackInput(params, initialState, phaseIndex);
    }
    RCLCPP_WARN(get_node()->get_logger(), "opti_pessi bridge: solve for phase %d failed the sanity gate, using %s fallback",
               phaseIndex, usedSaturatedSolve ? "saturated-solve" : "capture-point");
  }

  opti_pessi_bridge::PhasePlan plan;
  plan.robotState = initialState;
  plan.robotInput = appliedInput;
  plan.phaseIndex = phaseIndex;
  plan.valid = true;
  return plan;
}

void OptiPessiController::stopSolverThread() {
  {
    std::lock_guard<std::mutex> lock(solveInputMutex_);
    solverRunning_ = false;
  }
  solveCv_.notify_all();
  if (solverThread_.joinable()) {
    solverThread_.join();
  }
}

OptiPessiController::~OptiPessiController() {
  // Defensive: on_deactivate() already does this on the normal path, but a controller can be
  // destroyed without ever having been deactivated (e.g. it never activated at all, or the manager
  // tears it down straight from an active state). stopSolverThread() is idempotent -- joinable() is
  // false after the first join(), so a second call here is a no-op -- so calling it unconditionally
  // is always safe and is the only thing standing between us and a leaked/detached thread crashing
  // the process on unload.
  stopSolverThread();

  if (executor_) {
    executor_->cancel();
  }
  if (spinThread_.joinable()) {
    spinThread_.join();
  }
}

}  // namespace legged

PLUGINLIB_EXPORT_CLASS(legged::OptiPessiController, controller_interface::ControllerInterface)
