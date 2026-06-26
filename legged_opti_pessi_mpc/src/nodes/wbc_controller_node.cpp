/**
 * @file wbc_controller_node.cpp
 * @brief Method implementations for WbcControllerNode.
 */

#include "opti_pessi_control/nodes/WbcControllerNode.h"

// =============================================================================
//  go2_kinematics::analyticalIK  (definition)
// =============================================================================

bool go2_kinematics::analyticalIK(double foot_x, double foot_y, double foot_z,
                                  int mirror, double &q_hip, double &q_thigh,
                                  double &q_calf) {
  double y_eff = foot_y - mirror * HIP_OFFSET;
  q_hip = std::atan2(y_eff, -foot_z);

  double dz = -std::sqrt(
      std::max(y_eff * y_eff + foot_z * foot_z - HIP_OFFSET * HIP_OFFSET, 0.0));
  double L = std::sqrt(foot_x * foot_x + dz * dz);
  L = std::clamp(L, std::abs(THIGH_LENGTH - CALF_LENGTH) + 0.001,
                 THIGH_LENGTH + CALF_LENGTH - 0.001);

  double cos_calf =
      (THIGH_LENGTH * THIGH_LENGTH + CALF_LENGTH * CALF_LENGTH - L * L) /
      (2.0 * THIGH_LENGTH * CALF_LENGTH);
  q_calf = -(M_PI - std::acos(std::clamp(cos_calf, -1.0, 1.0)));

  double alpha = std::atan2(-foot_x, -dz);
  double cos_beta =
      (THIGH_LENGTH * THIGH_LENGTH + L * L - CALF_LENGTH * CALF_LENGTH) /
      (2.0 * THIGH_LENGTH * L);
  q_thigh = alpha + std::acos(std::clamp(cos_beta, -1.0, 1.0));

  q_hip = std::clamp(q_hip, HIP_MIN, HIP_MAX);
  q_thigh = std::clamp(q_thigh, THIGH_MIN, THIGH_MAX);
  q_calf = std::clamp(q_calf, CALF_MIN, CALF_MAX);
  return true;
}

// =============================================================================
//  WbcControllerNode — constructor / destructor
// =============================================================================

WbcControllerNode::WbcControllerNode() : Node("wbc_controller") {
  // ---- ROS parameters ----
  kp_ = declare_parameter("kp", 60.0);
  kd_ = declare_parameter("kd", 5.0);
  swing_height_ = declare_parameter("swing_height", 0.10);
  log_file_path_ = declare_parameter(
      "log_file_path", "/home/abarbi/go2_ws/wbc_trajectory_log/wbc_trajectory_log.csv");

  // ---- Logging ----
  log_file_.open(log_file_path_, std::ios::out | std::ios::trunc);
  if (log_file_.is_open()) {
    RCLCPP_INFO(get_logger(), "Logging WBC data to %s", log_file_path_.c_str());
    writeCSVHeader();
  } else {
    RCLCPP_WARN(get_logger(), "Could not open log file: %s",
                log_file_path_.c_str());
  }

  // ---- Initial joint state (standing pose) ----
  for (int leg = 0; leg < 4; ++leg) {
    const int b = leg * 3;
    target_joint_pos_[b] = prev_target_joint_pos_[b] = 0.0;
    target_joint_pos_[b + 1] = prev_target_joint_pos_[b + 1] = 0.67;
    target_joint_pos_[b + 2] = prev_target_joint_pos_[b + 2] = -1.3;
  }

  // ---- Build Pinocchio interface & WBC ----
  const std::string taskFile =
      "/home/abarbi/RaNAV/src/legged_control/legged_opti_pessi_mpc/config/task.info";
  const std::string urdfFile = "/home/abarbi/go2_ws/src/go2_control/"
                               "go2_description/urdf/go2_description.urdf";
  const std::string referenceFile = "/home/abarbi/RaNAV/src/legged_control/"
                                    "legged_opti_pessi_mpc/config/reference.info";

  const std::vector<std::string> jointNames = {
      "FL_hip_joint",   "FL_thigh_joint", "FL_calf_joint",  "FR_hip_joint",
      "FR_thigh_joint", "FR_calf_joint",  "RL_hip_joint",   "RL_thigh_joint",
      "RL_calf_joint",  "RR_hip_joint",   "RR_thigh_joint", "RR_calf_joint"};
  const std::vector<std::string> contactNames = {"FL_foot", "FR_foot",
                                                 "RL_foot", "RR_foot"};

  try {
    auto pinocchioInterface = std::make_unique<ocs2::PinocchioInterface>(
        ocs2::centroidal_model::createPinocchioInterface(urdfFile, jointNames));

    centroidalModelInfo_ = ocs2::centroidal_model::createCentroidalModelInfo(
        *pinocchioInterface,
        ocs2::centroidal_model::loadCentroidalType(taskFile),
        ocs2::centroidal_model::loadDefaultJointState(12, referenceFile),
        contactNames, {});

    ocs2::CentroidalModelPinocchioMapping pinocchioMapping(
        centroidalModelInfo_);
    auto eeKinematics = std::make_unique<ocs2::PinocchioEndEffectorKinematics>(
        *pinocchioInterface, pinocchioMapping, contactNames);

    wbc_ = std::make_unique<legged::OptiPessiWbc>(
        *pinocchioInterface, centroidalModelInfo_, *eeKinematics);
    wbc_->loadTasksSetting(taskFile, true);

    RCLCPP_INFO(get_logger(), "OptiPessiWbc initialised successfully.");
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_logger(), "WBC init failed: %s", e.what());
  }

  // ---- Gait pattern: dynamic_walk (hardcoded from config/gait.info) ----
  // OCS2 mode names encode the STANCE legs: LF=FL, RF=FR, LH=RL, RH=RR
  // Unitree bitmask: FL=bit0, FR=bit1, RL=bit2, RR=bit3 (bit SET = stance)
  //   [0] LF_RF_RH  →  FL+FR+RR stance  →  0b1011 = 11  (RL swings)
  //   [1] RF_LH_RH  →  FR+RL+RR stance  →  0b1110 = 14  (FL swings)
  //   [2] LF_RF_LH  →  FL+FR+RL stance  →  0b0111 = 7   (RR swings)
  //   [3] LF_LH_RH  →  FL+RL+RR stance  →  0b1101 = 13  (FR swings)
  // switchingTimes: 0.0 → 0.20 → 0.40 → 0.60 → 0.80 s
  gait_schedule_.mode_sequence = {11, 14, 7, 13};
  gait_schedule_.phase_duration = 0.20; // s per phase
  gait_schedule_.cycle_period = 0.80;   // T = 4 × 0.20 s

  // ---- Subscriptions ----
  policy_sub_ = create_subscription<ocs2_msgs::msg::MpcFlattenedController>(
      "/legged_robot_mpc_policy", 10,
      std::bind(&WbcControllerNode::policyCallback, this,
                std::placeholders::_1));

  lowstate_sub_ = create_subscription<unitree_go::msg::LowState>(
      "/lowstate", 10,
      std::bind(&WbcControllerNode::lowstateCallback, this,
                std::placeholders::_1));

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&WbcControllerNode::odomCallback, this, std::placeholders::_1));

  robot_mode_sub_ = create_subscription<std_msgs::msg::String>(
      "/robot_mode", 10,
      std::bind(&WbcControllerNode::robotModeCallback, this,
                std::placeholders::_1));

  kp_kd_sub_ = create_subscription<geometry_msgs::msg::Point>(
      "/joint_kp_kd", 10,
      std::bind(&WbcControllerNode::kpKdCallback, this, std::placeholders::_1));

  // ---- Publisher ----
  lowcmd_pub_ = create_publisher<unitree_go::msg::LowCmd>("/lowcmd", 10);

  // ---- Control loop at 500 Hz ----
  control_timer_ =
      create_wall_timer(std::chrono::microseconds(2000),
                        std::bind(&WbcControllerNode::controlLoop, this));

  RCLCPP_INFO(get_logger(), "WBC Controller ready (500 Hz, kp=%.1f, kd=%.1f).",
              kp_, kd_);
}

WbcControllerNode::~WbcControllerNode() {
  if (log_file_.is_open())
    log_file_.close();
}

// =============================================================================
//  Callbacks
// =============================================================================

void WbcControllerNode::kpKdCallback(
    const geometry_msgs::msg::Point::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(gains_mutex_);
  kp_ = msg->x;
  kd_ = msg->y;
}

void WbcControllerNode::robotModeCallback(
    const std_msgs::msg::String::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(robot_mode_mutex_);
  const std::string &m = msg->data;
  if (m == "lie")
    robot_mode_ = RobotMode::LIE;
  else if (m == "stand")
    robot_mode_ = RobotMode::STAND;
  else if (m == "walk")
    robot_mode_ = RobotMode::WALK;
  else
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "robot_mode sconosciuto: '%s'", m.c_str());
}

void WbcControllerNode::policyCallback(
    const ocs2_msgs::msg::MpcFlattenedController::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(policy_mutex_);
  policy_received_ = true;
  policy_time_ = get_clock()->now().seconds();
  time_trajectory_ = msg->time_trajectory;

  state_trajectory_.clear();
  for (const auto &s : msg->state_trajectory)
    state_trajectory_.push_back(s.value);

  input_trajectory_.clear();
  for (const auto &u : msg->input_trajectory)
    input_trajectory_.push_back(u.value);

  mode_event_times_.clear();
  mode_sequence_.clear();
  for (double t : msg->mode_schedule.event_times)
    mode_event_times_.push_back(t);
  for (int m : msg->mode_schedule.mode_sequence)
    mode_sequence_.push_back(m);
}

void WbcControllerNode::lowstateCallback(
    const unitree_go::msg::LowState::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!state_received_) {
    RCLCPP_INFO(get_logger(), "First robot state received. Matching target "
                              "positions to actual joint angles.");
    for (int i = 0; i < 12; ++i) {
      target_joint_pos_[i] = msg->motor_state[i].q;
      prev_target_joint_pos_[i] = msg->motor_state[i].q;
    }
  }
  state_received_ = true;
  for (int i = 0; i < 12; ++i) {
    current_joint_pos_[i] = msg->motor_state[i].q;
    current_joint_vel_[i] = msg->motor_state[i].dq;
  }
  gyroscope_[0] = msg->imu_state.gyroscope[0];
  gyroscope_[1] = msg->imu_state.gyroscope[1];
  gyroscope_[2] = msg->imu_state.gyroscope[2];
  rpy_[0] = msg->imu_state.rpy[0];
  rpy_[1] = msg->imu_state.rpy[1];
  rpy_[2] = msg->imu_state.rpy[2];
}

void WbcControllerNode::odomCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  odom_x_ = msg->pose.pose.position.x;
  odom_y_ = msg->pose.pose.position.y;

  // /odom pubblica posizione relativa al frame iniziale (z ≈ 0).
  // Aggiungiamo l'altezza del base link da terra nella standing pose.
  // NOMINAL_HEIGHT è l'altezza CoM; il base link è circa alla stessa quota
  // ma verifica dal URDF la trasformazione base_link → CoM se differisce.
  constexpr double BASE_HEIGHT_STANDING = 0.32; // metri — adatta se necessario
  odom_z_ = msg->pose.pose.position.z + BASE_HEIGHT_STANDING;

  odom_vx_ = msg->twist.twist.linear.x;
  odom_vy_ = msg->twist.twist.linear.y;
  odom_vz_ = msg->twist.twist.linear.z;

  const double qx = msg->pose.pose.orientation.x;
  const double qy = msg->pose.pose.orientation.y;
  const double qz = msg->pose.pose.orientation.z;
  const double qw = msg->pose.pose.orientation.w;
  odom_yaw_ =
      std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
}

// =============================================================================
//  Main control loop (500 Hz)
// =============================================================================

void WbcControllerNode::controlLoop() {
  if (!wbc_)
    return;

  double kp, kd;
  {
    std::lock_guard<std::mutex> lock(gains_mutex_);
    kp = kp_;
    kd = kd_;
  }

  // Wait until we have joint states
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!state_received_)
      return;
  }

  // Initialise foot world-frame targets from current base pose once
  if (!foot_targets_initialized_) {
    initFootTargets();
    foot_targets_initialized_ = true;
  }

  // Read current mode
  RobotMode current_mode;
  {
    std::lock_guard<std::mutex> lock(robot_mode_mutex_);
    current_mode = robot_mode_;
  }

  // ------------------------------------------------------------------
  // Hardcoded target joint angles for LIE and STAND
  // ------------------------------------------------------------------
  static const double LIE_POSE[12] = {
      0.0, 1.36, -2.65, // FL
      0.0, 1.36, -2.65, // FR
      0.0, 1.36, -2.65, // RL
      0.0, 1.36, -2.65  // RR
  };
  static const double STAND_POSE[12] = {
      0.0, 0.67, -1.3, // FL
      0.0, 0.67, -1.3, // FR
      0.0, 0.67, -1.3, // RL
      0.0, 0.67, -1.3  // RR
  };
  constexpr double INTERP_STEP = 0.002; // rad/tick @ 500 Hz
  constexpr size_t kAllStanceMode = 15; // bitmask 1111

  // ------------------------------------------------------------------
  // Per-mode setup: fill DesiredState, target_joints, swingPos/Vel, wbc_mode
  // ------------------------------------------------------------------
  DesiredState des{};
  double target_joints[12] = {};
  std::vector<legged::vector3_t> swingPos(4, legged::vector3_t::Zero());
  std::vector<legged::vector3_t> swingVel(4, legged::vector3_t::Zero());
  size_t wbc_mode = kAllStanceMode;
  bool policy_active = false;
  double t_query = 0.0;

  if (current_mode == RobotMode::LIE || current_mode == RobotMode::STAND) {
    // ---- Hardcoded pose mode ----------------------------------------
    const double *target_pose =
        (current_mode == RobotMode::LIE) ? LIE_POSE : STAND_POSE;

    // Interpolate joint targets smoothly toward the hardcoded pose
    for (int j = 0; j < 12; ++j) {
      const double diff = target_pose[j] - target_joint_pos_[j];
      if (std::abs(diff) > INTERP_STEP)
        target_joint_pos_[j] += std::copysign(INTERP_STEP, diff);
      else
        target_joint_pos_[j] = target_pose[j];
      target_joints[j] = target_joint_pos_[j];
    }

    // Reset foot tracking and gait phase so that switching to WALK starts
    // cleanly
    foot_targets_initialized_ = false;
    gait_start_time_ = -1.0;
    for (int i = 0; i < 4; ++i) {
      first_loop_[i] = true;
      was_in_stance_[i] = true;
    }

    // Bypass WBC for LIE/STAND: compute closed-loop PD + gravity compensation
    // control
    {
      // Build measured state securely under state_mutex_
      ocs2::vector_t rbdStateMeasured;
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        rbdStateMeasured = buildMeasuredRbdState();
      }

      // Compute gravity compensation using the robot model
      ocs2::vector_t gravity_comp =
          wbc_->getGravityCompensation(rbdStateMeasured);

      double torques[12] = {};
      double joint_vel_cmd[12] = {};
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        for (int i = 0; i < 12; ++i) {
          // Closed-loop PD control: tau = kp*(q_des - q_meas) + kd*(dq_des -
          // dq_meas) + tau_gravity q_des = target_joint_pos_, dq_des = 0.0
          torques[i] = kp * (target_joint_pos_[i] - current_joint_pos_[i]) +
                       kd * (0.0 - current_joint_vel_[i]) + gravity_comp[i];

          // Compute joint velocity command for logging / bridge interpolation
          joint_vel_cmd[i] =
              std::clamp((target_joint_pos_[i] - prev_target_joint_pos_[i]) /
                             kControlPeriod_,
                         -10.0, 10.0);
          prev_target_joint_pos_[i] = target_joint_pos_[i];
        }
      }

      // Publish torque command with zero kp/kd parameters to avoid double-PD
      // action
      publishLowCmd(target_joint_pos_, joint_vel_cmd, torques, 0.0, 0.0);
      logToCSV(false, 0.0, kAllStanceMode, des, target_joints, joint_vel_cmd,
               torques);
    }
    return;

  } else {
    // ---- WALK mode: follow MPC trajectory --------------------------

    // Initialise gait timing on first entry into WALK mode
    if (gait_start_time_ < 0.0) {
      gait_start_time_ = get_clock()->now().seconds();
    }

    // Snapshot policy data
    double t_start, t_end;
    std::vector<double> event_times;
    std::vector<int> mode_seq;
    {
      std::lock_guard<std::mutex> lock(policy_mutex_);
      if (time_trajectory_.empty()) {
        // No policy yet — fall back to STAND pose silently
        for (int j = 0; j < 12; ++j) {
          const double diff = STAND_POSE[j] - target_joint_pos_[j];
          if (std::abs(diff) > INTERP_STEP)
            target_joint_pos_[j] += std::copysign(INTERP_STEP, diff);
          else
            target_joint_pos_[j] = STAND_POSE[j];
          target_joints[j] = target_joint_pos_[j];
        }
        wbc_mode = kAllStanceMode;
        policy_active = false;
        goto run_wbc; // skip trajectory setup, go straight to WBC
      }
      const double t_since = get_clock()->now().seconds() - policy_time_;
      t_query = time_trajectory_.front() + t_since;
      t_start = time_trajectory_.front();
      t_end = time_trajectory_.back();
      if (t_query >= t_end || t_since > 0.5) {
        // Stale policy — fall back to STAND
        for (int j = 0; j < 12; ++j) {
          const double diff = STAND_POSE[j] - target_joint_pos_[j];
          if (std::abs(diff) > INTERP_STEP)
            target_joint_pos_[j] += std::copysign(INTERP_STEP, diff);
          else
            target_joint_pos_[j] = STAND_POSE[j];
          target_joints[j] = target_joint_pos_[j];
        }
        wbc_mode = kAllStanceMode;
        policy_active = false;
        goto run_wbc;
      }
    }

    // Build mode schedule from the gait.info dynamic_walk pattern
    computeGaitModeSchedule(t_start, t_end, event_times, mode_seq);

    // Interpolate desired CoM state and foothold inputs at t_query
    des = interpolateTrajectory(t_query);

    // Current base pose snapshot
    double base_x, base_y, base_z, base_yaw;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      base_x = odom_x_;
      base_y = odom_y_;
      base_z = odom_z_;
      base_yaw = odom_yaw_;
    }

    // Compute per-leg swing/stance foot targets and run IK
    for (int leg = 0; leg < 4; ++leg) {
      updateFootTarget(leg, t_query, t_start, t_end, event_times, mode_seq,
                       des);
      swingPos[leg] = {foot_target_x_[leg], foot_target_y_[leg],
                       foot_target_z_[leg]};
      swingVel[leg] = {foot_vel_x_[leg], foot_vel_y_[leg], foot_vel_z_[leg]};
      runIK(leg, base_x, base_y, base_z, base_yaw, target_joints);
    }

    // Rate-limit joint position steps (max 20 rad/s @ 500 Hz = 0.04 rad/tick)
    constexpr double MAX_STEP = 0.04;
    for (int i = 0; i < 12; ++i) {
      const double diff = target_joints[i] - target_joint_pos_[i];
      target_joint_pos_[i] +=
          std::abs(diff) > MAX_STEP ? std::copysign(MAX_STEP, diff) : diff;
    }

    wbc_mode = findActiveMode(t_query, t_start, t_end, event_times, mode_seq);
    policy_active = true;
  }

run_wbc:
  // ------------------------------------------------------------------
  // Common WBC call: compute pos, vel, torque for all modes
  // ------------------------------------------------------------------
  {
    ocs2::vector_t mpcInput(5);
    mpcInput << des.p0_x, des.p0_y, des.p1_x, des.p1_y, des.alpha;

    ocs2::vector_t stateDesired = buildDesiredState(des);
    ocs2::vector_t inputDesired = ocs2::vector_t::Zero(24);

    wbc_->setMpcInput(mpcInput);
    wbc_->setSwingFootTargets(swingPos, swingVel);
    ocs2::vector_t wbcSol =
        wbc_->update(stateDesired, inputDesired, buildMeasuredRbdState(),
                     wbc_mode, kControlPeriod_);

    const int n = static_cast<int>(wbcSol.size());
    double torques[12] = {0.0};
    double joint_vel_cmd[12] = {0.0};

    if (n >= 12) {
      for (int i = 0; i < 12; ++i)
        torques[i] = wbcSol[n - 12 + i];
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "wbcSol size=%d < 12, fallback a torque zero", n);
    }

    for (int i = 0; i < 12; ++i) {
      joint_vel_cmd[i] = std::clamp(
          (target_joint_pos_[i] - prev_target_joint_pos_[i]) / kControlPeriod_,
          -10.0, 10.0);
      prev_target_joint_pos_[i] = target_joint_pos_[i];
    }

    publishLowCmd(target_joint_pos_, joint_vel_cmd, torques);
    logToCSV(policy_active, t_query, wbc_mode, des, target_joints,
             joint_vel_cmd, torques);
  }
}

// =============================================================================
//  Trajectory interpolation
// =============================================================================

WbcControllerNode::DesiredState
WbcControllerNode::interpolateTrajectory(double t_query) {
  DesiredState des;
  std::lock_guard<std::mutex> lock(policy_mutex_);

  if (state_trajectory_.empty() || time_trajectory_.empty() ||
      input_trajectory_.empty())
    return des;

  // Find bracket index
  size_t idx = 0;
  for (size_t i = 0; i + 1 < time_trajectory_.size(); ++i) {
    if (t_query < time_trajectory_[i + 1]) {
      idx = i;
      break;
    }
    idx = i;
  }
  const size_t idx1 = std::min(idx + 1, state_trajectory_.size() - 1);
  const double dt = time_trajectory_[idx1] - time_trajectory_[idx];
  const double a =
      (dt > 1e-6) ? std::clamp((t_query - time_trajectory_[idx]) / dt, 0.0, 1.0)
                  : 0.0;

  // State interpolation (24-dim centroidal state)
  const auto &s0 = state_trajectory_[idx];
  const auto &s1 = state_trajectory_[idx1];
  auto lerp_s = [&](size_t i) {
    return (double)s0[i] * (1 - a) + (double)s1[i] * a;
  };
  if (s0.size() >= 12) {
    des.vx = lerp_s(3);
    des.vy = lerp_s(4);
    des.yaw = lerp_s(6);
    des.com_x = lerp_s(9);
    des.com_y = lerp_s(10);
  }

  // Input interpolation (foothold params: [p0_x, p0_y, p1_x, p1_y, alpha, ...])
  const auto &u0 = input_trajectory_[idx];
  const auto &u1 = input_trajectory_[idx1];
  auto lerp_u = [&](size_t i) {
    return (double)u0[i] * (1 - a) + (double)u1[i] * a;
  };
  if (u0.size() >= 5) {
    des.p0_x = lerp_u(0);
    des.p0_y = lerp_u(1);
    des.p1_x = lerp_u(2);
    des.p1_y = lerp_u(3);
    des.alpha = lerp_u(4);
  }
  return des;
}

// =============================================================================
//  Foot target tracking
// =============================================================================

size_t
WbcControllerNode::findActiveMode(double t_query, double t_start, double t_end,
                                  const std::vector<double> &event_times,
                                  const std::vector<int> &mode_seq) const {
  if (mode_seq.empty())
    return 15; // all-stance fallback
  std::vector<double> times;
  times.push_back(t_start);
  times.insert(times.end(), event_times.begin(), event_times.end());
  times.push_back(t_end);
  size_t idx = 0;
  for (size_t i = 0; i + 1 < times.size(); ++i) {
    if (t_query < times[i + 1]) {
      idx = i;
      break;
    }
    idx = i;
  }
  return static_cast<size_t>(mode_seq[std::min(idx, mode_seq.size() - 1)]);
}

WbcControllerNode::SwingInfo
WbcControllerNode::computeSwingInfo(int leg, double t_query, double t_start,
                                    double t_end,
                                    const std::vector<double> &event_times,
                                    const std::vector<int> &mode_seq) const {
  SwingInfo info;
  if (mode_seq.empty())
    return info;

  std::vector<double> times;
  times.push_back(t_start);
  times.insert(times.end(), event_times.begin(), event_times.end());
  times.push_back(t_end);

  // Find current bracket
  size_t idx = 0;
  for (size_t i = 0; i + 1 < times.size(); ++i) {
    if (t_query < times[i + 1]) {
      idx = i;
      break;
    }
    idx = i;
  }
  const int cur_mode = mode_seq[std::min(idx, mode_seq.size() - 1)];
  info.in_stance = isStance(leg, cur_mode);

  if (info.in_stance)
    return info;

  // Find swing start (last stance→swing transition)
  double t_swing_start = t_start;
  for (int i = (int)idx; i >= 0; --i) {
    const int m = mode_seq[std::min((size_t)i, mode_seq.size() - 1)];
    if (isStance(leg, m)) {
      t_swing_start = times[i + 1];
      break;
    }
  }

  // Find swing end (next swing→stance transition)
  double t_swing_end = t_end;
  for (size_t i = idx; i < mode_seq.size(); ++i) {
    if (isStance(leg, mode_seq[i])) {
      t_swing_end = times[i];
      break;
    }
  }

  const double dur = t_swing_end - t_swing_start;
  if (dur > 1e-4) {
    info.duration = dur;
    info.phase = std::clamp((t_query - t_swing_start) / dur, 0.0, 1.0);
  }
  return info;
}

//  Gait schedule from gait.info

void WbcControllerNode::computeGaitModeSchedule(
    double t_start, double t_end, std::vector<double> &event_times,
    std::vector<int> &mode_seq) {
  event_times.clear();
  mode_seq.clear();

  const auto &modes = gait_schedule_.mode_sequence;
  const double pd = gait_schedule_.phase_duration;
  const double T = gait_schedule_.cycle_period;
  const int N = static_cast<int>(modes.size());
  if (N == 0 || pd < 1e-6 || T < 1e-6) {
    mode_seq.push_back(15); // fallback: all-stance
    return;
  }

  // Compute the gait phase at this moment relative to when WALK mode started.
  const double elapsed = get_clock()->now().seconds() - gait_start_time_;
  double phase = std::fmod(elapsed, T);
  if (phase < 0.0)
    phase += T;

  // Active phase index and remaining time within the current phase
  int cur_idx = static_cast<int>(phase / pd) % N;
  double remaining = pd - std::fmod(phase, pd);
  if (remaining < 1e-6)
    remaining = pd; // guard against floating-point flush to zero

  // The first mode is active at t_start
  mode_seq.push_back(modes[cur_idx]);

  // Emit event_times for each subsequent phase transition until t_end
  double t_next = t_start + remaining;
  while (t_next < t_end) {
    event_times.push_back(t_next);
    cur_idx = (cur_idx + 1) % N;
    mode_seq.push_back(modes[cur_idx]);
    t_next += pd;
  }
}

void WbcControllerNode::footholdWorld(int leg, double px, double py, double yaw,
                                      double &wx, double &wy) const {
  const auto &cfg = go2_kinematics::LEGS[leg];
  const double lx = cfg.sign_x * go2_kinematics::LEG_OFFSET_X;
  const double ly =
      cfg.mirror * (go2_kinematics::LEG_OFFSET_Y + go2_kinematics::HIP_OFFSET);
  wx = px + lx * std::cos(yaw) - ly * std::sin(yaw);
  wy = py + lx * std::sin(yaw) + ly * std::cos(yaw);
}

void WbcControllerNode::initFootTargets() {
  std::lock_guard<std::mutex> lock(state_mutex_);
  for (int leg = 0; leg < 4; ++leg) {
    const auto &cfg = go2_kinematics::LEGS[leg];
    const double lx = cfg.sign_x * go2_kinematics::LEG_OFFSET_X;
    const double ly = cfg.mirror * (go2_kinematics::LEG_OFFSET_Y +
                                    go2_kinematics::HIP_OFFSET);
    foot_target_x_[leg] =
        odom_x_ + lx * std::cos(odom_yaw_) - ly * std::sin(odom_yaw_);
    foot_target_y_[leg] =
        odom_y_ + lx * std::sin(odom_yaw_) + ly * std::cos(odom_yaw_);
    foot_target_z_[leg] = 0.0;
    foot_takeoff_x_[leg] = foot_target_x_[leg];
    foot_takeoff_y_[leg] = foot_target_y_[leg];
  }
}

void WbcControllerNode::updateFootTarget(int leg, double t_query,
                                         double t_start, double t_end,
                                         const std::vector<double> &event_times,
                                         const std::vector<int> &mode_seq,
                                         const DesiredState &des) {
  const SwingInfo sw =
      computeSwingInfo(leg, t_query, t_start, t_end, event_times, mode_seq);
  const double px = (leg <= 1) ? des.p0_x : des.p1_x;
  const double py = (leg <= 1) ? des.p0_y : des.p1_y;

  if (sw.in_stance) {
    // On swing→stance transition update the planted foothold from MPC
    if (!was_in_stance_[leg] && !first_loop_[leg]) {
      footholdWorld(leg, px, py, des.yaw, foot_target_x_[leg],
                    foot_target_y_[leg]);
    }
    first_loop_[leg] = false;
    was_in_stance_[leg] = true;
    foot_target_z_[leg] = 0.0;
    foot_vel_x_[leg] = foot_vel_y_[leg] = foot_vel_z_[leg] = 0.0;
  } else {
    // Record takeoff position at stance→swing transition
    if (was_in_stance_[leg] || first_loop_[leg]) {
      foot_takeoff_x_[leg] = foot_target_x_[leg];
      foot_takeoff_y_[leg] = foot_target_y_[leg];
      first_loop_[leg] = false;
    }
    was_in_stance_[leg] = false;

    // Linear XY interpolation + sine-wave Z lift
    double target_wx, target_wy;
    footholdWorld(leg, px, py, des.yaw, target_wx, target_wy);

    foot_target_x_[leg] =
        (1 - sw.phase) * foot_takeoff_x_[leg] + sw.phase * target_wx;
    foot_target_y_[leg] =
        (1 - sw.phase) * foot_takeoff_y_[leg] + sw.phase * target_wy;
    foot_target_z_[leg] = swing_height_ * std::sin(M_PI * sw.phase);

    const double dphase_dt = (sw.duration > 1e-4) ? 1.0 / sw.duration : 0.0;
    foot_vel_x_[leg] = (target_wx - foot_takeoff_x_[leg]) * dphase_dt;
    foot_vel_y_[leg] = (target_wy - foot_takeoff_y_[leg]) * dphase_dt;
    foot_vel_z_[leg] =
        swing_height_ * M_PI * std::cos(M_PI * sw.phase) * dphase_dt;
  }
}

// =============================================================================
//  Inverse kinematics
// =============================================================================

void WbcControllerNode::runIK(int leg, double base_x, double base_y,
                              double base_z, double base_yaw,
                              double target_joints[12]) const {
  const auto &cfg = go2_kinematics::LEGS[leg];

  const double dx = foot_target_x_[leg] - base_x;
  const double dy = foot_target_y_[leg] - base_y;
  const double fx_base = dx * std::cos(base_yaw) + dy * std::sin(base_yaw);
  const double fy_base = -dx * std::sin(base_yaw) + dy * std::cos(base_yaw);
  const double fx_hip = fx_base - cfg.sign_x * go2_kinematics::LEG_OFFSET_X;
  const double fy_hip = fy_base - cfg.mirror * go2_kinematics::LEG_OFFSET_Y;

  // base_z è odom_z_ che rappresenta il CoM (≈ 0.32 m da terra).
  // Il base link meccanico (origine dell'IK) è COM_TO_BASE_Z metri più in
  // basso.
  const double base_link_z = base_z - go2_kinematics::COM_TO_BASE_Z;
  const double fz_hip = foot_target_z_[leg] - base_link_z;

  double q_hip, q_thigh, q_calf;
  go2_kinematics::analyticalIK(fx_hip, fy_hip, fz_hip, cfg.mirror, q_hip,
                               q_thigh, q_calf);
  target_joints[leg * 3 + 0] = q_hip;
  target_joints[leg * 3 + 1] = q_thigh;
  target_joints[leg * 3 + 2] = q_calf;
}

// =============================================================================
//  WBC state vector builders
// =============================================================================

ocs2::vector_t WbcControllerNode::buildMeasuredRbdState() const {
  ocs2::vector_t s = ocs2::vector_t::Zero(36);
  s[0] = odom_yaw_;
  s[1] = rpy_[1]; // pitch
  s[2] = rpy_[0]; // roll
  s[3] = odom_x_;
  s[4] = odom_y_;
  s[5] = odom_z_;
  for (int i = 0; i < 12; ++i)
    s[6 + i] = current_joint_pos_[i];
  s[18] = gyroscope_[0];
  s[19] = gyroscope_[1];
  s[20] = gyroscope_[2];
  s[21] = odom_vx_;
  s[22] = odom_vy_;
  s[23] = odom_vz_;
  for (int i = 0; i < 12; ++i)
    s[24 + i] = current_joint_vel_[i];
  return s;
}

ocs2::vector_t
WbcControllerNode::buildDesiredState(const DesiredState &des) const {
  const double mass = centroidalModelInfo_.robotMass;
  ocs2::vector_t s = ocs2::vector_t::Zero(24);
  s[0] = des.vx * mass;                   // linear momentum x
  s[1] = des.vy * mass;                   // linear momentum y
  s[6] = des.yaw;                         // desired yaw
  s[9] = des.com_x;                       // CoM x
  s[10] = des.com_y;                      // CoM y
  s[11] = go2_kinematics::NOMINAL_HEIGHT; // CoM z
  for (int i = 0; i < 12; ++i)
    s[12 + i] = target_joint_pos_[i];
  return s;
}

// =============================================================================
//  Low-level hardware publish
// =============================================================================

void WbcControllerNode::publishLowCmd(const double positions[12],
                                      const double velocities[12],
                                      const double torques[12]) {
  double kp, kd;
  {
    std::lock_guard<std::mutex> lock(gains_mutex_);
    kp = kp_;
    kd = kd_;
  }
  publishLowCmd(positions, velocities, torques, kp, kd);
}

void WbcControllerNode::publishLowCmd(const double positions[12],
                                      const double velocities[12],
                                      const double torques[12], double kp,
                                      double kd) {
  unitree_go::msg::LowCmd cmd;
  cmd.head[0] = 0xFE;
  cmd.head[1] = 0xEF;
  cmd.level_flag = 0xFF;
  cmd.gpio = 0;

  for (int i = 0; i < 20; ++i) {
    cmd.motor_cmd[i].mode = 0x01;
    cmd.motor_cmd[i].q = 0.0f;
    cmd.motor_cmd[i].dq = 0.0f;
    cmd.motor_cmd[i].kp = 0.0f;
    cmd.motor_cmd[i].kd = 0.0f;
    cmd.motor_cmd[i].tau = 0.0f;
  }
  for (int i = 0; i < 12; ++i) {
    cmd.motor_cmd[i].q = static_cast<float>(positions[i]);
    cmd.motor_cmd[i].dq = static_cast<float>(velocities[i]);
    cmd.motor_cmd[i].kp = static_cast<float>(kp);
    cmd.motor_cmd[i].kd = static_cast<float>(kd);
    cmd.motor_cmd[i].tau = static_cast<float>(torques[i]);
  }
  lowcmd_pub_->publish(cmd);
}

// =============================================================================
//  CSV Logging
// =============================================================================

void WbcControllerNode::writeCSVHeader() {
  log_file_ << "timestamp,policy_active,t_query,mode,"
            << "des_vx,des_vy,des_yaw,des_com_x,des_com_y,"
            << "odom_x,odom_y,odom_z,odom_yaw,odom_vx,odom_vy,odom_vz,"
            << "gyro_x,gyro_y,gyro_z,rpy_roll,rpy_pitch,rpy_yaw,";
  for (int i = 0; i < 12; ++i)
    log_file_ << "q_meas_" << i << ",";
  for (int i = 0; i < 12; ++i)
    log_file_ << "dq_meas_" << i << ",";
  for (int i = 0; i < 12; ++i)
    log_file_ << "q_ik_" << i << ",";
  for (int i = 0; i < 12; ++i)
    log_file_ << "q_cmd_" << i << ",";
  for (int i = 0; i < 12; ++i)
    log_file_ << "dq_cmd_" << i << ",";
  for (int i = 0; i < 12; ++i)
    log_file_ << "tau_cmd_" << i << ",";
  for (int i = 0; i < 4; ++i)
    log_file_ << "foot_trg_" << i << "_x,"
              << "foot_trg_" << i << "_y,"
              << "foot_trg_" << i << "_z,";
  for (int i = 0; i < 4; ++i)
    log_file_ << "foot_takeoff_" << i << "_x,"
              << "foot_takeoff_" << i << "_y,";
  for (int i = 0; i < 4; ++i) {
    log_file_ << "foot_vel_trg_" << i << "_x,"
              << "foot_vel_trg_" << i << "_y,"
              << "foot_vel_trg_" << i << "_z";
    if (i < 3)
      log_file_ << ",";
  }
  log_file_ << "\n";
  log_file_.flush();
}

void WbcControllerNode::logToCSV(bool policy_active, double t_query,
                                 size_t mode, const DesiredState &des,
                                 const double q_ik[12], const double dq_cmd[12],
                                 const double torques[12]) {
  if (!log_file_.is_open())
    return;

  double odom_x, odom_y, odom_z, odom_yaw, odom_vx, odom_vy, odom_vz;
  double qm[12], dqm[12], gyro[3], rpy[3];
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    odom_x = odom_x_;
    odom_y = odom_y_;
    odom_z = odom_z_;
    odom_yaw = odom_yaw_;
    odom_vx = odom_vx_;
    odom_vy = odom_vy_;
    odom_vz = odom_vz_;
    std::copy(current_joint_pos_, current_joint_pos_ + 12, qm);
    std::copy(current_joint_vel_, current_joint_vel_ + 12, dqm);
    std::copy(gyroscope_, gyroscope_ + 3, gyro);
    std::copy(rpy_, rpy_ + 3, rpy);
  }

  const double now = get_clock()->now().seconds();
  log_file_ << std::fixed << std::setprecision(6) << now << ","
            << (policy_active ? 1 : 0) << "," << t_query << "," << mode << ","
            << des.vx << "," << des.vy << "," << des.yaw << "," << des.com_x
            << "," << des.com_y << "," << odom_x << "," << odom_y << ","
            << odom_z << "," << odom_yaw << "," << odom_vx << "," << odom_vy
            << "," << odom_vz << "," << gyro[0] << "," << gyro[1] << ","
            << gyro[2] << "," << rpy[0] << "," << rpy[1] << "," << rpy[2]
            << ",";

  for (int i = 0; i < 12; ++i)
    log_file_ << qm[i] << ",";
  for (int i = 0; i < 12; ++i)
    log_file_ << dqm[i] << ",";
  for (int i = 0; i < 12; ++i)
    log_file_ << q_ik[i] << ",";
  for (int i = 0; i < 12; ++i)
    log_file_ << target_joint_pos_[i] << ",";
  for (int i = 0; i < 12; ++i)
    log_file_ << dq_cmd[i] << ",";
  for (int i = 0; i < 12; ++i)
    log_file_ << torques[i] << ",";
  for (int i = 0; i < 4; ++i)
    log_file_ << foot_target_x_[i] << "," << foot_target_y_[i] << ","
              << foot_target_z_[i] << ",";
  for (int i = 0; i < 4; ++i)
    log_file_ << foot_takeoff_x_[i] << "," << foot_takeoff_y_[i] << ",";
  for (int i = 0; i < 4; ++i) {
    log_file_ << foot_vel_x_[i] << "," << foot_vel_y_[i] << ","
              << foot_vel_z_[i];
    if (i < 3)
      log_file_ << ",";
  }
  log_file_ << "\n";
  log_file_.flush();
}

// =============================================================================
//  Entry point
// =============================================================================
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WbcControllerNode>();
  RCLCPP_INFO(node->get_logger(), "WBC Controller Node started.");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
