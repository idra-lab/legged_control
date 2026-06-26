/**
 * @file opti_pessi_mpc_node.cpp
 * @brief Method implementations for OptiPessiMpcNode.
 */

#include "opti_pessi_control/nodes/OptiPessiMpcNode.h"

// =============================================================================
//  OptiPessiMpcNode — constructor
// =============================================================================

OptiPessiMpcNode::OptiPessiMpcNode() : Node("opti_pessi_mpc_node") {
    RCLCPP_INFO(this->get_logger(), "Inizializzazione Nodo Opti-Pessi MPC per WBC...");

    obs_state_ = std::make_shared<ocs2::quadruped::ObstacleState>();

    // 1. Publisher per la Policy Macroscopica verso il WBC
    mpc_policy_pub_ = this->create_publisher<ocs2_msgs::msg::MpcFlattenedController>(
        "/legged_robot_mpc_policy", 10);

    // 2. Sottoscrizioni
    robot_state_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&OptiPessiMpcNode::robotStateCallback, this, std::placeholders::_1));

    obstacle_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
        "/obstacle_pose", 10, std::bind(&OptiPessiMpcNode::obstacleCallback, this, std::placeholders::_1));

    goal_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
        "/goal_pose", 10, std::bind(&OptiPessiMpcNode::goalCallback, this, std::placeholders::_1));

    // 3. Definizione del problema di controllo ottimo (OCP)
    ocs2::OptimalControlProblem ocp;
    ocp.dynamicsPtr.reset(new ocs2::quadruped::OptiPessiAugmentedDynamics());
    ocp.costPtr->add("augmented_cost", std::make_unique<ocs2::quadruped::OptiPessiAugmentedCost>());
    ocp.equalityConstraintPtr->add("coupling_constraint",
        std::make_unique<ocs2::quadruped::OptiPessiCouplingConstraint>(obs_state_));

    ocs2::RelaxedBarrierPenalty::Config obstacleBarrierConfig;
    obstacleBarrierConfig.mu    = 5.0;
    obstacleBarrierConfig.delta = 0.05;
    ocp.softConstraintPtr->add(
        "obstacle_avoidance",
        std::make_unique<ocs2::StateInputSoftConstraint>(
            std::make_unique<ocs2::quadruped::OptiPessiObstacleConstraint>(obs_state_),
            std::make_unique<ocs2::RelaxedBarrierPenalty>(obstacleBarrierConfig)));

    // 4. Configurazione impostazioni solutore
    ocs2::mpc::Settings mpcSettings;
    mpcSettings.timeHorizon_ = 2.0;

    ocs2::sqp::Settings sqpSettings;
    sqpSettings.sqpIteration                          = 5;
    sqpSettings.dt                                    = 0.04;
    sqpSettings.projectStateInputEqualityConstraints  = true;

    ocs2::DefaultInitializer initializer(16);
    mpc_solver_.reset(new ocs2::SqpMpc(mpcSettings, sqpSettings, ocp, initializer));

    // 5. Configurazione Traiettorie Goal
    ocs2::vector_t goal_state = ocs2::vector_t::Zero(20);
    goal_state[0] = 0.0;       // X ottimista (standing pose)
    goal_state[6] = 0.1934;    // p0_x front
    goal_state[8] = -0.1934;   // p1_x rear
    goal_state[10] = 0.0;      // X pessimista
    goal_state[16] = 0.1934;   // p0_x front
    goal_state[18] = -0.1934;  // p1_x rear

    ocs2::TargetTrajectories targetTrajectories({0.0}, {goal_state}, {ocs2::vector_t::Zero(16)});
    mpc_solver_->getSolverPtr()->getReferenceManager().setTargetTrajectories(targetTrajectories);

    current_augmented_state_ = ocs2::vector_t::Zero(20);
    current_time_ = 0.0;

    // 6. Loop principale a 50 Hz
    control_loop_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20), std::bind(&OptiPessiMpcNode::mpcControlLoop, this));

    RCLCPP_INFO(this->get_logger(), "Nodo Opti-Pessi MPC pronto per il WBC!");
}

// =============================================================================
//  Callbacks
// =============================================================================

void OptiPessiMpcNode::obstacleCallback(const geometry_msgs::msg::Point::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(obs_state_->mutex);
    obs_state_->x       = msg->x;
    obs_state_->y       = msg->y;
    obs_state_->max_vel = msg->z > 0.0 ? msg->z : 0.5;
}

void OptiPessiMpcNode::goalCallback(const geometry_msgs::msg::Point::SharedPtr msg) {
    double current_time = this->get_clock()->now().seconds();
    double t_relative   = (t_start_ > 0.0) ? (current_time - t_start_) : 0.0;

    ocs2::vector_t goal_state = ocs2::vector_t::Zero(20);
    goal_state[0] = msg->x;           // X ottimista
    goal_state[1] = msg->y;           // Y ottimista
    goal_state[6] = msg->x + 0.1934;  // p0_x front
    goal_state[7] = msg->y;           // p0_y front
    goal_state[8] = msg->x - 0.1934;  // p1_x rear
    goal_state[9] = msg->y;           // p1_y rear

    goal_state[10] = msg->x;          // X pessimista
    goal_state[11] = msg->y;          // Y pessimista
    goal_state[16] = goal_state[6];   // p0_x front
    goal_state[17] = goal_state[7];   // p0_y front
    goal_state[18] = goal_state[8];   // p1_x rear
    goal_state[19] = goal_state[9];   // p1_y rear

    ocs2::TargetTrajectories targetTrajectories({t_relative}, {goal_state}, {ocs2::vector_t::Zero(16)});
    mpc_solver_->getSolverPtr()->getReferenceManager().setTargetTrajectories(targetTrajectories);
}

void OptiPessiMpcNode::robotStateCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (current_augmented_state_.size() != 20) {
        current_augmented_state_ = ocs2::vector_t::Zero(20);
    }
    double qx = msg->pose.pose.orientation.x;
    double qy = msg->pose.pose.orientation.y;
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;
    current_yaw_ = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));

    // Scenario 1: Ottimista (Indici 0-9)
    current_augmented_state_[0] = msg->pose.pose.position.x;   // pos_x
    current_augmented_state_[1] = msg->pose.pose.position.y;   // pos_y
    current_augmented_state_[2] = current_yaw_;                // yaw (theta)
    current_augmented_state_[3] = msg->twist.twist.linear.x;   // vel_x
    current_augmented_state_[4] = msg->twist.twist.linear.y;   // vel_y
    current_augmented_state_[5] = msg->twist.twist.angular.z;  // yaw_rate

    double cos_yaw = std::cos(current_yaw_);
    double sin_yaw = std::sin(current_yaw_);
    double s_x = 0.1934;

    current_augmented_state_[6] = msg->pose.pose.position.x + s_x * cos_yaw; // p0_x
    current_augmented_state_[7] = msg->pose.pose.position.y + s_x * sin_yaw; // p0_y
    current_augmented_state_[8] = msg->pose.pose.position.x - s_x * cos_yaw; // p1_x
    current_augmented_state_[9] = msg->pose.pose.position.y - s_x * sin_yaw; // p1_y

    // Scenario 2: Pessimista (Indici 10-19)
    current_augmented_state_[10] = msg->pose.pose.position.x;  // pos_x
    current_augmented_state_[11] = msg->pose.pose.position.y;  // pos_y
    current_augmented_state_[12] = current_yaw_;               // yaw (theta)
    current_augmented_state_[13] = msg->twist.twist.linear.x;  // vel_x
    current_augmented_state_[14] = msg->twist.twist.linear.y;  // vel_y
    current_augmented_state_[15] = msg->twist.twist.angular.z; // yaw_rate

    current_augmented_state_[16] = current_augmented_state_[6];
    current_augmented_state_[17] = current_augmented_state_[7];
    current_augmented_state_[18] = current_augmented_state_[8];
    current_augmented_state_[19] = current_augmented_state_[9];

    current_time_ = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    state_received_ = true;
}

// =============================================================================
//  Main MPC control loop (50 Hz)
// =============================================================================

void OptiPessiMpcNode::mpcControlLoop() {
    if (!state_received_) return;

    double current_time = this->get_clock()->now().seconds();
    if (t_start_ < 0.0) {
        t_start_ = current_time;
    }
    double t_relative = current_time - t_start_;

    {
        std::lock_guard<std::mutex> lock(obs_state_->mutex);
        obs_state_->init_time = t_relative;
    }

    // Esecuzione del solutore SQP MPC
    bool solver_success = mpc_solver_->run(t_relative, current_augmented_state_);
    if (!solver_success) {
        RCLCPP_WARN(this->get_logger(), "Il solutore MPC ha fallito la convergenza!");
        return;
    }

    // Estrazione della soluzione ottimale (Primal Solution)
    ocs2::PrimalSolution primalSolution;
    mpc_solver_->getSolverPtr()->getPrimalSolution(t_relative, &primalSolution);

    auto& timeSequence  = primalSolution.timeTrajectory_;
    auto& stateSequence = primalSolution.stateTrajectory_;
    auto& inputSequence = primalSolution.inputTrajectory_;

    if (inputSequence.empty() || stateSequence.empty()) return;

    // =========================================================================
    // COSTRUZIONE DEL MESSAGGIO MPC FLATTENED CONTROLLER PER IL WBC
    // =========================================================================
    ocs2_msgs::msg::MpcFlattenedController policy_msg;

    policy_msg.controller_type = ocs2_msgs::msg::MpcFlattenedController::CONTROLLER_FEEDFORWARD;

    // 1. Traiettoria temporale
    policy_msg.time_trajectory = timeSequence;

    // 2. Post-event indices
    policy_msg.post_event_indices.reserve(primalSolution.postEventIndices_.size());
    for (auto idx : primalSolution.postEventIndices_) {
        policy_msg.post_event_indices.push_back(static_cast<uint16_t>(idx));
    }

    // 3. Traiettoria di Stato e Input
    policy_msg.state_trajectory.clear();
    policy_msg.input_trajectory.clear();

    for (size_t i = 0; i < stateSequence.size(); ++i) {
        ocs2_msgs::msg::MpcState mpc_state;
        mpc_state.value.resize(24, 0.0f);

        ocs2::vector_t x_real = stateSequence[i].head(10);

        // Mappatura: stato a 10 elementi → struttura centroidale a 24 elementi
        mpc_state.value[3]  = static_cast<float>(x_real[3]); // vx_com
        mpc_state.value[4]  = static_cast<float>(x_real[4]); // vy_com
        mpc_state.value[5]  = 0.0f;                          // vz_com
        mpc_state.value[6]  = static_cast<float>(x_real[2]); // yaw
        mpc_state.value[7]  = 0.0f;                          // pitch
        mpc_state.value[8]  = 0.0f;                          // roll
        mpc_state.value[9]  = static_cast<float>(x_real[0]); // x_com
        mpc_state.value[10] = static_cast<float>(x_real[1]); // y_com
        mpc_state.value[11] = 0.45f;                         // z_com (altezza nominale)

        policy_msg.state_trajectory.push_back(mpc_state);
    }

    for (size_t i = 0; i < inputSequence.size(); ++i) {
        ocs2_msgs::msg::MpcInput mpc_input;
        mpc_input.value.resize(24, 0.0f);
        for (size_t j = 0; j < 16 && j < inputSequence[i].size(); ++j) {
            mpc_input.value[j] = static_cast<float>(inputSequence[i][j]);
        }
        policy_msg.input_trajectory.push_back(mpc_input);
    }

    // 4. Generazione dinamica della Gait (MODE SCHEDULE)
    // Mappiamo alpha (u[4]) nei contatti discreti del quadruped per il WBC
    // Convenzione: Mode 15 = STANCE, Mode 9 = LF_RH Stance, Mode 6 = RF_LH Stance
    ocs2_msgs::msg::ModeSchedule mode_schedule_msg;
    int current_mode = -1;

    for (size_t i = 0; i < inputSequence.size(); ++i) {
        double t_step = timeSequence[i];
        double alpha  = inputSequence[i][4];

        int step_mode = 15;  // Default: STANCE
        if (alpha < 0.1) {
            step_mode = 9;   // Diagonale 1 (LF_RH) in stance
        } else if (alpha > 0.9) {
            step_mode = 6;   // Diagonale 2 (RF_LH) in stance
        }

        if (i == 0) {
            current_mode = step_mode;
            mode_schedule_msg.mode_sequence.push_back(current_mode);
        } else if (step_mode != current_mode) {
            mode_schedule_msg.event_times.push_back(t_step);
            current_mode = step_mode;
            mode_schedule_msg.mode_sequence.push_back(current_mode);
        }
    }

    policy_msg.mode_schedule = mode_schedule_msg;
    mpc_policy_pub_->publish(policy_msg);
}

// =============================================================================
//  Entry point
// =============================================================================
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OptiPessiMpcNode>());
    rclcpp::shutdown();
    return 0;
}
