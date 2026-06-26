/**
 * @file WbcControllerNode.h
 * @brief Whole-Body Controller node declaration for Unitree Go2 with Opti-Pessi MPC.
 *
 * Receives the centroidal policy from the MPC (/legged_robot_mpc_policy) and
 * the physical robot state (/lowstate + /odom), then:
 *   1. Interpolates the desired CoM state and foothold positions from the trajectory.
 *   2. Tracks swing/stance phases and computes per-leg foot targets in world frame.
 *   3. Runs analytical IK to produce initial joint references.
 *   4. Calls OptiPessiWbc::setMpcInput(), setSwingFootTargets(), update() to solve the QP.
 *   5. Publishes resulting joint positions, velocities, and torques on /lowcmd.
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cmath>
#include <mutex>
#include <vector>
#include <algorithm>
#include <fstream>
#include <iomanip>

// OCS2 policy messages
#include <ocs2_msgs/msg/mpc_flattened_controller.hpp>

// Unitree hardware messages
#include <unitree_go/msg/low_state.hpp>
#include <unitree_go/msg/low_cmd.hpp>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

// OCS2 & Pinocchio
#include <ocs2_centroidal_model/FactoryFunctions.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>

// Our WBC implementation (only public API used: setMpcInput, setSwingFootTargets, update)
#include "opti_pessi_control/OptiPessiWbc.h"

// =============================================================================
//  Go2 Kinematics Constants & Helpers
// =============================================================================
namespace go2_kinematics {

// Link lengths
constexpr double HIP_OFFSET   = 0.0955;
constexpr double THIGH_LENGTH = 0.213;
constexpr double CALF_LENGTH  = 0.213;

// Leg origins relative to the trunk centre
constexpr double LEG_OFFSET_X = 0.1934;
constexpr double LEG_OFFSET_Y = 0.0465;

// Nominal standing height of the base CoM
constexpr double NOMINAL_HEIGHT = 0.23;

// Joint limits (rad)
constexpr double HIP_MIN   = -1.0472;
constexpr double HIP_MAX   =  1.0472;
constexpr double THIGH_MIN = -0.5;
constexpr double THIGH_MAX =  3.4907;
constexpr double CALF_MIN  = -2.7227;
constexpr double CALF_MAX  = -0.4;

// Distance from CoM → base_link along the vertical (negative in standing pose)
constexpr double COM_TO_BASE_Z = 0.0;

// Default standing joint angles matching reference.info defaultJointState
// Left legs (FL=0, RL=2): +0.10 rad hip; Right legs (FR=1, RR=3): -0.10 rad hip
inline double defaultHip(int leg) { return (leg == 0 || leg == 2) ? 0.10 : -0.10; }
constexpr double DEFAULT_THIGH =  0.67;
constexpr double DEFAULT_CALF  = -1.3;

// Per-leg sign conventions (Unitree order: FL=0, FR=1, RL=2, RR=3)
struct LegConfig { double sign_x; double sign_y; int mirror; };
constexpr LegConfig LEGS[4] = {
    { 1.0,  1.0,  1},   // FL: front-left
    { 1.0, -1.0, -1},   // FR: front-right
    {-1.0,  1.0,  1},   // RL: rear-left
    {-1.0, -1.0, -1},   // RR: rear-right
};

/**
 * Analytical inverse kinematics for one leg.
 * @param foot_x/y/z  Foot position in the hip joint frame.
 * @param mirror       +1 for left legs, -1 for right legs.
 * @param q_hip/thigh/calf  Output joint angles (clamped to limits).
 * @return true always (outputs are clamped, not rejected).
 */
bool analyticalIK(double foot_x, double foot_y, double foot_z, int mirror,
                  double& q_hip, double& q_thigh, double& q_calf);

} // namespace go2_kinematics

// =============================================================================
//  WbcControllerNode
// =============================================================================

enum class RobotMode { LIE, STAND, WALK };

class WbcControllerNode : public rclcpp::Node {
public:
    WbcControllerNode();
    ~WbcControllerNode();

private:
    // =========================================================================
    //  Internal types
    // =========================================================================

    struct DesiredState {
        double vx = 0, vy = 0, yaw = 0;
        double com_x = 0, com_y = 0;
        double p0_x = 0, p0_y = 0;
        double p1_x = 0, p1_y = 0;
        double alpha = 0.5;
    };

    struct SwingInfo {
        bool   in_stance = true;
        double phase     = 0.0;   // [0, 1]
        double duration  = 0.35;
    };

    /// Gait pattern loaded from config/gait.info (hardcoded to dynamic_walk).
    struct GaitSchedule {
        std::vector<int> mode_sequence;   ///< Unitree bitmasks per phase (FL=bit0,FR=bit1,RL=bit2,RR=bit3; bit SET = stance)
        double           phase_duration;  ///< Uniform phase duration (s)
        double           cycle_period;    ///< Full cycle period = N * phase_duration
    };

    // =========================================================================
    //  Callbacks
    // =========================================================================

    void robotModeCallback(const std_msgs::msg::String::SharedPtr msg);
    void policyCallback(const ocs2_msgs::msg::MpcFlattenedController::SharedPtr msg);
    void lowstateCallback(const unitree_go::msg::LowState::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void kpKdCallback(const geometry_msgs::msg::Point::SharedPtr msg);

    // =========================================================================
    //  Main control loop (500 Hz)
    // =========================================================================
    void controlLoop();

    // =========================================================================
    //  Trajectory interpolation
    // =========================================================================

    /** Linearly interpolate desired CoM state and foothold parameters at t_query. */
    DesiredState interpolateTrajectory(double t_query);

    // =========================================================================
    //  Foot target tracking
    // =========================================================================

    /** Return true if leg is in stance in the given mode bitmask. */
    static bool isStance(int leg, int mode) { return (mode & (1 << leg)) != 0; }

    /** Find the mode at t_query from the (event_times, mode_seq) schedule. */
    size_t findActiveMode(double t_query, double t_start, double t_end,
                          const std::vector<double>& event_times,
                          const std::vector<int>& mode_seq) const;

    /** Determine swing/stance status and swing phase for one leg at t_query. */
    SwingInfo computeSwingInfo(int leg, double t_query, double t_start, double t_end,
                               const std::vector<double>& event_times,
                               const std::vector<int>& mode_seq) const;

    /**
     * Build a cyclic mode schedule from the loaded GaitSchedule for the interval [t_start, t_end].
     * The phase is anchored to gait_start_time_ (wall-clock time when WALK mode began).
     */
    void computeGaitModeSchedule(double t_start, double t_end,
                                  std::vector<double>& event_times,
                                  std::vector<int>& mode_seq);

    /** Compute the nominal foot contact position in world frame. */
    void footholdWorld(int leg, double px, double py, double yaw,
                       double& wx, double& wy) const;

    /** Initialise foot target positions from the current base odometry. */
    void initFootTargets();

    /** Update the world-frame foot target for one leg and compute foot velocity. */
    void updateFootTarget(int leg, double t_query, double t_start, double t_end,
                          const std::vector<double>& event_times,
                          const std::vector<int>& mode_seq,
                          const DesiredState& des);

    // =========================================================================
    //  Inverse kinematics
    // =========================================================================

    /** Run analytical IK for leg and write the result into target_joints[]. */
    void runIK(int leg, double base_x, double base_y, double base_z, double base_yaw,
               double target_joints[12]) const;

    // =========================================================================
    //  WBC state vector builders
    // =========================================================================

    /**
     * Build the 36-dim measured RBD state for OptiPessiWbc::update().
     * Layout: [yaw, pitch, roll, x, y, z, q_joints(12),
     *          gyro(3), vx, vy, vz, dq_joints(12)]
     */
    ocs2::vector_t buildMeasuredRbdState() const;

    /**
     * Build the 24-dim centroidal desired state for OptiPessiWbc::update().
     * Layout: [linear momentum (3), angular momentum (3), euler angles (3),
     *          CoM position (3), joint positions (12)]
     */
    ocs2::vector_t buildDesiredState(const DesiredState& des) const;

    // =========================================================================
    //  Low-level hardware publish
    // =========================================================================
    void publishLowCmd(const double positions[12], const double velocities[12],
                       const double torques[12]);
    void publishLowCmd(const double positions[12], const double velocities[12],
                       const double torques[12], double kp, double kd);

    // =========================================================================
    //  CSV Logging
    // =========================================================================
    void writeCSVHeader();
    void logToCSV(bool policy_active, double t_query, size_t mode,
                  const DesiredState& des,
                  const double q_ik[12], const double dq_cmd[12], const double torques[12]);

    // =========================================================================
    //  Member variables
    // =========================================================================

    // Control parameters
    double kp_, kd_, swing_height_;
    mutable std::mutex gains_mutex_;
    static constexpr double kControlPeriod_ = 0.002;  // 500 Hz

    // Robot state (protected by state_mutex_)
    std::mutex state_mutex_;
    bool   state_received_   = false;
    bool   odom_received_    = false;
    double current_joint_pos_[12] = {};
    double current_joint_vel_[12] = {};
    double gyroscope_[3] = {};
    double rpy_[3]       = {};
    double odom_x_  = 0, odom_y_  = 0, odom_z_  = 0, odom_yaw_ = 0;
    double odom_vx_ = 0, odom_vy_ = 0, odom_vz_ = 0;

    // MPC policy (protected by policy_mutex_)
    std::mutex policy_mutex_;
    bool   policy_received_ = false;
    double policy_time_     = 0.0;
    std::vector<double>              time_trajectory_;
    std::vector<std::vector<float>>  state_trajectory_;
    std::vector<std::vector<float>>  input_trajectory_;
    std::vector<double>              mode_event_times_;
    std::vector<int>                 mode_sequence_;

    std::mutex is_wbc_running_mutex_;
    std::mutex robot_mode_mutex_;
    RobotMode  robot_mode_ = RobotMode::STAND;

    // Joint targets (control loop only — no mutex needed)
    double target_joint_pos_[12]      = {};
    double prev_target_joint_pos_[12] = {};

    // Per-leg foot tracking (world frame)
    double foot_target_x_[4]  = {};
    double foot_target_y_[4]  = {};
    double foot_target_z_[4]  = {};
    double foot_takeoff_x_[4] = {};
    double foot_takeoff_y_[4] = {};
    double foot_vel_x_[4]     = {};
    double foot_vel_y_[4]     = {};
    double foot_vel_z_[4]     = {};
    bool   was_in_stance_[4]        = {true, true, true, true};
    bool   first_loop_[4]           = {true, true, true, true};
    bool   foot_targets_initialized_ = false;
    bool   is_wbc_running_          = false;
    GaitSchedule gait_schedule_;           ///< Loaded gait pattern (dynamic_walk from gait.info)
    double       gait_start_time_ = -1.0;  ///< Wall-clock time when WALK mode started (-1 = unset)

    // WBC & Pinocchio
    std::unique_ptr<legged::OptiPessiWbc> wbc_;
    ocs2::CentroidalModelInfo centroidalModelInfo_;

    // ROS 2
    rclcpp::Subscription<ocs2_msgs::msg::MpcFlattenedController>::SharedPtr policy_sub_;
    rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr              lowstate_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr                odom_sub_;
    rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr                   lowcmd_pub_;
    rclcpp::TimerBase::SharedPtr                                            control_timer_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr                    is_wbc_running_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr                  robot_mode_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr              kp_kd_sub_;

    // Logging
    std::ofstream log_file_;
    std::string   log_file_path_;
};
