#pragma once

// For dynamic memory and resource managing  
#include <memory>
// Threads allow multiple functions to execute concurrently
#include <thread>
// synchronization primitive that can be used to protect shared data from being simultaneously accessed by multiple threads
#include <mutex>
// needed for std::atomic and std::array
#include <atomic>
#include <array>
#include <vector>
#include <string>

// ROS2 Control
#include <controller_interface/controller_interface.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp/rclcpp.hpp>

// Hardware handlers and WBC
// FIX: HardwareCommandWriter.h does not exist. LeggedController.h defines HybridJointHandle, ImuSensorHandle,
// ContactSensorHandle and the 'using namespace ocs2 / legged_robot'
#include "legged_controllers/LeggedController.h"
#include "legged_controllers/SafetyChecker.h"
#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <legged_wbc/OptiPessiWbc.h>
#include <legged_interface/LeggedInterface.h>
#include <legged_estimation/LinearKalmanFilter.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>

// My solver
#include "acados_legged_controller/MpcSolver.hpp"

// Gait planner
#include "acados_legged_controller/GaitPlanner.hpp"

// CONFIG: parameters read from config.yaml
#include "acados_legged_controller/LipmConfig.hpp"

// Service per WALK
#include <std_srvs/srv/trigger.hpp>

// For Visualization
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

// Obstacle Manager
#include "acados_legged_controller/ObstacleManager.hpp"



namespace legged {

// Phase of the controller
enum class Phase { STAND_UP, STANCE, WALK };


class AcadosLeggedController : public controller_interface::ControllerInterface{
public: 
    AcadosLeggedController() = default;
    ~AcadosLeggedController() override;

    // Standard ROS2 Control overrides
    controller_interface::CallbackReturn on_init() override;
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

    // Real time loop
    controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;


private:
    void mpcThreadLoop(); // To play the async thread
    void updateStateEstimation(const rclcpp::Time& time, const rclcpp::Duration& period);
    // WARM START: initial guess for x and u along the horizon (port of the warm start of main.py)
    void setWarmStart(const std::vector<double>& x0, const GaitPlanner& gait, const std::vector<ObstacleState>& obstacles);
    void resetWarmStart(const std::vector<double>& x0);

    // Publish the MPC solution of the Optimistic and Pessimistic branch: CoM, feet trajectories and footholds
    void publishPredictions(const GaitPlanner& gait);
    // Publish obstacles and their keep-out zone in RViz
    void publishObstacles(); 

    // Torques, local velocities and limits for PlotJuggler
    void publishMonitor(const std::vector<double>& x0);
    void sendJointCommand(size_t j, double q, double dq, double kp, double kd, double tau);

    // WBC references as in OptiPessiController: all feet down where they are, CoM held at com_target_ with the height
    // ramp of the stance blend r in [0, 1] (holdStance + standUp)
    WbcReference stanceReference(double r) const;
    // Swing/stance feet and CoM of one MPC knot (updateFootReferences + updateComReference): CoM and yaw from the
    // reference of the step, forces and yaw acceleration of the knot from x_start (optimistic branch), the swing pair
    // is swing_time into its step
    WbcReference walkReference(const std::vector<double>& x_start, const std::vector<double>& u, double swing_time,
                               double step_duration, const GaitPlanner::Pair& stance, const GaitPlanner::Pair& swing) const;
    // Advances the CoM and yaw reference of the step by dt with the knot (x_start, u)
    void advanceWalkReference(const std::vector<double>& x_start, const std::vector<double>& u, double dt);
    // OptiPessiWbc on the reference, then the joint commands: WBC torque, velocity target = measured joint velocity
    // advanced by one step of the WBC accelerations (as OptiPessiController), PD (kp, kd) on q_target. False if the safety check fails
    bool updateWholeBodyControl(const WbcReference& reference, double period, const vector_t& q_target, double kp, double kd);


    // Hardware and state
    std::vector<HybridJointHandle> hybridJointHandles_;
    std::vector<ContactSensorHandle> contactHandles_;
    ImuSensorHandle imuSensorHandle_;

    // State estimation and WBC
    std::shared_ptr<StateEstimateBase> stateEstimate_;
    std::shared_ptr<OptiPessiWbc> wbc_;
    std::shared_ptr<PinocchioEndEffectorKinematics> eeKinematicsPtr_;

    std::shared_ptr<LeggedInterface> leggedInterface_;

    SystemObservation currentObservation_;
    vector_t measuredRbdState_;
    rclcpp::Node::SharedPtr ros2_node_;
    std::shared_ptr<CentroidalModelRbdConversions> rbdConversions_;
    std::shared_ptr<SafetyChecker> safetyChecker_;

    // acados MPC
    std::shared_ptr<MpcSolver> mpc_solver_; // FIX: std__ -> std::
    // CONFIG: all the numerical parameters (robot, limits, OCP dimensions) come from config.yaml
    LipmConfig cfg_;
    // dimensions of the OCP (copied from cfg_ in on_init)
    int n_obs_max_ = 0;
    int n_horizon_ = 0;
    size_t mpc_step_counter_ = 0;

    
    // WARM START: previous solution (X_sol_t_prec / U_sol_t_prec of main.py), used only by the MPC thread
    std::vector<std::vector<double>> x_prev_sol_;  // N+1 x nx
    std::vector<std::vector<double>> u_prev_sol_;  // N x nu
    size_t last_step_index_ = 0;                   // gait step of the last solve (shift only when it changes)
    std::shared_ptr<ObstacleManager> obstacle_manager_; // used only by the MPC thread


    // Multi threading
    std::thread mpc_thread_;
    std::mutex mpc_mutex_;
    std::atomic<bool> controller_running_{false};

    // Shared data between real time loop (update) and thread MPC
    std::vector<double> current_state_x0_;
    // Latest MPC solution (as AcceptedPlan of OptiPessiController): inputs of all the knots, the x0 it was solved from,
    // the gait step it was solved for and the time left in that step at that moment
    std::vector<std::vector<double>> latest_u_sol_;
    std::vector<double> latest_x0_;
    size_t latest_step_index_ = 0;
    double next_step_duration_ = 0.0; // duration of next step, protected by mpc_mutex_

    // YAW ZERO: yaw of the IMU at the first valid sample, removed from the orientation (as the position, zero at start)
    double yaw_offset_ = 0.0;
    bool yaw_offset_set_ = false;

    // Gait planner
    std::shared_ptr<GaitPlanner> gait_planner_; // protected by mpc_mutex_
    std::array<double, 4> stance_feet_xy_{};
    std::array<std::array<double, 2>, 4> swing_start_xy_{}; // lift-off xy of each foot, protected by mpc_mutex_ (RViz)


    // Stand up
    std::atomic<Phase> phase_{Phase::STAND_UP}; // atomic because even threadMPC reads it
    vector_t q_start_;          // joint angles at activation
    vector_t q_stand_;          // target posture
    double phase_time_ = 0.0;   // time spent in the current phase [s]
    
    // Stance: CoM and yaw held by the WBC
    vector3_t com_target_ = vector3_t::Zero();  // xy measured when entering STANCE
    double z_start_ = 0.0;                      // CoM height when entering STANCE, ramps to h_com
    double yaw_target_ = 0.0;

    // Walk
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_walking_srv_;
    std::atomic<bool> walk_requested_{false};   // set by the service, starts MPC thread
    std::atomic<bool> mpc_ready_{false};        // true after first successful solve !!
    // Feet
    std::array<vector3_t, 4> feet_pos_;         // measured feet {LF, RF, LH, RH}
    std::array<vector3_t, 4> liftoff_;          // feet at the start of the step (STANCE: where they are held)
    vector3_t com_pos_ = vector3_t::Zero();     // measured whole-body CoM
    size_t walk_step_ = SIZE_MAX;               // gait step seen by update
    int steps_at_goal_ = 0;                     // foot switches spent at the goal waiting fot the feet under hips
    std::vector<double> x0_measured_;           // LIPM x0 of this tick (control thread copy of current_state_x0_)
    std::vector<double> phase_start_x0_;        // LIPM x0 measured when the current step started (shifted plan)
    // CoM and yaw reference of the WBC: restarts from the measured robot at each step, then advanced every tick
    Eigen::Vector2d ref_c_ = Eigen::Vector2d::Zero(), ref_dc_ = Eigen::Vector2d::Zero();
    double ref_yaw_ = 0.0, ref_yaw_rate_ = 0.0;
    // Get the goal pose from /goal_pose from RViz
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    std::atomic<double> goal_x_{0.0}, goal_y_{0.0}; // Current goal, written by /goal_pose
    // HEADING: goal yaw from the orientation of /goal_pose. goal_has_yaw_ false (useGoalYaw off, or orientation
    // not set) -> the robot only faces the velocity and stops with any yaw
    std::atomic<double> goal_theta_{0.0};
    std::atomic<bool> goal_has_yaw_{false};
    bool use_goal_yaw_ = true;
    // Weight of the velocity alignment at distance d from the goal: 0 below d_near, 1 above d_far (smoothstep)
    double headingBlend(double d) const;
    std::atomic<double> sim_time_{0.0}; // written by update(), read by MPC thread

    // Save CoP traj of the two branches
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr prediction_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacle_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr monitor_pub_;
    std::array<std::atomic<double>, 12> tau_cmd_{}; // total commanded torques, written by update(), read by MPC thread
    // WBC tracking for PlotJuggler, written by update(), read by MPC thread
    enum WbcMonitor { YAW_REF, YAW_RATE_REF, YAW_ACC_FF, YAW_ACC_WBC, COM_ERR_X, COM_ERR_Y,
                      RESIDUAL_X, RESIDUAL_Y, RESIDUAL_Z, N_WBC_MONITOR };
    std::array<std::atomic<double>, N_WBC_MONITOR> wbc_monitor_{};
 
};

} // namespace legged