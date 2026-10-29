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
#include <legged_wbc/WeightedWbc.h>
#include <legged_interface/LeggedInterface.h>
#include <legged_estimation/LinearKalmanFilter.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>

// My solver
#include "acados_legged_controller/MpcSolver.hpp"

// Gait planner
#include "acados_legged_controller/GaitPlanner.hpp"

// CONFIG: parameters read from config.yaml
#include "acados_legged_controller/LipmConfig.hpp"


namespace legged {

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


    // Hardware and state
    std::vector<HybridJointHandle> hybridJointHandles_;
    std::vector<ContactSensorHandle> contactHandles_;
    ImuSensorHandle imuSensorHandle_;

    // State estimation and WBC
    std::shared_ptr<StateEstimateBase> stateEstimate_;
    std::shared_ptr<WeightedWbc> wbc_;
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

    // Multi threading
    std::thread mpc_thread_;
    std::mutex mpc_mutex_;
    std::atomic<bool> controller_running_{false};

    // Shared data between real time loop (update) and thread MPC
    std::vector<double> current_state_x0_; 
    std::vector<double> latest_u_opt_;
    std::vector<double> latest_x_opt_;


    // Gait planner
    std::shared_ptr<GaitPlanner> gait_planner_; // protected by mpc_mutex_
    std::array<double, 4> stance_feet_xy_{};


};

} // namespace legged