// FIX: pinocchio forward declarations must be included before everything else
#include <pinocchio/fwd.hpp>

#include "acados_legged_controller/AcadosLeggedController.hpp"
#include <pluginlib/class_list_macros.hpp>

// ocs2 helper to load the kinematic for WBC from pinocchio
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>

// (angles::shortest_angular_distance, getJointAngles/getGeneralizedCoordinates, forward kinematics)
#include <angles/angles.h>
#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>

// WARM START: std::cos/sin/hypot, std::min/max/copy
#include <cmath>
#include <algorithm>
#include <chrono>
// CONFIG: default path of config.yaml (installed in share/acados_legged_controller/config)
#include <ament_index_cpp/get_package_share_directory.hpp>


namespace legged {

controller_interface::CallbackReturn AcadosLeggedController::on_init(){
    auto node = this->get_node();




    // Load parameters from launch file
    if (!node->has_parameter("urdfFile")){
        node->declare_parameter<std::string>("urdfFile", "");
    }
    if (!node->has_parameter("taskFile")){
        node->declare_parameter<std::string>("taskFile", "");
    }
    if (!node->has_parameter("referenceFile")){
        node->declare_parameter<std::string>("referenceFile", "");
    }
    // CONFIG: path of config.yaml (if empty use the installed one)
    if (!node->has_parameter("lipmConfigFile")){
        node->declare_parameter<std::string>("lipmConfigFile", "");
    }

    

    std::string urdfFile = node->get_parameter("urdfFile").as_string();
    std::string taskFile = node->get_parameter("taskFile").as_string();
    std::string referenceFile = node->get_parameter("referenceFile").as_string();
    std::string lipmConfigFile = node->get_parameter("lipmConfigFile").as_string();
    bool verbose = true;

    // CONFIG: load config.yaml
    if (lipmConfigFile.empty()){
        lipmConfigFile = ament_index_cpp::get_package_share_directory("acados_legged_controller") + "/config/config.yaml";
    }
    try{
        cfg_ = LipmConfig::fromYaml(lipmConfigFile);
    }catch(const std::exception& e){
        RCLCPP_ERROR(node->get_logger(), "%s", e.what());
        return controller_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(node->get_logger(), "[Acados MPC]: Loaded config from %s", lipmConfigFile.c_str());

    // Initialize the ocs2 interface only for reading the phisics of the robot
    leggedInterface_ = std::make_shared<LeggedInterface>(taskFile, urdfFile, referenceFile);
    leggedInterface_->setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose);

    // Extract the stand posture of the joints 
    // Stand posture from h_com: foot under the hip, HAA = 0 -> feet at y = off_y
    const double hfe = std::acos((cfg_.h_com - cfg_.foot_radius) / (2.0 * cfg_.leg_length));
    q_stand_.resize(12);
    for (int leg = 0; leg < 4; ++leg) q_stand_.segment<3>(3 * leg) << 0.0, hfe, -2.0 * hfe;
    RCLCPP_INFO_STREAM(node->get_logger(), "[Acados MPC]: q_stand is: " << q_stand_.transpose());

    // Guard: LIPM mass unit match URDF
    const double urdf_mass = leggedInterface_->getCentroidalModelInfo().robotMass;
    RCLCPP_INFO(node->get_logger(), "[Acados MPC]: mass of config.yaml %.3f, URDF %.3f", cfg_.m, urdf_mass);


    // The Kalman filter needs an rclcpp::Node, before it was never created
    ros2_node_ = std::make_shared<rclcpp::Node>("acados_legged_controller_node");

    // Initialize acados MPC solver
    // FIX: store the dimensions in the members (used by mpcThreadLoop)
    // CONFIG: dimensions from config.yaml (MpcSolver checks them against c_generated_code)
    n_obs_max_ = cfg_.N_OBS_MAX;
    n_horizon_ = cfg_.N_horizon;
    try{
        mpc_solver_ = std::make_shared<MpcSolver>(n_obs_max_, n_horizon_);
    }catch(const std::exception& e){
        RCLCPP_ERROR(node->get_logger(), "%s", e.what());
        return controller_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(node->get_logger(), "[Acados MPC]: Solver initialized with success!!");

    // CONFIG: off_x, off_y and nominal step duration from config.yaml
    gait_planner_ = std::make_shared<GaitPlanner>(cfg_.off_x, cfg_.off_y, cfg_.nominalStepDuration());


    // Setup of the kinematic, state estimate and Whole Body Control
    CentroidalModelPinocchioMapping pinocchioMapping(leggedInterface_->getCentroidalModelInfo());
    eeKinematicsPtr_ = std::make_shared<PinocchioEndEffectorKinematics>(
        leggedInterface_->getPinocchioInterface(), pinocchioMapping,
        leggedInterface_->modelSettings().contactNames3DoF);
    // without this getPosition() throws "pinocchioInterfacePtr_ is not set"
    eeKinematicsPtr_->setPinocchioInterface(leggedInterface_->getPinocchioInterface());

    // created once here instead of at every cycle
    rbdConversions_ = std::make_shared<CentroidalModelRbdConversions>(
        leggedInterface_->getPinocchioInterface(), leggedInterface_->getCentroidalModelInfo());
    safetyChecker_ = std::make_shared<SafetyChecker>(leggedInterface_->getCentroidalModelInfo());

    stateEstimate_ = std::make_shared<KalmanFilterEstimate>(
        ros2_node_, leggedInterface_->getPinocchioInterface(),
        leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_);
    dynamic_cast<KalmanFilterEstimate&>(*stateEstimate_).loadSettings(taskFile, verbose);
    
    
    wbc_ = std::make_shared<WeightedWbc>(
        leggedInterface_->getPinocchioInterface(),
        leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_);
    wbc_->loadTasksSetting(taskFile, verbose);
    
    return controller_interface::CallbackReturn::SUCCESS;
}






controller_interface::InterfaceConfiguration AcadosLeggedController::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    
    // Tell to ROS2 that we want to control pos, vel, effort, kp, kd for each joint
    std::vector<std::string> joint_names{"LF_HAA", "LF_HFE", "LF_KFE", "LH_HAA", "LH_HFE", "LH_KFE", 
                                        "RF_HAA", "RF_HFE", "RF_KFE", "RH_HAA", "RH_HFE", "RH_KFE"};

    for (const auto& joint : joint_names){
        config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
        config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
        config.names.push_back(joint + "/kp");
        config.names.push_back(joint + "/kd");
        config.names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
    }
    return config;
}





controller_interface::InterfaceConfiguration AcadosLeggedController::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    // JOINTS
    std::vector<std::string> joint_names{"LF_HAA", "LF_HFE", "LF_KFE", "LH_HAA", "LH_HFE", "LH_KFE", 
                                        "RF_HAA", "RF_HFE", "RF_KFE", "RH_HAA", "RH_HFE", "RH_KFE"};
    for (const auto& joint : joint_names){
        config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
        config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
        config.names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
    }


    // IMU SENSOR
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


    // CONTACT SENSORS
    std::vector<std::string> contact_names{"LF_FOOT", "LH_FOOT", "RF_FOOT", "RH_FOOT"};
    for(const auto& contact : contact_names){
        config.names.push_back(contact + "/contact");
    }
    return config;
}






controller_interface::CallbackReturn AcadosLeggedController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
    using Trigger = std_srvs::srv::Trigger;

    // get_node() is spun (eseguito) by the controller_manager, ros2_node_ is not
    start_walking_srv_ = get_node()->create_service<Trigger>("~/start_walking",
        [this](const std::shared_ptr<Trigger::Request> /*req*/, std::shared_ptr<Trigger::Response> res) {
            if (phase_ != Phase::STANCE){
                res->success = false;
                res->message = "The robot is not in STANCE";
                return;
            }
            walk_requested_ = true;
            res->success = true;
            res->message = "MPC started";
        }   
    );

    return controller_interface::CallbackReturn::SUCCESS;
}




// on_activate: link the pointers of ros2 to the phisical hardware and start the thread
controller_interface::CallbackReturn AcadosLeggedController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
    auto node = this->get_node();

    // Binding of the hardware interfaces
    hybridJointHandles_.clear();
    contactHandles_.clear();
    
    std::vector<std::string> joint_names{"LF_HAA", "LF_HFE", "LF_KFE", "LH_HAA", "LH_HFE", "LH_KFE", 
                                        "RF_HAA", "RF_HFE", "RF_KFE", "RH_HAA", "RH_HFE", "RH_KFE"};

    for(const auto& joint : joint_names){
        HybridJointHandle joint_handle;
        for(auto& state_interface : state_interfaces_){
            if(state_interface.get_prefix_name() == joint){
                if(state_interface.get_interface_name() == hardware_interface::HW_IF_POSITION) joint_handle.positionState = &state_interface;
                else if(state_interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY) joint_handle.velocityState = &state_interface;
                else if(state_interface.get_interface_name() == hardware_interface::HW_IF_EFFORT) joint_handle.effortState = &state_interface;                
            }
        }
        for (auto& command_interface : command_interfaces_) {
            if (command_interface.get_prefix_name() == joint) {
                if (command_interface.get_interface_name() == hardware_interface::HW_IF_POSITION) joint_handle.positionCmd = &command_interface;
                else if (command_interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY) joint_handle.velocityCmd = &command_interface;
                else if (command_interface.get_interface_name() == "kp") joint_handle.kpCmd = &command_interface;
                else if (command_interface.get_interface_name() == "kd") joint_handle.kdCmd = &command_interface;
                else if (command_interface.get_interface_name() == hardware_interface::HW_IF_EFFORT) joint_handle.effortCmd = &command_interface;
            }
        }
        hybridJointHandles_.push_back(joint_handle);
    }


    // IMU
    std::string imu_name = "base_imu";
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


    // CONTACTS
    std::vector<std::string> contact_names{"LF_FOOT","RF_FOOT","LH_FOOT","RH_FOOT"};
    for(const auto& contact : contact_names){
        ContactSensorHandle contact_handle;
        for(auto& state_interface : state_interfaces_){
            if(state_interface.get_prefix_name() == contact && state_interface.get_interface_name() == "contact"){
                contact_handle.contactState = &state_interface;
                break;
            }
        }
        contactHandles_.push_back(contact_handle);
    }

    // At each activation, even after a deactivate, restart from the current pose of the robot
    q_start_.resize(hybridJointHandles_.size());
    for(size_t j = 0; j < hybridJointHandles_.size(); ++j) q_start_(j) = hybridJointHandles_[j].getPosition();
    phase_time_ = 0.0;
    phase_ = Phase::STAND_UP;


    // Initialization of the shared data
    current_state_x0_.assign(22, 0.0);
    latest_x_opt_.assign(22, 0.0);
    // latest_u_opt_ will be resized in the first solve()

    // initialize the observation (state(9) is read in updateStateEstimation) and do a first
    // estimation, so the MPC thread does not start from x0 = 0
    currentObservation_.state.setZero(leggedInterface_->getCentroidalModelInfo().stateDim);
    currentObservation_.input.setZero(leggedInterface_->getCentroidalModelInfo().inputDim);
    currentObservation_.mode = ModeNumber::STANCE;
    updateStateEstimation(node->get_clock()->now(), rclcpp::Duration::from_seconds(0.002));


    // Initialize the step duration
    next_step_duration_ = cfg_.nominalStepDuration();
    walk_requested_ = false;
    mpc_ready_ = false;


    // Start the mpc thread
    controller_running_ = true;
    mpc_step_counter_ = 0;
    mpc_thread_ = std::thread(&AcadosLeggedController::mpcThreadLoop, this);

    RCLCPP_INFO(node->get_logger(), "[Acados MPC]: Activated controller. MPC Thread started.");
    
    return controller_interface::CallbackReturn::SUCCESS;
}



// Destructor 
AcadosLeggedController::~AcadosLeggedController(){
    controller_running_ = false;
    if(mpc_thread_.joinable()){
        mpc_thread_.join();
    }
}



// Deactivate
controller_interface::CallbackReturn AcadosLeggedController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/){
    controller_running_ = false;
    if(mpc_thread_.joinable()){
        mpc_thread_.join(); // wait for the closing of the thread
    }
    return controller_interface::CallbackReturn::SUCCESS;
}





// WARM START: previous solution = x0 repeated, controls at zero with dt = dt_max (as u_init in main.py)
void AcadosLeggedController::resetWarmStart(const std::vector<double>& x0){
    const int n_u_branch = 8 + 3 * n_obs_max_;
    std::vector<double> u_init(2 * n_u_branch, 0.0);
    u_init[7] = cfg_.dt_max / cfg_.step_per_phase;
    u_init[n_u_branch + 7] = cfg_.dt_max / cfg_.step_per_phase;

    x_prev_sol_.assign(n_horizon_ + 1, x0);
    u_prev_sol_.assign(n_horizon_, u_init);
}


// WARM START: port of the warm start of main.py
// The hyperplane (a, b) must start from a unit vector: with a = 0 the hard constraint 0.5 <= |a|^2 has zero
// gradient and the linearized QP is infeasible (ACADOS_MINSTEP at the first SQP iteration)
void AcadosLeggedController::setWarmStart(const std::vector<double>& x0, const GaitPlanner& gait){
    const int n_u_branch = 8 + 3 * n_obs_max_;
    const int n_x_branch = 11;

    // Shift by one node only if a new step started since the last solve (in main.py every solve was one step)
    const int shift = (gait.stepIndex() != last_step_index_) ? 1 : 0;
    last_step_index_ = gait.stepIndex();

    // Guess of the hyperplane between the closest robot point (CoM or hips) and the obstacle
    auto hyperplaneGuess = [](const double c[2], const double hips[4], const double obs[2]) {
        const double candidates[3][2] = {{c[0], c[1]}, {hips[0], hips[1]}, {hips[2], hips[3]}};
        int best = 0;
        double best_dist = 1e12;
        for(int j = 0; j < 3; ++j){
            const double d = std::hypot(obs[0] - candidates[j][0], obs[1] - candidates[j][1]);
            if(d < best_dist){ best_dist = d; best = j; }
        }
        const double* pt = candidates[best];
        const double norm = best_dist + 1e-5;
        const double ax = (obs[0] - pt[0]) / norm;
        const double ay = (obs[1] - pt[1]) / norm;
        const double b = -(ax * (pt[0] + obs[0]) / 2.0 + ay * (pt[1] + obs[1]) / 2.0);
        return std::array<double, 3>{ax, ay, b};
    };

    std::vector<double> x_guess = x0;
    for(int k = 0; k < n_horizon_; ++k){
        const int idx_x = std::min(k + shift, n_horizon_);
        const int idx_u = std::min(k + shift, n_horizon_ - 1);
        const std::vector<double>& x_prev = x_prev_sol_[idx_x];
        const std::vector<double>& u_prev = u_prev_sol_[idx_u];

        // Hip offsets of the pair placed at node k (same as the parameters)
        const std::vector<double> off = gait.hipOffsetParams(k);

        std::vector<double> u_guess(2 * n_u_branch, 0.0);
        x_guess = x_prev;

        // b = 0 optimistic, b = 1 pessimistic
        for(int br = 0; br < 2; ++br){
            const int ix = br * n_x_branch;
            const int iu = br * n_u_branch;
            const double c[2] = {x_prev[ix + 0], x_prev[ix + 1]};
            const double th = x_prev[ix + 2];
            const double ct = std::cos(th), st = std::sin(th);

            // Hips of the pair at node k (compute_hip_positions of gait_planner.py)
            const double hips[4] = {c[0] + ct * off[0] - st * off[1], c[1] + st * off[0] + ct * off[1],
                                    c[0] + ct * off[2] - st * off[3], c[1] + st * off[2] + ct * off[3]};

            // [p0_next, p1_next] under the hips, alpha = 0.5, beta = gamma = 0, dt from previous solution
            u_guess[iu + 0] = hips[0]; u_guess[iu + 1] = hips[1];
            u_guess[iu + 2] = hips[2]; u_guess[iu + 3] = hips[3];
            u_guess[iu + 4] = 0.5;
            u_guess[iu + 7] = (k == 0) ? std::max(gait.timeLeft(), 1e-3) : u_prev[iu + 7];  // node 0: dt fixed to the time left

            for(int i = 0; i < n_obs_max_; ++i){
                const double obs[2] = {kGhostObsPos + 2.0 * i, kGhostObsPos};
                const auto ab = hyperplaneGuess(c, hips, obs);
                u_guess[iu + 8 + 3 * i + 0] = ab[0];
                u_guess[iu + 8 + 3 * i + 1] = ab[1];
                u_guess[iu + 8 + 3 * i + 2] = ab[2];
            }

            // State guess: CoM/vel from the previous solution, feet from x_prev, time = dt
            x_guess[ix + 10] = u_prev[iu + 7];
        }

        // Node 0: the 8 physical controls must be equal in the two branches -> use the pessimistic ones (as main.py)
        if(k == 0){
            std::copy(u_guess.begin() + n_u_branch, u_guess.begin() + n_u_branch + 8, u_guess.begin());
        }

        // Node 0 state is fixed by x0, only the controls are needed
        mpc_solver_->setGuess(k, (k > 0) ? x_guess : std::vector<double>{}, u_guess);
    }
    mpc_solver_->setGuess(n_horizon_, x_guess, {});
}


// Thread of the MPC: async mpc
void AcadosLeggedController::mpcThreadLoop(){
    bool mpc_active = false; // warm start is rebuilt each time MPC starts

    while(controller_running_){

        // Manage the phase
        if (phase_ != Phase::WALK && !walk_requested_) {
            mpc_active = false;
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        if (!mpc_active){
            std::lock_guard<std::mutex> lock(mpc_mutex_);
            resetWarmStart(current_state_x0_);
            last_step_index_ = gait_planner_->stepIndex();
            mpc_active = true;
        }


        // Actual state of the robot
        std::vector<double> x0_local;
        GaitPlanner gait_local(0, 0, 0);
        {
            std::lock_guard<std::mutex> lock(mpc_mutex_);
            x0_local = current_state_x0_;
            gait_local = *gait_planner_; // copy
        }

        // Set the initial state and parameters
        mpc_solver_->setInitialState(x0_local);
        for(int k = 0; k <= n_horizon_; ++k){
            std::vector<double> p = gait_local.hipOffsetParams(k);
            for(int i = 0; i < n_obs_max_; ++i){
                // Ghost obstacle for now (same far-away placeholder used in main.py, not a physical parameter)
                p.insert(p.end(), {kGhostObsPos + 2.0 * i, kGhostObsPos, 0.0, 0.0});
            }
            mpc_solver_->setParameters(k, p); // FIX: missing ';'
        }
        mpc_solver_->setFirstStepDuration(gait_local.timeLeft());


        // LOCAL TARGET: at most max_lookahead from robot
        const double dir_x = cfg_.c_target_x - x0_local[0];
        const double dir_y = cfg_.c_target_y - x0_local[1];
        const double dist = std::hypot(dir_x, dir_y);
        double local_x = cfg_.c_target_x;
        double local_y = cfg_.c_target_y;
        if (dist > cfg_.max_lookahead) {
            local_x = x0_local[0] + dir_x / dist * cfg_.max_lookahead;
            local_y = x0_local[1] + dir_y / dist * cfg_.max_lookahead;
        }

        // yref: index 0-1 optimistic CoM, 17-18 pessimistic CoM, yref_e: 0-1 and 6-7
        std::vector<double> yref(34, 0.0);
        yref[0] = local_x; yref[1] = local_y;
        yref[17] = local_x; yref[18] = local_y;
        std::vector<double> yref_e(12, 0.0);
        yref_e[0] = local_x; yref_e[1] = local_y;
        yref_e[6] = local_x; yref_e[7] = local_y;
        for (int k = 0; k < n_horizon_; ++k) mpc_solver_->setReference(k, yref);
        mpc_solver_->setReference(n_horizon_, yref_e);



        // WARM START: initial guess for x and u
        setWarmStart(x0_local, gait_local);

        // Solve the ocp
        int status = mpc_solver_->solve();

        // publish the solution only if the solver succeeded (0) or reached max iter (2)
        if(status != 0 && status != 2){
            // WARM START: a failed solution is not a good guess, restart from x0
            resetWarmStart(x0_local);
            RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                                 "[Acados MPC]: solver failed with status %d", status);
            continue;
        }

        // WARM START: store the whole solution for the next solve
        for(int k = 0; k <= n_horizon_; ++k){
            x_prev_sol_[k] = mpc_solver_->getStatePrediction(k);
            if(k < n_horizon_) u_prev_sol_[k] = mpc_solver_->getControl(k);
        }

        

        std::vector<double> u_opt = mpc_solver_->getControlCommand();
        std::vector<double> x_next = mpc_solver_->getStatePrediction(1);

        {
            std::lock_guard<std::mutex> lock(mpc_mutex_);
            latest_u_opt_ = u_opt;
            latest_x_opt_ = x_next;
            // Save the duration of the next step
            next_step_duration_ = std::clamp(u_prev_sol_[1][7], cfg_.dt_min, cfg_.dt_max);
        }

        mpc_ready_ = true;
        RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
            "[Acados MPC]: status=%d, solve=%.1f ms, T_next=%.3f, cop=%.2f", 
            status, mpc_solver_->getComputationTime() * 1000.0, next_step_duration_, u_opt[4]);

        mpc_step_counter_++;
    }
}





// update(): 
/*
    1) read the real state from sensors
    2) extract the controls from tha lipm_model and formats it in WBC's way
    3) send commands to the motors 
*/
controller_interface::return_type AcadosLeggedController::update(const rclcpp::Time& time, const rclcpp::Duration& period){ // FIX: preiod -> period
    // update the estimate of the actual state starting from sensors
    updateStateEstimation(time, period);

    
    // Manage the phase
    phase_time_ += period.seconds();

    if (phase_ == Phase::STAND_UP){
        const double x = std::min(phase_time_ / cfg_.standup_duration, 1.0);
        const double s = x * x * (3.0 - 2.0 * x);    // smoothstep: zero velocity at start and end
        if (x >= 1.0) {
            phase_ = Phase::STANCE;
            phase_time_ = 0.0;
            base_target_ = currentObservation_.state.segment<6>(6); // x, y, z, yaw measured
            base_target_(4) = 0.0; // pitch
            base_target_(5) = 0.0; // roll
            z_start_ = base_target_(2);
            RCLCPP_INFO_STREAM(get_node()->get_logger(), "[Acados MPC]: STANCE, base target: " << base_target_.transpose());
        }
        for (size_t j = 0; j < hybridJointHandles_.size(); ++j) {
            const double q_des = (1.0 - s) * q_start_(j) + s * q_stand_(j);
            hybridJointHandles_[j].setCommand(q_des, 0.0, cfg_.kp_stand, cfg_.kd_stand, 0.0);
        }
        return controller_interface::return_type::OK;
    }
    if (phase_ == Phase::STANCE){
        const double r = std::min(phase_time_ / cfg_.stance_blend, 1.0);
        const double r_s = r * r * (3.0 - 2.0 * r);
        const double z_des = (1.0 - r_s) * z_start_ + r_s * cfg_.h_com; 

        const auto& info = leggedInterface_->getCentroidalModelInfo();
        const vector_t& s = currentObservation_.state;  // centroidal: [vcom(3), L/m(3), pos(3), zyx(3), q(12)]

        // 1) desired state: target_pose, zero momentum, joint at q_stand
        vector_t optimizedState = vector_t::Zero(info.stateDim);
        optimizedState.segment<6>(6) = base_target_;
        optimizedState(8) = z_des;
        optimizedState.segment<12>(12) = q_stand_;

        // 2) Base PD -> total force, split equally on 4 feet
        const double m = info.robotMass;
        const double g = cfg_.g;
        vector3_t acc;
        acc.x() = cfg_.kp_base_xy * (base_target_(0) - s(6)) - cfg_.kd_base_xy * s(0);
        acc.y() = cfg_.kp_base_xy * (base_target_(1) - s(7)) - cfg_.kd_base_xy * s(1);
        acc.z() = cfg_.kp_base_z  * (z_des - s(8)) - cfg_.kd_base_z * s(2);
        vector3_t f_foot = m * (acc + vector3_t(0, 0, g)) / 4.0;

        vector_t optimizedInput = vector_t::Zero(info.inputDim);
        for (size_t i = 0; i < 4; ++i) optimizedInput.segment<3>(3 * i) = f_foot; // joint velocities stay 0
        currentObservation_.input = optimizedInput;

    

        // 3) WBC with 4 feet in contact
        vector_t wbc_output = wbc_->update(optimizedState, optimizedInput, measuredRbdState_, ModeNumber::STANCE, period.seconds());
        vector_t torque = wbc_output.tail(12);
        
        // 4) Safety checker active from here on
        if (!safetyChecker_->check(currentObservation_, optimizedState, optimizedInput)) {
            RCLCPP_ERROR(get_node()->get_logger(), "[Acados MPC]: Safety check failed!!");
            return controller_interface::return_type::ERROR;
        }
    
        // 5) Joint PD (gain ramp from stand_up to stance) + WBC torque
        const double kp = (1.0 - r) * cfg_.kp_stand + r * cfg_.kp_stance;
        const double kd = (1.0 - r) * cfg_.kd_stand + r * cfg_.kd_stance;
        for (size_t j = 0; j < info.actuatedDofNum; ++j) {
            hybridJointHandles_[j].setCommand(q_stand_(j), 0.0, kp, kd, torque(j));
        }

        // RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
        // "z=%.3f/%.3f  xy_err=%.3f %.3f  pitch=%.3f roll=%.3f  fz=%.1f  tau_max=%.1f",
        // s(8), z_des, base_target_(0) - s(6), base_target_(1) - s(7), s(10), s(11),
        // f_foot.z(), torque.cwiseAbs().maxCoeff());

        return controller_interface::return_type::OK;
    }
    // Uncomment at step 6
    // if (phase_ == Phase::STANCE && walk_requested_ && mpc_ready_) phase_ = Phase::WALK;


    // Take the last available solution from the MPC
    std::vector<double> u_opt;
    std::vector<double> x_next;
    size_t current_mode;
    GaitPlanner::Pair stance;
    std::array<double, 4> feet;
    {
        std::lock_guard<std::mutex> lock(mpc_mutex_);
        u_opt = latest_u_opt_;
        x_next = latest_x_opt_;
        stance = gait_planner_->stancePair();
        current_mode = gait_planner_->mode();
        feet = stance_feet_xy_;
    }

    // Safety: if MPC has not started yet, stay put
    if(u_opt.empty()){
        return controller_interface::return_type::OK;
    }


    // Translate the output of the LIPM in the input of the WBC
    // WBC expect 'optimizedState' (dim = 24) e 'optimizedInput' (dim = 24)
    // optimizedState = [h_com (3), L_com (3), q_base (6), q_joints (12)]
    // optimizedInput = [f_feet (12), v_joints (12)]

    // CONSTRUCT THE STATE
    vector_t optimizedState(24);
    optimizedState.setZero();

    // Taret CoM from latest_x_opt
    optimizedState(6) = x_next[0];  // CoM X
    optimizedState(7) = x_next[1];  // CoM Y
    optimizedState(8) = cfg_.h_com; // Desired height CoM (CONFIG: from config.yaml)

    // Target Velocity CoM
    optimizedState(0) = x_next[3];  // Vel X
    optimizedState(1) = x_next[4];  // Vel Y
    optimizedState(2) = 0.0;        // Vel Z

    // Target Yaw
    optimizedState(9) = x_next[2];  // Yaw

    // Joint posture (keep the actual ones as reference for the WBC)
    // Just a reference, because the whole body control has higher priority on tracking the CoM and forces and the kinematic of the swing
    // measureRbdState_ -> measuredRbdState_; rbd state = [base(6), joints(12), base vel(6), joint vel(12)] -> joints start at 6
    optimizedState.segment<12>(12) = measuredRbdState_.segment<12>(6);



    // CONSTRUCT THE INPUT
    vector_t optimizedInput(24);
    optimizedInput.setZero();


    // Extract the alpha, beta, gamma inputs
    double alpha = u_opt[4];
    double beta = u_opt[5];
    double gamma = u_opt[6];


    // Compute the dynamic
    // CONFIG: from config.yaml
    const double mass = cfg_.m;
    const double g = cfg_.g;
    const double h_com = cfg_.h_com;

    double u_cop_x = feet[0] + alpha * (feet[2] - feet[0]);
    double u_cop_y = feet[1] + alpha * (feet[3] - feet[1]);

    double c_x = x_next[0];
    double c_y = x_next[1];

    double c_ddot_x = (g / h_com) * (c_x - u_cop_x);
    double c_ddot_y = (g / h_com) * (c_y - u_cop_y);

    // Compute the forces on the support feet
    double f0x = beta * mass * c_ddot_x;
    double f0y = gamma * mass * c_ddot_y;
    double f0z = (1.0 - alpha) * mass * g;

    double f1x = (1.0 - beta) * mass * c_ddot_x;
    double f1y = (1.0 - gamma) * mass * c_ddot_y;
    double f1z = alpha * mass * g;


    // Mapping the actual force on the real support feet
    optimizedInput.segment<3>(3 * stance[0]) << f0x, f0y, f0z;
    optimizedInput.segment<3>(3 * stance[1]) << f1x, f1y, f1z;


    // Execute the WBC: compute the torques of the motors
    currentObservation_.input = optimizedInput; // For visualization and checks

    // wbc_->update compute the torques and target position of the 12 motors
    vector_t wbc_output = wbc_->update(optimizedState, optimizedInput, measuredRbdState_, current_mode, period.seconds());


    // Safety checker for real motors (or Gazebo)
    if(!safetyChecker_->check(currentObservation_, optimizedState, optimizedInput)){
        RCLCPP_ERROR(this->get_node()->get_logger(), "[Acados MPC]: Safety check failed!!");
        return controller_interface::return_type::ERROR;
    }

    const auto& info = leggedInterface_->getCentroidalModelInfo();
    vector_t torque = wbc_output.tail(12);
    vector_t posDes = centroidal_model::getJointAngles(optimizedState, info);
    vector_t velDes = centroidal_model::getJointVelocities(optimizedInput, info);
    for(size_t j = 0; j < info.actuatedDofNum; ++j){
        hybridJointHandles_[j].setCommand(posDes(j), velDes(j), 0, 3, torque(j));
    }




    return controller_interface::return_type::OK;
}





// Utility function (identical to OCS2s) to update the estimate of the state
void AcadosLeggedController::updateStateEstimation(const rclcpp::Time& time, const rclcpp::Duration& period) { // FIX: preiod -> period
    vector_t jointPos(hybridJointHandles_.size()), jointVel(hybridJointHandles_.size()); // FIX: missing '_'
    contact_flag_t contactFlag;
    Eigen::Quaternion<scalar_t> quat;
    vector3_t angularVel, linearAccel;
    matrix3_t orientationCovariance, angularVelCovariance, linearAccelCovariance; // FIX: matrix3_T -> matrix3_t

    for(size_t i = 0; i < hybridJointHandles_.size(); ++i){
        jointPos(i) = hybridJointHandles_[i].getPosition();
        jointVel(i) = hybridJointHandles_[i].getVelocity();
    }
    for(size_t i = 0; i < contactHandles_.size(); ++i){
        if(contactHandles_[i].contactState){
            contactFlag[i] = contactHandles_[i].isContact();
        }else{
            contactFlag[i] = false;
        }
    }
    const double* ori = imuSensorHandle_.getOrientation();
    quat.coeffs() << ori[0], ori[1], ori[2], ori[3];

    const double* angVel = imuSensorHandle_.getAngularVelocity();
    angularVel << angVel[0], angVel[1], angVel[2]; // FIX: vector3_t has no coeffs()

    const double* linAcc = imuSensorHandle_.getLinearAcceleration();
    linearAccel << linAcc[0], linAcc[1], linAcc[2]; // FIX: vector3_t has no coeffs()


    orientationCovariance.setZero();
    angularVelCovariance.setZero();
    linearAccelCovariance.setZero();

    stateEstimate_->updateJointStates(jointPos, jointVel);
    stateEstimate_->updateContact(contactFlag);
    stateEstimate_->updateImu(quat, angularVel, linearAccel, orientationCovariance, angularVelCovariance, linearAccelCovariance);

    measuredRbdState_ = stateEstimate_->update(time, period);
    currentObservation_.time += period.seconds();


    // Update the observation (centroidal estimate state)
    // Use the member rbdConversions_ (created in on_init, not at every cycle)
    scalar_t yawLast = currentObservation_.state(9);
    currentObservation_.state = rbdConversions_->computeCentroidalStateFromRbdModel(measuredRbdState_);
    currentObservation_.state(9) = yawLast + angles::shortest_angular_distance(yawLast, currentObservation_.state(9)); // FIX: shortes_ -> shortest_
    currentObservation_.mode = stateEstimate_->getMode(); // FIX: currentObservation_mode -> currentObservation_.mode



    // MAPPING FROM OCS2 Centroidal State to LIPM x0 STATE
    // OCS2 centroidal state: 
    /*
        [0-2]: Normalized Linear Moment (velocity of the CoM)
        [3-5]: NOrmalized angular moment
        [6-8]: Base Position (CoM x, y, z)
        [9-11]: Base Orientation (yaw, pitch, roll)
        [12-23]: Position of the 12 joints
    */ 
    vector_t state = currentObservation_.state;

    std::vector<double>     x0_lipm(22, 0.0);

    // CoM data (optimistic)
    x0_lipm[0] = state(6); // CoM X
    x0_lipm[1] = state(7); // CoM Y
    x0_lipm[2] = state(9); // Yaw
    x0_lipm[3] = state(0); // Vel CoM X
    x0_lipm[4] = state(1); // Vel CoM Y
    // state(5) is the normalized angular momentum L_z/m -> omega_z = L_z / Iz = state(5) * m / Iz (m, Iz from config.yaml)
    x0_lipm[5] = state(5) * cfg_.m / cfg_.Iz; // Vel Yaw (omega_z) (CONFIG: m, Iz from config.yaml)

    // CoM data (pessimistic) (the same initially)
    x0_lipm[11] = x0_lipm[0];
    x0_lipm[12] = x0_lipm[1];
    x0_lipm[13] = x0_lipm[2];
    x0_lipm[14] = x0_lipm[3];
    x0_lipm[15] = x0_lipm[4];
    x0_lipm[16] = x0_lipm[5];


    // Compute the actual position of the feet with forward kinematic
    // getPosition() reads pinocchio data, so the forward kinematics must be computed first
    auto& pinocchioInterface = leggedInterface_->getPinocchioInterface();
    const vector_t q = centroidal_model::getGeneralizedCoordinates(state, leggedInterface_->getCentroidalModelInfo());
    pinocchio::forwardKinematics(pinocchioInterface.getModel(), pinocchioInterface.getData(), q);
    pinocchio::updateFramePlacements(pinocchioInterface.getModel(), pinocchioInterface.getData());
    auto feet_positions = eeKinematicsPtr_->getPosition(state);



    // GaitPlanner logic
    GaitPlanner::Pair stance;
    {
        std::lock_guard<std::mutex> lock(mpc_mutex_);
        if(phase_ == Phase::WALK && gait_planner_->advance(period.seconds())){
            gait_planner_->setStepDuration(next_step_duration_); // Duration of the step proposed by the MPC
        }
        stance = gait_planner_->stancePair();
    }

    x0_lipm[6] = feet_positions[stance[0]].x();
    x0_lipm[7] = feet_positions[stance[0]].y();
    x0_lipm[8] = feet_positions[stance[1]].x();
    x0_lipm[9] = feet_positions[stance[1]].y();
    // only the horizon time (10, 21) is reset; 11..20 were overwritten with zeros, deleting the pessimistic CoM
    x0_lipm[10] = 0.0;
    x0_lipm[21] = 0.0;


    // Copy for the pessimistic
    x0_lipm[17] = x0_lipm[6];
    x0_lipm[18] = x0_lipm[7];
    x0_lipm[19] = x0_lipm[8];
    x0_lipm[20] = x0_lipm[9];

    // Upgrading shared variable
    {
        std::lock_guard<std::mutex> lock(mpc_mutex_);
        current_state_x0_ = x0_lipm;
        stance_feet_xy_ = {x0_lipm[6], x0_lipm[7], x0_lipm[8], x0_lipm[9]};
    }

}



} // namespace legged




// register the controller as a ros2_control plugin
PLUGINLIB_EXPORT_CLASS(legged::AcadosLeggedController, controller_interface::ControllerInterface)
