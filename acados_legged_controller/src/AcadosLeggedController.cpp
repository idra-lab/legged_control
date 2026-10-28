// FIX: pinocchio forward declarations must be included before everything else
#include <pinocchio/fwd.hpp>

#include "acados_legged_controller/AcadosLeggedController.hpp"
#include <pluginlib/class_list_macros.hpp>

// ocs2 helper to load the kinematic for WBC from pinocchio
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>

// FIX: missing includes (angles::shortest_angular_distance, getJointAngles/getGeneralizedCoordinates, forward kinematics)
#include <angles/angles.h>
#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>


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

    std::string urdfFile = node->get_parameter("urdfFile").as_string();
    std::string taskFile = node->get_parameter("taskFile").as_string();
    std::string referenceFile = node->get_parameter("referenceFile").as_string();
    bool verbose = true;

    // Initialize the ocs2 interface only for reading the phisics of the robot
    leggedInterface_ = std::make_shared<LeggedInterface>(taskFile, urdfFile, referenceFile);
    leggedInterface_->setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose);

    // FIX: helper node (the Kalman filter needs an rclcpp::Node, before it was never created)
    ros2_node_ = std::make_shared<rclcpp::Node>("acados_legged_controller_node");

    // Initialize acados MPC solver
    // FIX: store the dimensions in the members (used by mpcThreadLoop)
    n_obs_max_ = 1;
    n_horizon_ = 6;
    mpc_solver_ = std::make_shared<MpcSolver>(n_obs_max_, n_horizon_);
    RCLCPP_INFO(node->get_logger(), "[Acados MPC]: Solver initialized with success!!");

    // FIX: gait planner was never created (off_x, off_y, nominal step duration from config.yaml)
    gait_planner_ = std::make_shared<GaitPlanner>(0.2407, 0.134, (0.1 + 0.25) / 2.0);


    // Setup of the kinematic, state estimate and Whole Body Control
    CentroidalModelPinocchioMapping pinocchioMapping(leggedInterface_->getCentroidalModelInfo());
    eeKinematicsPtr_ = std::make_shared<PinocchioEndEffectorKinematics>(
        leggedInterface_->getPinocchioInterface(), pinocchioMapping,
        leggedInterface_->modelSettings().contactNames3DoF);
    // FIX: without this getPosition() throws "pinocchioInterfacePtr_ is not set"
    eeKinematicsPtr_->setPinocchioInterface(leggedInterface_->getPinocchioInterface());

    // FIX: created once here instead of at every cycle
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


    // Initialization of the shared data
    current_state_x0_.assign(22, 0.0);
    latest_x_opt_.assign(22, 0.0);
    // latest_u_opt_ will be resized in the first solve()

    // FIX: initialize the observation (state(9) is read in updateStateEstimation) and do a first
    // estimation, so the MPC thread does not start from x0 = 0
    currentObservation_.state.setZero(leggedInterface_->getCentroidalModelInfo().stateDim);
    currentObservation_.input.setZero(leggedInterface_->getCentroidalModelInfo().inputDim);
    currentObservation_.mode = ModeNumber::STANCE;
    updateStateEstimation(node->get_clock()->now(), rclcpp::Duration::from_seconds(0.002));


    // Start the mpc thread
    controller_running_ = true;
    mpc_step_counter_ = 0;
    mpc_thread_ = std::thread(&AcadosLeggedController::mpcThreadLoop, this);

    RCLCPP_INFO(node->get_logger(), "[Acados MPC]: Activated controller. MPC Thread started.");
    
    return controller_interface::CallbackReturn::SUCCESS;
}



// FIX: destructor was declared but never defined (linker error)
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





// Thread of the MPC: async mpc
void AcadosLeggedController::mpcThreadLoop(){
    while(controller_running_){
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
                // Ghost obstacle for now
                p.insert(p.end(), {15.0 + 2.0 * i, 15.0, 0.0, 0.0});
            }
            mpc_solver_->setParameters(k, p); // FIX: missing ';'
        }
        mpc_solver_->setFirstStepDuration(gait_local.timeLeft());

        // Solve the ocp
        int status = mpc_solver_->solve();

        // FIX: publish the solution only if the solver succeeded (0) or reached max iter (2)
        if(status != 0 && status != 2){
            continue;
        }

        std::vector<double> u_opt = mpc_solver_->getControlCommand();
        std::vector<double> x_next = mpc_solver_->getStatePrediction(1);

        {
            std::lock_guard<std::mutex> lock(mpc_mutex_);
            latest_u_opt_ = u_opt;
            latest_x_opt_ = x_next;
        }

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


    // Take the last available solution from the MPC
    // FIX: std:. -> std::, mpc_mnutex_ -> mpc_mutex_, merged with the GaitPlanner block (current_planned_mode_ did not exist)
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
    optimizedState(8) = 0.38;       // Desired height CoM: FROM config.yaml

    // Target Velocity CoM
    optimizedState(0) = x_next[3];  // Vel X
    optimizedState(1) = x_next[4];  // Vel Y
    optimizedState(2) = 0.0;        // Vel Z

    // Target Yaw
    optimizedState(9) = x_next[2];  // Yaw

    // Joint posture (keep the actual ones as reference for the WBC)
    // Just a reference, because the whole body control has higher priority on tracking the CoM and forces and the kinematic of the swing
    // FIX: measureRbdState_ -> measuredRbdState_; rbd state = [base(6), joints(12), base vel(6), joint vel(12)] -> joints start at 6
    optimizedState.segment<12>(12) = measuredRbdState_.segment<12>(6);





    // CONSTRUCT THE INPUT
    vector_t optimizedInput(24);
    optimizedInput.setZero();

    // FIX: GaitPlanner block moved at the beginning of update() (it was a duplicate of the first lock)

    // Extract the alpha, beta, gamma inputs
    double alpha = u_opt[4];
    double beta = u_opt[5];
    double gamma = u_opt[6];

    // FIX: removed p0_next/p1_next (unused, the CoP uses the current stance feet)

    // Compute the dynamic
    double mass = 24.24;
    double g = 9.81;
    double h_com = 0.38;

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
    // FIX: writeHardwareCommand does not exist -> same logic of LeggedController::update, safetyChecker_ is a member
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
    // FIX: use the member rbdConversions_ (created in on_init, not at every cycle)
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
    // FIX: state(5) is the normalized angular momentum L_z/m -> omega_z = L_z / Iz = state(5) * m / Iz (m, Iz from config.yaml)
    x0_lipm[5] = state(5) * 24.24 / 1.048; // Vel Yaw (omega_z)

    // CoM data (pessimistic) (the same initially)
    x0_lipm[11] = x0_lipm[0];
    x0_lipm[12] = x0_lipm[1];
    x0_lipm[13] = x0_lipm[2];
    x0_lipm[14] = x0_lipm[3];
    x0_lipm[15] = x0_lipm[4];
    x0_lipm[16] = x0_lipm[5];


    // Compute the actual position of the feet with forward kinematic
    // FIX: getPosition() reads pinocchio data, so the forward kinematics must be computed first
    auto& pinocchioInterface = leggedInterface_->getPinocchioInterface();
    const vector_t q = centroidal_model::getGeneralizedCoordinates(state, leggedInterface_->getCentroidalModelInfo());
    pinocchio::forwardKinematics(pinocchioInterface.getModel(), pinocchioInterface.getData(), q);
    pinocchio::updateFramePlacements(pinocchioInterface.getModel(), pinocchioInterface.getData());
    auto feet_positions = eeKinematicsPtr_->getPosition(state);



    // GaitPlanner logic
    GaitPlanner::Pair stance;
    {
        std::lock_guard<std::mutex> lock(mpc_mutex_);
        if(gait_planner_->advance(period.seconds()) && !latest_u_opt_.empty()){
            gait_planner_->setStepDuration(latest_u_opt_[7]); // Duration of the step proposed by the MPC
        }
        stance = gait_planner_->stancePair();
    }

    x0_lipm[6] = feet_positions[stance[0]].x();
    x0_lipm[7] = feet_positions[stance[0]].y();
    x0_lipm[8] = feet_positions[stance[1]].x();
    x0_lipm[9] = feet_positions[stance[1]].y();
    // FIX: only the horizon time (10, 21) is reset; 11..20 were overwritten with zeros, deleting the pessimistic CoM
    x0_lipm[10] = 0.0;
    x0_lipm[21] = 0.0;



    // Copy for the pessimistic
    x0_lipm[17] = x0_lipm[6];
    x0_lipm[18] = x0_lipm[7];
    x0_lipm[19] = x0_lipm[8];
    x0_lipm[20] = x0_lipm[9];

    // Upgrading shared variable
    // FIX: removed old t_phase comments (timing is managed by the GaitPlanner), stance_feet_xy_ was never written
    {
        std::lock_guard<std::mutex> lock(mpc_mutex_);
        current_state_x0_ = x0_lipm;
        stance_feet_xy_ = {x0_lipm[6], x0_lipm[7], x0_lipm[8], x0_lipm[9]};
    }

}



} // namespace legged







// FIX: register the controller as a ros2_control plugin
PLUGINLIB_EXPORT_CLASS(legged::AcadosLeggedController, controller_interface::ControllerInterface)
