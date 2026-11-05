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
#include <pinocchio/algorithm/center-of-mass.hpp>

// Swing splines and ZYX Euler rates, as in OptiPessiController
#include <ocs2_legged_robot/foot_planner/CubicSpline.h>
#include <ocs2_legged_robot/foot_planner/SplineCpg.h>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>

// WARM START: std::cos/sin/hypot, std::min/max/copy
#include <cmath>
#include <algorithm>
#include <chrono>
// CONFIG: default path of config.yaml (installed in share/acados_legged_controller/config)
#include <ament_index_cpp/get_package_share_directory.hpp>


namespace legged {

namespace {

// Swing foot at `time` in [0, duration] (OptiPessiController::evaluateSwing): cubic in x and y with zero end
// velocities, SplineCpg in z through an apex swing_height above the higher end at mid-swing
void evaluateSwing(const vector3_t& liftoff, const vector3_t& touchdown, scalar_t duration, scalar_t swingHeight,
                   scalar_t time, vector3_t& position, vector3_t& velocity){
    const CubicSpline splineX({0.0, liftoff.x(), 0.0}, {duration, touchdown.x(), 0.0});
    const CubicSpline splineY({0.0, liftoff.y(), 0.0}, {duration, touchdown.y(), 0.0});
    const SplineCpg splineZ({0.0, liftoff.z(), 0.0}, std::max(liftoff.z(), touchdown.z()) + swingHeight, {duration, touchdown.z(), 0.0});
    position << splineX.position(time), splineY.position(time), splineZ.position(time);
    velocity << splineX.velocity(time), splineY.velocity(time), splineZ.velocity(time);
}

// Contact forces of the stance pair (p0, p1) of one LIPM knot (OptiPessiController::computeContactForces):
// tangential beta/gamma split of m ddc as in lipm_model.py, normal m g shared so that the CoP sits at alpha.
// x = [cx, cy, th, dcx, dcy, dth, p0x, p0y, p1x, p1y, t], u = [p0nx, p0ny, p1nx, p1ny, alpha, beta, gamma, dt, ...]
std::array<vector3_t, 2> computeContactForces(const std::vector<double>& x, const std::vector<double>& u, double m, double g, double w){
    const Eigen::Vector2d c(x[0], x[1]), p0(x[6], x[7]), p1(x[8], x[9]);
    const double alpha = u[4], beta = u[5], gamma = u[6];
    const Eigen::Vector2d cop = p0 + alpha * (p1 - p0);
    const Eigen::Vector2d F = m * w * w * (c - cop);
    std::array<vector3_t, 2> forces;
    forces[0] << beta * F.x(), gamma * F.y(), (1.0 - alpha) * m * g;
    forces[1] << (1.0 - beta) * F.x(), (1.0 - gamma) * F.y(), alpha * m * g;
    return forces;
}

// Yaw torque of the tangential forces of the stance pair about the CoM of x (tau_z of lipm_model.py)
double yawTorque(const std::vector<double>& x, const std::array<vector3_t, 2>& forces){
    return (x[6] - x[0]) * forces[0].y() - (x[7] - x[1]) * forces[0].x()
         + (x[8] - x[0]) * forces[1].y() - (x[9] - x[1]) * forces[1].x();
}

} // namespace


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
    // Obstacles of config.yaml: simulated, the lidar does not see them (false when exploring a real place)
    if (!node->has_parameter("simulatedObstacles")){
        node->declare_parameter<bool>("simulatedObstacles", true);
    }
    // HEADING: reach the yaw of /goal_pose (false with the exploration planners, their goals are waypoints)
    if (!node->has_parameter("useGoalYaw")){
        node->declare_parameter<bool>("useGoalYaw", true);
    }

    // OBSTACLE TOPIC: obstacles from legged_controllers/ObstacleArray (perception or obstacle_simulator) instead of config.yaml
    if (!node->has_parameter("obstaclesFromTopic")){
        node->declare_parameter<bool>("obstaclesFromTopic", false);
    }
    if (!node->has_parameter("obstaclesTopic")){
        node->declare_parameter<std::string>("obstaclesTopic", "/opti_pessi/obstacles");
    }
    if (!node->has_parameter("obstaclesTimeout")){
        node->declare_parameter<double>("obstaclesTimeout", 0.5);
    }
    if (!node->has_parameter("obstaclesMaxAge")){
        node->declare_parameter<double>("obstaclesMaxAge", 2.0);
    }




    std::string urdfFile = node->get_parameter("urdfFile").as_string();
    std::string taskFile = node->get_parameter("taskFile").as_string();
    std::string referenceFile = node->get_parameter("referenceFile").as_string();
    std::string lipmConfigFile = node->get_parameter("lipmConfigFile").as_string();
    bool verbose = true;

    // CONFIG: empty paths use the installed files
    const std::string configDir = ament_index_cpp::get_package_share_directory("acados_legged_controller") + "/config/";
    if (taskFile.empty()){
        taskFile = configDir + "task.info";
    }
    if (referenceFile.empty()){
        referenceFile = configDir + "reference.info";
    }
    if (lipmConfigFile.empty()){
        lipmConfigFile = configDir + "config.yaml";
    }

    // CONFIG: load config.yaml
    try{
        cfg_ = LipmConfig::fromYaml(lipmConfigFile);
    }catch(const std::exception& e){
        RCLCPP_ERROR(node->get_logger(), "%s", e.what());
        return controller_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(node->get_logger(), "[Acados MPC]: Loaded config from %s", lipmConfigFile.c_str());
    obstacles_from_topic_ = node->get_parameter("obstaclesFromTopic").as_bool();
    obstacles_topic_ = node->get_parameter("obstaclesTopic").as_string();
    obstacles_timeout_ = node->get_parameter("obstaclesTimeout").as_double();
    obstacles_max_age_ = node->get_parameter("obstaclesMaxAge").as_double();
    // From the topic the obstacles of config.yaml are not simulated here (obstacle_simulator does it)
    if (obstacles_from_topic_ || !node->get_parameter("simulatedObstacles").as_bool()){
        cfg_.obstacles.clear();
        cfg_.random_obstacles.count = 0;
        RCLCPP_INFO(node->get_logger(), "[Acados MPC]: simulated obstacles off");
    }
    if (obstacles_from_topic_){
        RCLCPP_INFO(node->get_logger(), "[Acados MPC]: obstacles from %s", obstacles_topic_.c_str());
    }

    use_goal_yaw_ = node->get_parameter("useGoalYaw").as_bool();
    RCLCPP_INFO(node->get_logger(), "[Acados MPC]: goal yaw %s", use_goal_yaw_ ? "on" : "off (face the velocity only)");

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
        obstacle_manager_ = std::make_shared<ObstacleManager>(cfg_);
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
    
    
    wbc_ = std::make_shared<OptiPessiWbc>(
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

    goal_x_ = cfg_.c_target_x;
    goal_y_ = cfg_.c_target_y;
    goal_theta_ = cfg_.theta_target;
    goal_has_yaw_ = use_goal_yaw_;
    using PoseStamped = geometry_msgs::msg::PoseStamped;
    goal_sub_ = get_node()->create_subscription<PoseStamped>("/goal_pose", 10,
        [this](const PoseStamped::SharedPtr msg) {
            // HEADING: yaw of the quaternion, a zero quaternion means orientation not set
            const auto& q = msg->pose.orientation;
            const double q_norm = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
            const bool has_yaw = use_goal_yaw_ && q_norm > 0.5;
            if (has_yaw) {
                goal_theta_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
            }
            goal_x_ = msg->pose.position.x;
            goal_y_ = msg->pose.position.y;
            goal_has_yaw_ = has_yaw;
            if (has_yaw) {
                RCLCPP_INFO(get_node()->get_logger(), "[Acados MPC]: New goal [%.2f, %.2f], yaw %.2f",
                            msg->pose.position.x, msg->pose.position.y, goal_theta_.load());
            } else {
                RCLCPP_INFO(get_node()->get_logger(), "[Acados MPC]: New goal [%.2f, %.2f]", msg->pose.position.x, msg->pose.position.y);
            }
            if (phase_ == Phase::STANCE) walk_requested_ = true; // walk from STANCE
        });


    // Obstacle topic: each message replaces the set (an empty one clears it), only odom (no TF here)
    if (obstacles_from_topic_) {
        using ObstacleArray = legged_controllers::msg::ObstacleArray;
        using ObstacleMsg = legged_controllers::msg::Obstacle;
        obstacles_sub_ = get_node()->create_subscription<ObstacleArray>(obstacles_topic_, rclcpp::QoS(1), 
            [this](const ObstacleArray::SharedPtr msg) {
                if (!msg->header.frame_id.empty() && msg->header.frame_id != "odom") {
                    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 5000, 
                    "[Acados MPC]: obstacles in frame '%s' ignored: expected odom", msg->header.frame_id.c_str());
                    return;
                }
                std::vector<ObstacleState> obstacles;
                for (const auto& obs : msg->obstacles) {
                    const ObstacleTypeConfig* type = nullptr;
                    if (obs.type == ObstacleMsg::HUMAN) type = &cfg_.human_type;
                    else if (obs.type == ObstacleMsg::CAR) type = &cfg_.car_type;
                    else {
                        RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 5000,
                            "[Acados MPC]: obstacle of unknown type %u ignored", static_cast<unsigned>(obs.type));
                        continue;
                    }
                    obstacles.push_back({obs.position.x, obs.position.y, type->radius, type->y_dot_max});
                }
                std::lock_guard<std::mutex> lock(obstacles_mutex_);
                topic_obstacles_ = std::move(obstacles);
                topic_obstacles_stamp_ = rclcpp::Time(msg->header.stamp).seconds();
                topic_obstacles_received_ = true;
            });
    }

    
    prediction_pub_ = get_node()->create_publisher<visualization_msgs::msg::MarkerArray>("/mpc_predictions", 1);
    obstacle_pub_ = get_node()->create_publisher<visualization_msgs::msg::MarkerArray>("/mpc_obstacles", 1);
    monitor_pub_ = get_node()->create_publisher<sensor_msgs::msg::JointState>("/monitor", 10);


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
    phase_start_x0_.assign(22, 0.0);
    latest_x0_.assign(22, 0.0);
    latest_u_sol_.clear(); // filled by the first solve()

    // initialize the observation (state(9) is read in updateStateEstimation) and do a first
    // estimation, so the MPC thread does not start from x0 = 0
    currentObservation_.state.setZero(leggedInterface_->getCentroidalModelInfo().stateDim);
    currentObservation_.input.setZero(leggedInterface_->getCentroidalModelInfo().inputDim);
    currentObservation_.mode = ModeNumber::STANCE;
    updateStateEstimation(node->get_clock()->now(), rclcpp::Duration::from_seconds(0.002));


    // Initialize the step duration
    next_step_duration_ = cfg_.nominalStepDuration();
    walk_requested_ = false;
    walk_step_ = SIZE_MAX;
    liftoff_ = feet_pos_;
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


// HEADING: 0 -> only goal yaw (d <= d_near), 1 -> only velocity alignment (d >= d_far), smoothstep in between
double AcadosLeggedController::headingBlend(double d) const {
    const double x = std::clamp((d - cfg_.heading_d_near) / (cfg_.heading_d_far - cfg_.heading_d_near), 0.0, 1.0);
    return x * x * (3.0 - 2.0 * x);
}


// WARM START: port of the warm start of main.py
// The hyperplane (a, b) must start from a unit vector: with a = 0 the hard constraint 0.5 <= |a|^2 has zero
// gradient and the linearized QP is infeasible (ACADOS_MINSTEP at the first SQP iteration)
void AcadosLeggedController::setWarmStart(const std::vector<double>& x0, const GaitPlanner& gait, const std::vector<ObstacleState>& obstacles){
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
                const double obs[2] = {obstacles[i].x, obstacles[i].y};
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





// Pusblish the predicted CoM of both branches: optimistic in blue and pessimistic orange
// Same namespaces and colors as the OCS2 visualizer (CoM Trajectory, EE Trajectories, Future footholds), the
// pessimistic branch under "Pessimistic ..." and transparent. Node k: the stance pair is in x_k, the other pair
// swings from its last foothold to the one of x_{k+1}, drawn with the swing profile of update()
void AcadosLeggedController::publishPredictions(const GaitPlanner& gait){
    // feet {LF, RF, LH, RH}: blue, orange, yellow, purple
    static constexpr std::array<std::array<float, 3>, 4> kFootColor{{
        {0.0f, 0.447f, 0.741f}, {0.85f, 0.325f, 0.098f}, {0.929f, 0.694f, 0.125f}, {0.494f, 0.184f, 0.556f}}};
    constexpr int kArcPoints = 10;
    using Marker = visualization_msgs::msg::Marker;

    std::array<std::array<double, 2>, 4> swing_start;
    {
        std::lock_guard<std::mutex> lock(mpc_mutex_);
        swing_start = swing_start_xy_;
    }

    auto point = [](double x, double y, double z) {
        geometry_msgs::msg::Point p;
        p.x = x;
        p.y = y;
        p.z = z;
        return p;
    };

    visualization_msgs::msg::MarkerArray msg;
    for (int branch = 0; branch < 2; ++branch){
        const bool optimistic = (branch == 0);
        const size_t offset = optimistic ? 0 : 11; // x = [x_op(11), x_pre(11)]
        const float alpha = optimistic ? 1.0f : 0.35f;

        auto marker = [&](const std::string& ns, int id, int type, double scale) {
            Marker m;
            m.header.frame_id = "odom";
            m.ns = (optimistic ? "" : "Pessimistic ") + ns;
            m.id = id;
            m.type = type;
            m.action = Marker::ADD;
            m.pose.orientation.w = 1.0;
            m.scale.x = m.scale.y = m.scale.z = scale;
            m.color.a = alpha;
            m.lifetime = rclcpp::Duration::from_seconds(0.5); // disappears when MPC stops
            return m;
        };

        Marker com = marker("CoM Trajectory", 0, Marker::LINE_STRIP, 0.01);
        com.color.r = 1.0f;
        for (const auto& x : x_prev_sol_) com.points.push_back(point(x[offset], x[offset + 1], cfg_.com_height));
        msg.markers.push_back(com);

        // Start of each foot: its foothold in x_0, or its lift-off for the pair swinging now
        std::array<std::array<double, 2>, 4> last = swing_start;
        const GaitPlanner::Pair& stance = gait.stancePair();
        for (size_t i = 0; i < 2; ++i) {
            last[stance[i]] = {x_prev_sol_[0][offset + 6 + 2 * i], x_prev_sol_[0][offset + 7 + 2 * i]};
        }
        std::array<Marker, 4> feet;
        for (size_t foot = 0; foot < 4; ++foot) {
            feet[foot] = marker("EE Trajectories", static_cast<int>(foot), Marker::LINE_STRIP, 0.01);
            feet[foot].color.r = kFootColor[foot][0];
            feet[foot].color.g = kFootColor[foot][1];
            feet[foot].color.b = kFootColor[foot][2];
            feet[foot].points.push_back(point(last[foot][0], last[foot][1], cfg_.foot_radius));
        }
        Marker footholds = marker("Future footholds", 0, Marker::SPHERE_LIST, 0.04);

        for (int k = 1; k <= n_horizon_; ++k) {
            const GaitPlanner::Pair& pair = gait.paramPair(k - 1); // lands at the start of node k
            for (size_t i = 0; i < 2; ++i) {
                const size_t foot = pair[i];
                const double fx = x_prev_sol_[k][offset + 6 + 2 * i];
                const double fy = x_prev_sol_[k][offset + 7 + 2 * i];
                const vector3_t liftoff(last[foot][0], last[foot][1], cfg_.foot_radius);
                const vector3_t touchdown(fx, fy, cfg_.foot_radius);
                for (int j = 1; j <= kArcPoints; ++j) {
                    vector3_t p, v;
                    evaluateSwing(liftoff, touchdown, 1.0, cfg_.swing_height, static_cast<double>(j) / kArcPoints, p, v);
                    feet[foot].points.push_back(point(p.x(), p.y(), p.z()));
                }
                last[foot] = {fx, fy};
                footholds.points.push_back(point(fx, fy, cfg_.foot_radius));
                std_msgs::msg::ColorRGBA color;
                color.r = kFootColor[foot][0];
                color.g = kFootColor[foot][1];
                color.b = kFootColor[foot][2];
                color.a = alpha;
                footholds.colors.push_back(color);
            }
        }
        for (auto& foot : feet) msg.markers.push_back(foot);
        msg.markers.push_back(footholds);
    }

    prediction_pub_->publish(msg);
}




// OBSTACLE TOPIC: obstacles the OCP sees now. From the topic: the class radius grows with y_dot_max times the age of
// the message (the obstacle may have moved that much since it was seen, as the pessimistic branch assumes); with more
// obstacles than N_OBS_MAX the closest to the robot (distance minus radius) are kept
std::vector<ObstacleState> AcadosLeggedController::activeObstacles(double robot_x, double robot_y){
    if (!obstacles_from_topic_) return obstacle_manager_->active();

    std::vector<ObstacleState> obstacles;
    double stamp = 0.0;
    bool received = false;
    {
        std::lock_guard<std::mutex> lock(obstacles_mutex_);
        obstacles = topic_obstacles_;
        stamp = topic_obstacles_stamp_;
        received = topic_obstacles_received_;
    }
    if (!received) {
        RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 5000,
                             "[Acados MPC]: no message on %s yet: no obstacles", obstacles_topic_.c_str());
        return obstacles;
    }

    // stamp 0 (ros2 topic pub) = now
    double age = (stamp > 0.0) ? sim_time_ - stamp : 0.0;
    if (age > obstacles_timeout_) {
        RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
                             "[Acados MPC]: obstacles %.2f s old, radius inflated", age);
    }
    age = std::clamp(age, 0.0, obstacles_max_age_);
    for (auto& obs : obstacles) obs.radius += obs.y_dot_max * age;

    if (static_cast<int>(obstacles.size()) > n_obs_max_) {
        auto clearance = [robot_x, robot_y](const ObstacleState& o) {
            return std::hypot(o.x - robot_x, o.y - robot_y) - o.radius;
        };
        std::partial_sort(obstacles.begin(), obstacles.begin() + n_obs_max_, obstacles.end(),
                          [&](const ObstacleState& a, const ObstacleState& b) { return clearance(a) < clearance(b); });
        obstacles.resize(n_obs_max_);
    }
    return obstacles;
}


std::vector<ObstacleState> AcadosLeggedController::toSlots(std::vector<ObstacleState> obstacles) const {
    constexpr double kGhostObsPos = 15.0;   // same ghosts as ObstacleManager::slots
    for (int i = static_cast<int>(obstacles.size()); i < n_obs_max_; ++i) {
        obstacles.push_back({kGhostObsPos + 2.0 * i, kGhostObsPos, 0.0, 0.0});
    }
    return obstacles;
}







// Publish each obstacle in red and its keep-out zone as a transparent disc
void AcadosLeggedController::publishObstacles(const std::vector<ObstacleState>& obstacles){
    visualization_msgs::msg::MarkerArray msg;


    for (size_t i = 0; i < obstacles.size(); ++i){
        for (const bool keep_out : {false, true}){
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "odom";
            marker.ns = keep_out ? "obstacle_keep_out" : "obstacle";
            marker.id = static_cast<int>(i);
            marker.type = visualization_msgs::msg::Marker::CYLINDER;
            marker.action = visualization_msgs::msg::Marker::ADD;
            // 1.8 = factor of the obstacle constraint in lipm_model.py
            const double radius = keep_out ? 1.8 * obstacles[i].radius : obstacles[i].radius;
            marker.scale.x = 2.0 * radius;
            marker.scale.y = 2.0 * radius;
            marker.scale.z = keep_out ? 0.01 : 0.5;
            marker.pose.position.x = obstacles[i].x;
            marker.pose.position.y = obstacles[i].y;
            marker.pose.position.z = marker.scale.z / 2.0;
            marker.pose.orientation.w = 1.0;
            marker.color.r = 1.0f;
            marker.color.a = keep_out ? 0.25f : 1.0f;
            marker.lifetime = rclcpp::Duration::from_seconds(0.5); // disappears if removed from the yaml
            msg.markers.push_back(marker);
        }
    }

    obstacle_pub_->publish(msg);
}





// Send command and store the total motor torque: same PD + feedforward law of Hardware
void AcadosLeggedController::sendJointCommand(size_t j, double q, double dq, double kp, double kd, double tau){
    auto& joint = hybridJointHandles_[j];
    joint.setCommand(q, dq, kp, kd, tau);
    tau_cmd_[j] = kp * (q - joint.getPosition()) + kd * (dq - joint.getVelocity()) + tau;
}



// One message with all the values: JointState gives a name to each of them in PlotJuggler
void AcadosLeggedController::publishMonitor(const std::vector<double>& x0){
    static const std::array<std::string, 12> joint_names{
        "LF_HAA", "LF_HFE", "LF_KFE", "LH_HAA", "LH_HFE", "LH_KFE",
        "RF_HAA", "RF_HFE", "RF_KFE", "RH_HAA", "RH_HFE", "RH_KFE"
    };

    sensor_msgs::msg::JointState msg;
    msg.header.stamp = get_node()->now();
    auto add = [&msg](const std::string& name, double value) {
        msg.name.push_back(name);
        msg.position.push_back(value);
    };

    for (size_t j = 0; j < joint_names.size(); ++j) add("tau_" + joint_names[j], tau_cmd_[j]);
    add("tau_hip_max", cfg_.tau_max_hip);
    add("tau_hip_min", -cfg_.tau_max_hip);
    add("tau_knee_max", cfg_.tau_max_knee);
    add("tau_knee_min", -cfg_.tau_max_knee);

    // Same local velocities of constraints of lipm_model.py
    const double ct = std::cos(x0[2]);
    const double st = std::sin(x0[2]);
    add("v_x", x0[3] * ct + x0[4] * st);
    add("v_y", -x0[3] * st + x0[4] * ct);
    add("yaw_rate", x0[5]);
    add("v_x_max", cfg_.v_max_x);
    add("v_x_min", -cfg_.v_max_x);
    add("v_y_max", cfg_.v_max_y);
    add("v_y_min", -cfg_.v_max_y);
    add("yaw_rate_max", cfg_.theta_dot_max);
    add("yaw_rate_min", -cfg_.theta_dot_max);

    // WBC tracking: references against the measured robot (yaw, yaw_rate above), achieved - requested CoM acceleration
    add("yaw", x0[2]);
    add("yaw_ref", wbc_monitor_[YAW_REF]);
    add("yaw_rate_ref", wbc_monitor_[YAW_RATE_REF]);
    add("yaw_acc_ff", wbc_monitor_[YAW_ACC_FF]);
    add("yaw_acc_wbc", wbc_monitor_[YAW_ACC_WBC]);
    add("com_err_x", wbc_monitor_[COM_ERR_X]);
    add("com_err_y", wbc_monitor_[COM_ERR_Y]);
    add("wbc_residual_x", wbc_monitor_[RESIDUAL_X]);
    add("wbc_residual_y", wbc_monitor_[RESIDUAL_Y]);
    add("wbc_residual_z", wbc_monitor_[RESIDUAL_Z]);

    monitor_pub_->publish(msg);
}






// Thread of the MPC: async mpc
void AcadosLeggedController::mpcThreadLoop(){
    bool mpc_active = false; // warm start is rebuilt each time MPC starts
    auto last_obs_pub = std::chrono::steady_clock::now();
    auto last_pred_pub = last_obs_pub;
    double last_obs_t = sim_time_;

    // For logging average computation time
    double solve_time_sum = 0.0;
    int solve_count = 0;
    double last_log_t = sim_time_;

    // For monitor in PlotJuggler
    auto last_monitor_pub = std::chrono::steady_clock::now();

    while(controller_running_){

        // Move the obstacles
        const double t = sim_time_;
        double robot_x, robot_y;
        {
            std::lock_guard<std::mutex> lock(mpc_mutex_);
            robot_x = current_state_x0_[0];
            robot_y = current_state_x0_[1];
        }
        // dt = 0 outside walk, MPC is off and adversarial must not reach robot standing still
        if (!obstacles_from_topic_) {
            const double obs_dt = (phase_ == Phase::WALK) ? t - last_obs_t : 0.0;
            obstacle_manager_->updateAll(obs_dt, t, robot_x, robot_y);
        }
        last_obs_t = t;
        const std::vector<ObstacleState> active_obstacles = activeObstacles(robot_x, robot_y);

        // Obstacles in RViz at 30Hz, even if robot stands still
        const auto now = std::chrono::steady_clock::now();
        if (now - last_obs_pub > std::chrono::milliseconds(33)){
            publishObstacles(active_obstacles);
            last_obs_pub = now;
        }


        // Values for PlotJuggler at 50Hz
        if(now - last_monitor_pub > std::chrono::milliseconds(20)) {
            std::vector<double> x0_monitor;
            {
                std::lock_guard<std::mutex> lock(mpc_mutex_);
                x0_monitor = current_state_x0_;
            }
            publishMonitor(x0_monitor);
            last_monitor_pub = now;
        }

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
        const std::vector<ObstacleState> obstacles = toSlots(active_obstacles);
        // HEADING: blend of node k on the distance from the goal of the CoM predicted by the previous solution,
        // the last nodes start turning to the goal yaw before the robot gets there
        const double goal_x_now = goal_x_, goal_y_now = goal_y_;
        const bool has_yaw = goal_has_yaw_;
        const double theta_goal = goal_theta_;
        for(int k = 0; k <= n_horizon_; ++k){
            std::vector<double> p = gait_local.hipOffsetParams(k);
            for(const auto& obs : obstacles){
                p.insert(p.end(), {obs.x, obs.y, obs.radius, obs.y_dot_max});
            }
            const std::vector<double>& x_pred = (k == 0) ? x0_local : x_prev_sol_[k];
            const double s_align = has_yaw ? headingBlend(std::hypot(goal_x_now - x_pred[0], goal_y_now - x_pred[1])) : 1.0;
            p.insert(p.end(), {s_align, theta_goal});
            mpc_solver_->setParameters(k, p);
        }
        mpc_solver_->setFirstStepDuration(gait_local.timeLeft());


        // LOCAL TARGET: at most max_lookahead from robot
        const double goal_x = goal_x_;
        const double goal_y = goal_y_;
        const double dir_x = goal_x - x0_local[0];
        const double dir_y = goal_y - x0_local[1];
        const double dist = std::hypot(dir_x, dir_y);
        double local_x = goal_x;
        double local_y = goal_y;
        if (dist > cfg_.max_lookahead) {
            local_x = x0_local[0] + dir_x / dist * cfg_.max_lookahead;
            local_y = x0_local[1] + dir_y / dist * cfg_.max_lookahead;
        }

        // yref: index 0-1 optimistic CoM, 17-18 pessimistic CoM, yref_e: 0-1 and 6-7
        // heading residuals (34-37, 12-15) have reference 0: theta_goal is a parameter
        std::vector<double> yref(OCP_LIPM_OPTI_PESSI_NY, 0.0);
        yref[0] = local_x; yref[1] = local_y;
        yref[17] = local_x; yref[18] = local_y;
        std::vector<double> yref_e(OCP_LIPM_OPTI_PESSI_NYN, 0.0);
        yref_e[0] = local_x; yref_e[1] = local_y;
        yref_e[6] = local_x; yref_e[7] = local_y;
        for (int k = 0; k < n_horizon_; ++k) mpc_solver_->setReference(k, yref);
        mpc_solver_->setReference(n_horizon_, yref_e);



        // WARM START: initial guess for x and u
        setWarmStart(x0_local, gait_local, obstacles);

        // Solve the ocp
        int status = mpc_solver_->solve();
        solve_time_sum += mpc_solver_->getComputationTime();
        solve_count++;

        // publish the solution only if the solver succeeded (0) or reached max iter (2)
        if(status != 0 && status != 2){
            // WARM START: a failed solution is not a good guess, restart from x0
            resetWarmStart(x0_local);
            RCLCPP_INFO(get_node()->get_logger(), "[Acados MPC]: solver failed with status %d", status);
            continue;
        }

        // WARM START: store the whole solution for the next solve
        for(int k = 0; k <= n_horizon_; ++k){
            x_prev_sol_[k] = mpc_solver_->getStatePrediction(k);
            if(k < n_horizon_) u_prev_sol_[k] = mpc_solver_->getControl(k);
        }

        
        // publish predictions at 30 Hz, like the obstacles
        if (std::chrono::steady_clock::now() - last_pred_pub > std::chrono::milliseconds(33)){
            publishPredictions(gait_local);
            last_pred_pub = std::chrono::steady_clock::now();
        }

        {
            std::lock_guard<std::mutex> lock(mpc_mutex_);
            latest_u_sol_ = u_prev_sol_;
            latest_x0_ = x0_local;
            latest_step_index_ = gait_local.stepIndex();
            // Save the duration of the next step
            next_step_duration_ = std::clamp(u_prev_sol_[1][7], cfg_.dt_min, cfg_.dt_max);
        }

        if (walk_requested_) mpc_ready_ = true;   // not after the goal, a solve can end after the stop
        if (t - last_log_t >= 1.0) {
            RCLCPP_INFO(get_node()->get_logger(),
                "[Acados MPC]: status=%d, solve_avg=%.1f ms (%d solves), T_next=%.3f, cop=%.2f",
                status, solve_time_sum / solve_count * 1000.0, solve_count, next_step_duration_, u_prev_sol_[0][4]);
            solve_time_sum = 0.0;
            solve_count = 0;
            last_log_t = t;
        }

        mpc_step_counter_++;
    }
}





// update():
/*
    1) read the real state from sensors
    2) build the task-space references of OptiPessiWbc from the MPC plan (as OptiPessiController)
    3) send commands to the motors
*/
controller_interface::return_type AcadosLeggedController::update(const rclcpp::Time& time, const rclcpp::Duration& period){ // FIX: preiod -> period
    if (period.seconds() < 1e-6) { // repeated clock tick: would divide by zero below
        RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000, "Zero period, cycle skipped");
        return controller_interface::return_type::OK;
    }
    sim_time_ = time.seconds();


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
            // Feet held where they are, CoM xy and yaw measured, CoM height ramps to com_height
            liftoff_ = feet_pos_;
            com_target_ = com_pos_;
            z_start_ = com_pos_.z();
            yaw_target_ = currentObservation_.state(9);
            RCLCPP_INFO_STREAM(get_node()->get_logger(), "[Acados MPC]: STANCE, CoM target: " << com_target_.transpose() << ", yaw " << yaw_target_);
        }
        for (size_t j = 0; j < hybridJointHandles_.size(); ++j) {
            const double q_des = (1.0 - s) * q_start_(j) + s * q_stand_(j);
            sendJointCommand(j, q_des, 0.0, cfg_.kp_stand, cfg_.kd_stand, 0.0);
        }
        return controller_interface::return_type::OK;
    }
    if (phase_ == Phase::STANCE && walk_requested_ && mpc_ready_) phase_ = Phase::WALK;
    if (phase_ == Phase::STANCE){
        // Joint PD of STAND_UP faded out over stance_blend, then WBC torques and joint damping kd_wbc only (OptiPessiController)
        const double r = std::min(phase_time_ / cfg_.stance_blend, 1.0);
        const double kp = (1.0 - r) * cfg_.kp_stand;
        const double kd = (1.0 - r) * cfg_.kd_stand + r * cfg_.kd_wbc;
        if (!updateWholeBodyControl(stanceReference(r), period.seconds(), q_stand_, kp, kd)) {
            return controller_interface::return_type::ERROR;
        }
        return controller_interface::return_type::OK;
    }


    // From here on: WALK
    const vector_t& s = currentObservation_.state;

    // Last MPC Solution and gait state
    std::vector<std::vector<double>> u_sol;
    std::vector<double> x0_solve;
    size_t solve_step = 0, step_index = 0;
    double time_left = 0.0, step_duration = 1.0;
    GaitPlanner::Pair stance, swing;

    {
        std::lock_guard<std::mutex> lock(mpc_mutex_);
        u_sol = latest_u_sol_;
        x0_solve = latest_x0_;
        solve_step = latest_step_index_;
        stance = gait_planner_->stancePair();
        swing = gait_planner_->swingPair();
        step_index = gait_planner_->stepIndex();
        time_left = gait_planner_->timeLeft();
        step_duration = gait_planner_->stepDuration();
    }
    if (u_sol.empty()) return controller_interface::return_type::OK;

    // Goal reached: stop at a foot switch, when the swing feet have just landed
    // HEADING: with a goal yaw also the yaw error must be below yaw_tolerance
    // POSTURE: the MPC keeps trotting at the goal until every foot is under its hip (posture cost of the OCP), at most
    // max_settle_steps steps, so the robot stops in the stand posture and not with the feet of the last stride
    const bool yaw_reached = !goal_has_yaw_ ||
        std::abs(angles::shortest_angular_distance(s(9), goal_theta_)) < cfg_.yaw_tolerance;
    const bool at_goal = yaw_reached &&
        std::hypot(goal_x_ - com_pos_.x(), goal_y_ - com_pos_.y()) < cfg_.goal_tolerance;
    if (step_index != walk_step_ && walk_step_ != SIZE_MAX) {
        if (!at_goal) {
            steps_at_goal_ = 0;
        } else {
            // Largest distance of a foot from its nominal foothold c + R(yaw) hip_offset (hips of the MPC)
            const double cy = std::cos(s(9)), sy = std::sin(s(9));
            const std::array<std::array<double, 2>, 4> off{{{cfg_.off_x, cfg_.off_y}, {cfg_.off_x, -cfg_.off_y},
                                                            {-cfg_.off_x, cfg_.off_y}, {-cfg_.off_x, -cfg_.off_y}}};
            double feet_err = 0.0;
            for (size_t foot = 0; foot < 4; ++foot) {
                const double hx = com_pos_.x() + cy * off[foot][0] - sy * off[foot][1];
                const double hy = com_pos_.y() + sy * off[foot][0] + cy * off[foot][1];
                feet_err = std::max(feet_err, std::hypot(feet_pos_[foot].x() - hx, feet_pos_[foot].y() - hy));
            }
            ++steps_at_goal_;
            if (feet_err < cfg_.feet_tolerance || steps_at_goal_ > cfg_.max_settle_steps) {
                walk_requested_ = false;
                mpc_ready_ = false;
                walk_step_ = SIZE_MAX;
                // Four feet down where they landed, CoM held where it is at com_height (OptiPessiController::standAtGoal)
                liftoff_ = feet_pos_;
                com_target_ = com_pos_;
                z_start_ = cfg_.com_height;
                yaw_target_ = s(9);

                phase_time_ = cfg_.stance_blend;   // already at com_height: no height and gain ramp
                phase_ = Phase::STANCE;
                RCLCPP_INFO_STREAM(get_node()->get_logger(), "[Acados MPC]: Goal reached after " << steps_at_goal_
                                   << " step(s), feet error " << feet_err << " m, STANCE, CoM target: " << com_target_.transpose());
                steps_at_goal_ = 0;
                return controller_interface::return_type::OK;
            }
        }
    }


    // New step: lift-off of the swing pair, the stance pair stays where it is. The CoM and yaw reference restart from
    // the robot measured now and run for the whole step (OptiPessiController closes the LIP loop once per step), so
    // the WBC PD sees and corrects what the robot does not follow
    const bool new_step = (step_index != walk_step_);
    if (new_step) {
        walk_step_ = step_index;
        liftoff_ = feet_pos_;
        phase_start_x0_ = x0_measured_;
        ref_c_ << x0_measured_[0], x0_measured_[1];
        ref_dc_ << x0_measured_[3], x0_measured_[4];
        ref_yaw_ = x0_measured_[2];
        ref_yaw_rate_ = x0_measured_[5];
        std::lock_guard<std::mutex> lock(mpc_mutex_);
        for (size_t foot : swing) swing_start_xy_[foot] = {feet_pos_[foot].x(), feet_pos_[foot].y()};
    }

    // Knot of this step (OptiPessiController::advanceOptiPessiPhase): the solve of this step, or, until it arrives,
    // the latest solve shifted by the steps completed since, from the x0 measured at the start of this step
    const size_t offset = step_index > solve_step ? step_index - solve_step : 0;
    const std::vector<double>& u = u_sol[std::min(offset, u_sol.size() - 1)];
    const std::vector<double>& x_start = (offset == 0) ? x0_solve : phase_start_x0_;
    if (!new_step) advanceWalkReference(x_start, u, period.seconds());

    // Swing time on the gait clock, which decides the foot switches
    const WbcReference reference = walkReference(x_start, u, step_duration - time_left, step_duration, stance, swing);
    if (!updateWholeBodyControl(reference, period.seconds(), s.segment<12>(12), 0.0, cfg_.kd_wbc)) {
        return controller_interface::return_type::ERROR;
    }

    return controller_interface::return_type::OK;
}




// STANCE references (OptiPessiController::holdStance + the height ramp of standUp)
WbcReference AcadosLeggedController::stanceReference(double r) const {
    WbcReference reference;
    const double m = cfg_.m;
    for (size_t foot = 0; foot < 4; ++foot) {
        reference.contact[foot] = true;
        reference.footPosition[foot] = liftoff_[foot];
        reference.footVelocity[foot].setZero();
        reference.footForce[foot] << 0.0, 0.0, 0.25 * m * cfg_.g;
    }

    // CoM height: cubic from z_start_ to com_height over stance_blend, zero velocity at both ends
    const double T = cfg_.stance_blend;
    const double rise = cfg_.com_height - z_start_;
    reference.comPosition << com_target_.x(), com_target_.y(), z_start_ + (3.0 - 2.0 * r) * r * r * rise;
    reference.comVelocity << 0.0, 0.0, r < 1.0 ? 6.0 * r * (1.0 - r) * rise / T : 0.0;
    reference.comAcceleration << 0.0, 0.0, r < 1.0 ? (6.0 - 12.0 * r) * rise / (T * T) : 0.0;
    reference.yaw = yaw_target_;
    reference.yawRate = 0.0;
    reference.yawAcceleration = 0.0;
    return reference;
}


// WALK references (OptiPessiController::updateFootReferences + updateComReference)
WbcReference AcadosLeggedController::walkReference(const std::vector<double>& x_start, const std::vector<double>& u, double swing_time,
                                                   double step_duration, const GaitPlanner::Pair& stance,
                                                   const GaitPlanner::Pair& swing) const {
    WbcReference reference;
    const double w = std::sqrt(cfg_.g / cfg_.h_com);
    swing_time = std::clamp(swing_time, 0.0, step_duration);

    // Stance pair (p0, p1 of x): stays where it was at lift-off and carries the forces of the knot
    const auto forces = computeContactForces(x_start, u, cfg_.m, cfg_.g, w);
    for (size_t k = 0; k < 2; ++k) {
        const size_t foot = stance[k];
        reference.contact[foot] = true;
        reference.footPosition[foot] = liftoff_[foot];
        reference.footVelocity[foot].setZero();
        reference.footForce[foot] = forces[k];
    }

    // Swing pair: from lift-off to the footholds of the knot (u[0..1] first foot, u[2..3] second), flat ground
    for (size_t k = 0; k < 2; ++k) {
        const size_t foot = swing[k];
        const vector3_t touchdown(u[2 * k], u[2 * k + 1], liftoff_[foot].z());
        reference.contact[foot] = false;
        evaluateSwing(liftoff_[foot], touchdown, step_duration, cfg_.swing_height, swing_time,
                      reference.footPosition[foot], reference.footVelocity[foot]);
        reference.footForce[foot].setZero();
    }

    // CoM: reference of the step, LIP acceleration ddc = w^2 (c - cop) with the CoP held at alpha
    const Eigen::Vector2d p0(x_start[6], x_start[7]), p1(x_start[8], x_start[9]);
    const Eigen::Vector2d cop = p0 + u[4] * (p1 - p0);
    reference.comPosition << ref_c_, cfg_.com_height;
    reference.comVelocity << ref_dc_, 0.0;
    reference.comAcceleration << (w * w) * (ref_c_ - cop), 0.0;

    // Yaw: reference of the step, torque of the tangential forces of the knot (lipm_model.py: dth_next = dth + dt tau_z / Iz)
    reference.yaw = ref_yaw_;
    reference.yawRate = ref_yaw_rate_;
    reference.yawAcceleration = yawTorque(x_start, forces) / cfg_.Iz;
    return reference;
}


// CoM and yaw reference of the step one tick ahead (OptiPessiController::updateComReference, integrated tick by tick
// because the MPC changes alpha and the forces within the step): exact LIP flow over dt with the CoP held at alpha,
// yaw with the torque of the knot
void AcadosLeggedController::advanceWalkReference(const std::vector<double>& x_start, const std::vector<double>& u, double dt){
    const double w = std::sqrt(cfg_.g / cfg_.h_com);
    const Eigen::Vector2d p0(x_start[6], x_start[7]), p1(x_start[8], x_start[9]);
    const Eigen::Vector2d cop = p0 + u[4] * (p1 - p0);
    const double ch = std::cosh(w * dt);
    const double sh = std::sinh(w * dt);
    const Eigen::Vector2d c = ref_c_;
    ref_c_ = ch * c + (sh / w) * ref_dc_ + (1.0 - ch) * cop;
    ref_dc_ = (w * sh) * (c - cop) + ch * ref_dc_;

    const double yaw_acc = yawTorque(x_start, computeContactForces(x_start, u, cfg_.m, cfg_.g, w)) / cfg_.Iz;
    ref_yaw_ += dt * ref_yaw_rate_ + 0.5 * dt * dt * yaw_acc;
    ref_yaw_rate_ += dt * yaw_acc;
}


// OptiPessiController::updateWholeBodyControl
bool AcadosLeggedController::updateWholeBodyControl(const WbcReference& reference, double period, const vector_t& q_target, double kp, double kd){
    const auto& info = leggedInterface_->getCentroidalModelInfo();
    const size_t nq = info.generalizedCoordinatesNum;
    const vector_t x = wbc_->update(reference, measuredRbdState_);   // [qdd, F, tau]

    // Tracking of the references, for PlotJuggler (publishMonitor)
    const vector3_t residual = wbc_->getLastCentroidalResidual();
    wbc_monitor_[YAW_REF] = reference.yaw;
    wbc_monitor_[YAW_RATE_REF] = reference.yawRate;
    wbc_monitor_[YAW_ACC_FF] = reference.yawAcceleration;
    wbc_monitor_[YAW_ACC_WBC] = x(3);   // qdd of the ZYX yaw
    wbc_monitor_[COM_ERR_X] = reference.comPosition.x() - com_pos_.x();
    wbc_monitor_[COM_ERR_Y] = reference.comPosition.y() - com_pos_.y();
    wbc_monitor_[RESIDUAL_X] = residual.x();
    wbc_monitor_[RESIDUAL_Y] = residual.y();
    wbc_monitor_[RESIDUAL_Z] = residual.z();

    // No joint references: the velocity target is the measured joint velocity advanced by one step of the WBC
    // joint accelerations
    vector_t desiredInput = vector_t::Zero(info.inputDim);
    desiredInput.head(3 * info.numThreeDofContacts) = x.segment(nq, 3 * info.numThreeDofContacts);
    desiredInput.tail(info.actuatedDofNum) =
        measuredRbdState_.segment(nq + 6, info.actuatedDofNum) + period * x.segment(6, info.actuatedDofNum);
    currentObservation_.input = desiredInput;

    if (!safetyChecker_->check(currentObservation_, currentObservation_.state, desiredInput)) {
        RCLCPP_ERROR(get_node()->get_logger(), "[Acados MPC]: Safety check failed!!");
        return false;
    }

    const vector_t torque = x.tail(info.actuatedDofNum);
    for (size_t j = 0; j < info.actuatedDofNum; ++j) {
        sendJointCommand(j, q_target(j), desiredInput(3 * info.numThreeDofContacts + j), kp, kd, torque(j));
    }
    return true;
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

    // YAW ZERO: the yaw of the IMU starts at its power on, the position of the estimator at on_init.
    // Rotate the world by the first yaw so that odom starts with robot at (0, 0, 0)
    if (!yaw_offset_set_ && quat.norm() > 1e-6) {
        yaw_offset_ = quatToZyx(quat.normalized())(0);
        yaw_offset_set_ = true;
        RCLCPP_INFO(get_node()->get_logger(), "[Acados MPC]: initial imu yaw %.3f rad set to zero", yaw_offset_);
    }
    if (yaw_offset_set_) {
        quat = Eigen::Quaternion<scalar_t>(Eigen::AngleAxis<scalar_t>(-yaw_offset_, vector3_t::UnitZ())) * quat;
    }

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



    // MAPPING FROM the measured robot to the LIPM x0 STATE (OptiPessiController::measureLipState): whole-body CoM
    // (the one OptiPessiWbc tracks) and its velocity, yaw and ZYX yaw rate, stance feet from forward kinematics
    const vector_t& state = currentObservation_.state;
    const auto& info = leggedInterface_->getCentroidalModelInfo();
    const size_t nq = info.generalizedCoordinatesNum;
    const vector_t& rbd = measuredRbdState_;  // [zyx, basePos, q, angularVelWorld, baseLinearVelWorld, dq]
    const vector3_t zyx = rbd.head<3>();
    const vector3_t angularVelWorld = rbd.segment<3>(nq);
    vector_t qPino(nq), vPino(nq);
    qPino << rbd.segment<3>(3), zyx, rbd.segment(6, info.actuatedDofNum);
    vPino << rbd.segment<3>(nq + 3), getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<scalar_t>(zyx, angularVelWorld),
             rbd.segment(nq + 6, info.actuatedDofNum);

    // centerOfMass() also runs the forward kinematics that getPosition() reads
    auto& pinocchioInterface = leggedInterface_->getPinocchioInterface();
    com_pos_ = pinocchio::centerOfMass(pinocchioInterface.getModel(), pinocchioInterface.getData(), qPino, vPino);
    const vector3_t com_vel = pinocchioInterface.getData().vcom[0];
    pinocchio::updateFramePlacements(pinocchioInterface.getModel(), pinocchioInterface.getData());
    auto feet_positions = eeKinematicsPtr_->getPosition(state);
    for (size_t i = 0; i < 4; ++i) feet_pos_[i] = feet_positions[i];

    std::vector<double>     x0_lipm(22, 0.0);

    // CoM data (optimistic)
    x0_lipm[0] = com_pos_.x();
    x0_lipm[1] = com_pos_.y();
    x0_lipm[2] = state(9);      // Yaw (unwrapped)
    x0_lipm[3] = com_vel.x();
    x0_lipm[4] = com_vel.y();
    x0_lipm[5] = vPino(3);      // ZYX yaw rate, the one of the yaw task of OptiPessiWbc

    // CoM data (pessimistic) (the same initially)
    x0_lipm[11] = x0_lipm[0];
    x0_lipm[12] = x0_lipm[1];
    x0_lipm[13] = x0_lipm[2];
    x0_lipm[14] = x0_lipm[3];
    x0_lipm[15] = x0_lipm[4];
    x0_lipm[16] = x0_lipm[5];


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

    x0_measured_ = x0_lipm;

    // Upgrading shared variable
    {
        std::lock_guard<std::mutex> lock(mpc_mutex_);
        current_state_x0_ = x0_lipm;
        stance_feet_xy_ = {x0_lipm[6], x0_lipm[7], x0_lipm[8], x0_lipm[9]};
        // Not walking: every foot is on the ground, its lift-off is where it stands
        if (phase_ != Phase::WALK) {
            for (size_t i = 0; i < 4; ++i) swing_start_xy_[i] = {feet_positions[i].x(), feet_positions[i].y()};
        }
    }

}



} // namespace legged




// register the controller as a ros2_control plugin
PLUGINLIB_EXPORT_CLASS(legged::AcadosLeggedController, controller_interface::ControllerInterface)
