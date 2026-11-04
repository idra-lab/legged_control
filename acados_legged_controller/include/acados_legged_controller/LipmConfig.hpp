#pragma once

#include <string>
#include <vector>
#include <array>

// Entry of obstacles.instances in config.yaml
struct ObstacleConfig {
    std::string type;
    double x, y;            // m
    double r_obs;           // m    
    double y_dot_max;       // m/s
    double end_x = 0.0;     // m
    double end_y = 0.0;     // m
    double speed = 0.0;     // m/s
};


// obstacle.random in congig.yaml
struct RandomObstacleConfig {
    int count = 0;
    unsigned int seed = 0;
    std::array<double, 4> bounds{};         // m: x_min, x_max, y_min, y_max
    std::array<double, 2> r_range{};        // m
    std::array<double, 2> speed_range{};    // m/s
    double y_dot_max = 0.0;                 // m/s
};



// Parameters shared with the python code generator (scripts/model_gen.py), read from config/config.yaml
struct LipmConfig {
    // robot
    double h_com;           // m
    double g;               // m/s^2
    double m;               // kg
    double Iz;              // kg/m^2
    double off_x;           // m
    double off_y;           // m

    // limits
    double dt_min;                      // sec
    double dt_max;                      // sec
    double v_max_x, v_max_y;            // m/s
    double theta_dot_max;               // rad/s
    double tau_max_hip, tau_max_knee;   // Nm 

    // simulation
    int N_horizon;
    int step_per_phase;
    double c_target_x;
    double c_target_y;
    double max_lookahead;
    double goal_tolerance;  // m

    // obstacles
    int N_OBS_MAX;
    std::vector<ObstacleConfig> obstacles;
    RandomObstacleConfig random_obstacles;


    
    double leg_length;  // m
    double foot_radius; // m

    // controller (CoM, orientation and swing gains are those of OptiPessiWbc in task.info)
    double standup_duration, kp_stand, kd_stand;
    double kd_wbc;          // joint damping with the WBC torques, all joints (OptiPessiController: 3)
    double stance_blend;
    double swing_height;
    double com_height;      // m, CoM height tracked by the WBC (h_com is the one of the acados model)

    // Same nominal dt used in lipm_model.py
    double nominalStepDuration() const { return (dt_min + dt_max) / (2.0 * step_per_phase); }

    // Load the values from the yaml file, throws std::runtime_error if the file or a key is missing
    static LipmConfig fromYaml(const std::string& path);
};
