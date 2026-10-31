#pragma once

#include <string>
#include <vector>


// Parameters shared with the python code generator (scripts/model_gen.py), read from config/config.yaml
struct LipmConfig {
    // robot
    double h_com;           // m
    double g;               // m/s^2
    double m;               // kg
    double Iz;              // kg/m^2
    double off_x;           // m
    double off_y;           // m
    double hip_y;           // m
    double thigh_offset;    // m

    // limits
    double dt_min;  // sec
    double dt_max;  // sec

    // simulation
    int N_horizon;
    int step_per_phase;
    double c_target_x;
    double c_target_y;
    double max_lookahead;

    // obstacles
    int N_OBS_MAX;


    
    double leg_length;  // m
    double foot_radius; // m

    // controller
    double standup_duration, kp_stand, kd_stand;
    double kp_stance, kd_stance;
    double kp_base_xy, kd_base_xy, kp_base_z, kd_base_z;
    double stance_blend;
    double kp_base_ang;
    double kd_base_ang;
    double kp_swing;
    double swing_height;

    // Same nominal dt used in lipm_model.py
    double nominalStepDuration() const { return (dt_min + dt_max) / (2.0 * step_per_phase); }

    // Load the values from the yaml file, throws std::runtime_error if the file or a key is missing
    static LipmConfig fromYaml(const std::string& path);
};
