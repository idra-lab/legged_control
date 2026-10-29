#pragma once

#include <string>


// Parameters shared with the python code generator (scripts/model_gen.py), read from config/config.yaml
struct LipmConfig {
    // robot
    double h_com;   // m
    double g;       // m/s^2
    double m;       // kg
    double Iz;      // kg/m^2
    double off_x;   // m
    double off_y;   // m

    // limits
    double dt_min;  // sec
    double dt_max;  // sec

    // simulation
    int N_horizon;
    int step_per_phase;

    // obstacles
    int N_OBS_MAX;

    // Same nominal dt used in lipm_model.py
    double nominalStepDuration() const { return (dt_min + dt_max) / (2.0 * step_per_phase); }

    // Load the values from the yaml file, throws std::runtime_error if the file or a key is missing
    static LipmConfig fromYaml(const std::string& path);
};
