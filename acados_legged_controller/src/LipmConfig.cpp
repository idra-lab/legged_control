#include "acados_legged_controller/LipmConfig.hpp"

#include <stdexcept>
#include <yaml-cpp/yaml.h>


namespace {

// Read node[section][key] with a clear error message if it is missing
template <typename T>
T readKey(const YAML::Node& root, const std::string& section, const std::string& key){
    const YAML::Node value = root[section][key];
    if(!value){
        throw std::runtime_error("[LipmConfig]: missing key '" + section + "." + key + "' in config.yaml");
    }
    return value.as<T>();
}

} // namespace


LipmConfig LipmConfig::fromYaml(const std::string& path){
    YAML::Node root;
    try{
        root = YAML::LoadFile(path);
    }catch(const YAML::Exception& e){
        throw std::runtime_error("[LipmConfig]: impossible to load '" + path + "': " + e.what());
    }

    LipmConfig cfg;

    cfg.h_com = readKey<double>(root, "robot", "h_com");
    cfg.g = readKey<double>(root, "robot", "g");
    cfg.m = readKey<double>(root, "robot", "m");
    cfg.Iz = readKey<double>(root, "robot", "Iz");
    cfg.off_x = readKey<double>(root, "robot", "off_x");
    cfg.off_y = readKey<double>(root, "robot", "off_y");
    cfg.leg_length = readKey<double>(root, "robot", "leg_length");
    cfg.foot_radius = readKey<double>(root, "robot", "foot_radius");
    cfg.hip_y = readKey<double>(root, "robot", "hip_y");
    cfg.thigh_offset = readKey<double>(root, "robot", "thigh_offset");


    cfg.dt_min = readKey<double>(root, "limits", "dt_min");
    cfg.dt_max = readKey<double>(root, "limits", "dt_max");

    cfg.N_horizon = readKey<int>(root, "simulation", "N_horizon");
    cfg.step_per_phase = readKey<int>(root, "simulation", "step_per_phase");

    const auto c_target = readKey<std::vector<double>>(root, "simulation", "c_target");
    if (c_target.size() != 2){
        throw std::runtime_error("[LipmConfig]: simulation.c_target must have 2 elements!!");
    }
    cfg.c_target_x = c_target[0];
    cfg.c_target_y = c_target[1];
    cfg.max_lookahead = readKey<double>(root, "simulation", "max_lookahead");
    cfg.goal_tolerance = readKey<double>(root, "simulation", "goal_tolerance");


    cfg.N_OBS_MAX = readKey<int>(root, "obstacles", "N_OBS_MAX");

    cfg.standup_duration = readKey<double>(root, "controller", "standup_duration");
    cfg.kp_stand = readKey<double>(root, "controller", "kp_stand");
    cfg.kd_stand = readKey<double>(root, "controller", "kd_stand");
    cfg.kp_stance = readKey<double>(root, "controller", "kp_stance");
    cfg.kd_stance = readKey<double>(root, "controller", "kd_stance");
    cfg.kp_base_xy = readKey<double>(root, "controller", "kp_base_xy");
    cfg.kd_base_xy = readKey<double>(root, "controller", "kd_base_xy");
    cfg.kp_base_z = readKey<double>(root, "controller", "kp_base_z");
    cfg.kd_base_z = readKey<double>(root, "controller", "kd_base_z");
    cfg.stance_blend = readKey<double>(root, "controller", "stance_blend");
    cfg.kp_base_ang = readKey<double>(root, "controller", "kp_base_ang");
    cfg.kd_base_ang = readKey<double>(root, "controller", "kd_base_ang");
    cfg.kp_swing = readKey<double>(root, "controller", "kp_swing");
    cfg.swing_height = readKey<double>(root, "controller", "swing_height");
    


    return cfg;
}
