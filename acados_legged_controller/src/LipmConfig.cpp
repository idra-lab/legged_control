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


    cfg.dt_min = readKey<double>(root, "limits", "dt_min");
    cfg.dt_max = readKey<double>(root, "limits", "dt_max");
    cfg.v_max_x = readKey<double>(root, "limits", "v_max_x");
    cfg.v_max_y = readKey<double>(root, "limits", "v_max_y");
    cfg.theta_dot_max = readKey<double>(root, "limits", "theta_dot_max");
    cfg.tau_max_hip = readKey<double>(root, "limits", "tau_max_hip");
    cfg.tau_max_knee = readKey<double>(root, "limits", "tau_max_knee");


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
    cfg.feet_tolerance = readKey<double>(root, "simulation", "feet_tolerance");
    cfg.max_settle_steps = readKey<int>(root, "simulation", "max_settle_steps");
    cfg.theta_target = readKey<double>(root, "simulation", "theta_target");

    const auto blend_dist = readKey<std::vector<double>>(root, "heading", "blend_dist");
    if (blend_dist.size() != 2 || blend_dist[0] < 0.0 || blend_dist[1] <= blend_dist[0]){
        throw std::runtime_error("[LipmConfig]: heading.blend_dist must be [d_near, d_far] with 0 <= d_near < d_far!!");
    }
    cfg.heading_d_near = blend_dist[0];
    cfg.heading_d_far = blend_dist[1];
    cfg.yaw_tolerance = readKey<double>(root, "heading", "yaw_tolerance");


    cfg.N_OBS_MAX = readKey<int>(root, "obstacles", "N_OBS_MAX");
    for (const auto& obs : root["obstacles"]["instances"]) {
        const auto pos = obs["pos"].as<std::vector<double>>();
        const auto pos_end = obs["pos_end"].as<std::vector<double>>(pos);
        if (pos.size() != 2 || pos_end.size() != 2){
            throw std::runtime_error("[LipmConfig]: obstacle.instances pos and pos_end must have 2 elements!!");
        }
        ObstacleConfig obstacle;
        obstacle.type = obs["type"].as<std::string>();
        obstacle.x = pos[0];
        obstacle.y = pos[1];
        obstacle.r_obs = obs["r_obs"].as<double>();
        obstacle.y_dot_max = obs["y_dot_max"].as<double>();
        obstacle.end_x = pos_end[0];
        obstacle.end_y = pos_end[1];
        obstacle.speed = obs["speed"].as<double>(0.0);
        obstacle.obstacle_class = obs["class"].as<std::string>("human");
        if (obstacle.obstacle_class != "human" && obstacle.obstacle_class != "car"){
            throw std::runtime_error("[LipmConfig]: obstacle class '" + obstacle.obstacle_class + "' unknown (human, car)!");
        }
        cfg.obstacles.push_back(obstacle);
    }
    const YAML::Node random = root["obstacles"]["random"];
    if (random){
        cfg.random_obstacles.count = random["count"].as<int>();
        cfg.random_obstacles.seed = random["seed"].as<unsigned int>();
        cfg.random_obstacles.bounds = random["bounds"].as<std::array<double, 4>>();
        cfg.random_obstacles.r_range = random["r_range"].as<std::array<double, 2>>();
        cfg.random_obstacles.speed_range = random["speed_range"].as<std::array<double, 2>>();
        cfg.random_obstacles.y_dot_max = random["y_dot_max"].as<double>();
    }


    auto readType = [&root](const std::string& name) {
        const YAML::Node type = root["obstacle_types"][name];
        if (!type || !type["radius"] || !type["y_dot_max"]){
            throw std::runtime_error("[LipmConfig]: missing obstacle_types." + name + ".{radius, y_dot_max} in config.yaml");
        }
        return ObstacleTypeConfig{type["radius"].as<double>(), type["y_dot_max"].as<double>()};
    };
    cfg.human_type = readType("human");
    cfg.car_type = readType("car");


    cfg.standup_duration = readKey<double>(root, "controller", "standup_duration");
    cfg.kp_stand = readKey<double>(root, "controller", "kp_stand");
    cfg.kd_stand = readKey<double>(root, "controller", "kd_stand");
    cfg.kd_wbc = readKey<double>(root, "controller", "kd_wbc");
    cfg.stance_blend = readKey<double>(root, "controller", "stance_blend");
    cfg.swing_height = readKey<double>(root, "controller", "swing_height");
    cfg.com_height = readKey<double>(root, "controller", "com_height");


    return cfg;
}
