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

    cfg.dt_min = readKey<double>(root, "limits", "dt_min");
    cfg.dt_max = readKey<double>(root, "limits", "dt_max");

    cfg.N_horizon = readKey<int>(root, "simulation", "N_horizon");
    cfg.step_per_phase = readKey<int>(root, "simulation", "step_per_phase");

    cfg.N_OBS_MAX = readKey<int>(root, "obstacles", "N_OBS_MAX");

    return cfg;
}
