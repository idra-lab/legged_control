#include "acados_legged_controller/MpcSolver.hpp"
#include "acados_legged_controller/LipmConfig.hpp"
#include <iostream>
#include <vector>


int main(int argc, char** argv){
    // CONFIG: dimensions from config.yaml (path as first argument, default: config.yaml of the source package)
    const std::string config_path = (argc > 1) ? argv[1] : LIPM_CONFIG_FILE;
    LipmConfig cfg = LipmConfig::fromYaml(config_path);
    int n_obs_max = cfg.N_OBS_MAX;
    int n_horizon = cfg.N_horizon;


    std::cout << "---Starting MpcSolver ---" << std::endl;
    std::cout << "Initializaion of the solver..." << std::endl;
    MpcSolver solver(n_obs_max, n_horizon);


    // Initial state (zeros)
    std::vector<double> x0(22, 0.0);
    solver.setInitialState(x0);
    
    // Parameter and reference
    int np = 4 + 4 * n_obs_max;
    std::vector<double> p(np, 0.0);

    std::vector<double> yref(34, 0.0);
    std::vector<double> yref_e(12, 0.0);

    for (int k = 0; k <= n_horizon; ++k){
        solver.setParameters(k, p);
        if(k < n_horizon){
            solver.setReference(k, yref);
        }else{
            solver.setReference(k, yref_e);
        }
    }


    // Solve the OCP
    std::cout << "Solving the OCP..." << std::endl;
    int status = solver.solve();

    std::cout << "\n --- Results ---\n";
    std::cout << "Status of the solver: " << status << "\n";
    std::cout << "Computation time: " << solver.getComputationTime() * 1000 << " ms\n";
    

    auto u_opt = solver.getControlCommand();
    if(!u_opt.empty()){
        std::cout << "Control action computed [p0_next_x, ...]: \n";
        for(int i = 0; i < 4; ++i){
            std::cout << "u[" << i << "] = " << u_opt[i] << std::endl;
        }
    }


    return 0;
}