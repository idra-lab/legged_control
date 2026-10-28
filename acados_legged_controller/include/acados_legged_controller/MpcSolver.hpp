#pragma once

#include <vector>
#include <iostream>

#include "acados_solver_ocp_lipm_opti_pessi_0e58481e.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"



class MpcSolver {
public:
    // Pass N_OBS_MAX to the constructor to compute the correct dimension of the ocp
    MpcSolver(int n_obs_max, int n_horizon);
    ~MpcSolver();
    
    
    // Set initial state
    bool setInitialState(const std::vector<double>& x0);

    // Set the reference (yref) at node k - dim 34 (12 if k == N)
    bool setReference(int stage, const std::vector<double>& yref);

    // Set params at node k - sim 4 + 4*N_obs_max
    bool setParameters(int stage, const std::vector<double>& p);

    // Set the initial guess for the state and control (warm start)
    bool setGuess(int stage, const std::vector<double>& x_guess, const std::vector<double>& u_guess);

    // Solve the OCP
    int solve();

    // Retrieve the optimal control at node 0
    std::vector<double> getControlCommand();

    // Get the prediction at node k (for debugging or logging)
    std::vector<double> getStatePrediction(int stage);

    // Get the computation time
    double getComputationTime() const;



private:
    int n_obs_max_;
    int n_horizon_;
    int nx_;
    int nu_;
    int np_;

    ocp_lipm_opti_pessi_0e58481e_solver_capsule* capsule_;
    ocp_nlp_config* nlp_config_;
    ocp_nlp_dims* nlp_dims_;
    ocp_nlp_in* nlp_in_;
    ocp_nlp_out* nlp_out_;
    ocp_nlp_solver* nlp_solver_;
    void* nlp_opts_;

};