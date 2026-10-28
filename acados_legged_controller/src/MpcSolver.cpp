#include "acados_legged_controller/MpcSolver.hpp"



MpcSolver::MpcSolver(int n_obs_max, int n_horizon) : n_obs_max_(n_obs_max), n_horizon_(n_horizon)
{
    nx_ = 22;
    nu_ = (8 + 3 * n_obs_max_) * 2;
    np_ = 4 + 4 * n_obs_max_;

    // Create the acados solver
    capsule_ = ocp_lipm_opti_pessi_0e58481e_acados_create_capsule();
    int status = ocp_lipm_opti_pessi_0e58481e_acados_create(capsule_);

    if(status != 0 || status != 2){
        std::cerr << "[MpcSolver] Sovler error: impossible to create the solver. Status: " << status << std::endl;
    }

    // Extraction of the pointers
    nlp_config_ = ocp_lipm_opti_pessi_0e58481e_acados_get_nlp_config(capsule_);
    nlp_dims_ = ocp_lipm_opti_pessi_0e58481e_acados_get_nlp_dims(capsule_);
    nlp_in_ = ocp_lipm_opti_pessi_0e58481e_acados_get_nlp_in(capsule_);
    nlp_out_ = ocp_lipm_opti_pessi_0e58481e_acados_get_nlp_out(capsule_);
    nlp_solver_ = ocp_lipm_opti_pessi_0e58481e_acados_get_nlp_solver(capsule_);
    nlp_opts_ = ocp_lipm_opti_pessi_0e58481e_acados_get_nlp_opts(capsule_);

}


MpcSolver::~MpcSolver(){
    ocp_lipm_opti_pessi_0e58481e_acados_free(capsule_);
    ocp_lipm_opti_pessi_0e58481e_acados_free_capsule(capsule_);
}


bool MpcSolver::setInitialState(const std::vector<double>& x0){
    if(static_cast<int>(x0.size()) != nx_) return false;
    // In acados to set the initial state we have to set lower and upper bound equal to the node 0
    ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "lbx", const_cast<double*>(x0.data()));
    ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "ubx", const_cast<double*>(x0.data()));

    return true;
}

bool MpcSolver::setReference(int stage, const std::vector<double>& yref){
    if(stage < 0 || stage > n_horizon_) return false;
    // Intermediate stage has 34 variables the final 12
    if (stage == n_horizon_ && yref.size() != 12) return false;
    if (stage < n_horizon_ && yref.size() != 34) return false;

    ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, stage, "yref", const_cast<double*>(yref.data()));
    return true;
}


bool MpcSolver::setParameters(int stage, const std::vector<double>& p){
    if(stage < 0 || stage > n_horizon_ || static_cast<int>(p.size()) != np_) return false;
    ocp_lipm_opti_pessi_0e58481e_acados_update_params(capsule_, stage, const_cast<double*>(p.data()), p.size());

    return true;
}


bool MpcSolver::setGuess(int stage, const std::vector<double>& x_guess, const std::vector<double>& u_guess){
    if(stage < 0 || stage > n_horizon_) return false;

    if (!x_guess.empty() && static_cast<int>(x_guess.size()) == nx_){
        ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, nlp_in_, stage, "x", const_cast<double*>(x_guess.data()));
    }
    if (!u_guess.empty() && static_cast<int>(u_guess.size()) == nu_ && stage < n_horizon_) {
        ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, nlp_in_, stage, "u", const_cast<double*>(u_guess.data()));
    }
    return true;
}


int MpcSolver::solve(){
    int status = ocp_lipm_opti_pessi_0e58481e_acados_solve(capsule_);
    return status;
}


std::vector<double> MpcSolver::getControlCommand(){
    std::vector<double> u_opt(nu_);
    ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, 0, "u", u_opt.data());
    return u_opt;
}

std::vector<double> MpcSolver::getStatePrediction(int stage){
    std::vector<double> x_opt(nx_);
    if(stage >= 0 && stage <= n_horizon_){
        ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, stage, "x", x_opt.data());
    }
    return x_opt;
}


double MpcSolver::getComputationTime() const {
    double time = 0.0;
    ocp_nlp_get(nlp_solver_, "time_tot", &time);
    return time;
}