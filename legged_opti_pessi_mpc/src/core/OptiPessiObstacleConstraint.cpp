#include "opti_pessi_control/OptiPessiObstacleConstraint.h"
#include <mutex>
#include <algorithm>

namespace ocs2 {
namespace quadruped {

OptiPessiObstacleConstraint::OptiPessiObstacleConstraint(std::shared_ptr<ObstacleState> obs_state)
    : StateInputConstraint(ConstraintOrder::Linear), obs_state_(obs_state) {
    R0_ = 0.8;
}

OptiPessiObstacleConstraint* OptiPessiObstacleConstraint::clone() const {
    return new OptiPessiObstacleConstraint(obs_state_);
}

size_t OptiPessiObstacleConstraint::getNumConstraints(scalar_t t) const {
    return 2;
}

vector_t OptiPessiObstacleConstraint::getValue(scalar_t t, const vector_t& x, const vector_t& u, const PreComputation& preComp) const {
    vector_t g(2);

    double obs_x, obs_y, v_max_obs, init_time;
    {
        std::lock_guard<std::mutex> lock(obs_state_->mutex);
        obs_x = obs_state_->x;
        obs_y = obs_state_->y;
        v_max_obs = obs_state_->max_vel;
        init_time = obs_state_->init_time;
    }

    const scalar_t tau = std::max<scalar_t>(0.0, t - init_time);

    // Scenario Ottimista
    scalar_t co_x = x[0];
    scalar_t co_y = x[1];

    // Scenario Pessimista
    scalar_t cp_x = x[10];
    scalar_t cp_y = x[11];

    scalar_t dist_sq_op = (co_x - obs_x) * (co_x - obs_x) + (co_y - obs_y) * (co_y - obs_y);
    g[0] = dist_sq_op - (R0_ * R0_);

    scalar_t R_pe = R0_ + v_max_obs * tau;
    scalar_t dist_sq_pe = (cp_x - obs_x) * (cp_x - obs_x) + (cp_y - obs_y) * (cp_y - obs_y);
    g[1] = dist_sq_pe - (R_pe * R_pe);

    return g;
}

VectorFunctionLinearApproximation OptiPessiObstacleConstraint::getLinearApproximation(scalar_t t, const vector_t& x, const vector_t& u, const PreComputation& preComp) const {
    VectorFunctionLinearApproximation linearApproximation;

    linearApproximation.f = getValue(t, x, u, preComp);
    linearApproximation.dfdx = matrix_t::Zero(2, 20);
    linearApproximation.dfdu = matrix_t::Zero(2, 16);

    double obs_x, obs_y;
    {
        std::lock_guard<std::mutex> lock(obs_state_->mutex);
        obs_x = obs_state_->x;
        obs_y = obs_state_->y;
    }

    linearApproximation.dfdx(0, 0) = 2.0 * (x[0] - obs_x);
    linearApproximation.dfdx(0, 1) = 2.0 * (x[1] - obs_y);

    linearApproximation.dfdx(1, 10) = 2.0 * (x[10] - obs_x);
    linearApproximation.dfdx(1, 11) = 2.0 * (x[11] - obs_y);

    return linearApproximation;
}

} // namespace quadruped
} // namespace ocs2