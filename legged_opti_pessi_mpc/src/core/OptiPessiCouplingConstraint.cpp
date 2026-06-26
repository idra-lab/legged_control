#include "opti_pessi_control/OptiPessiCouplingConstraint.h"
#include <mutex>
#include <cmath>

namespace ocs2 {
namespace quadruped {

OptiPessiCouplingConstraint::OptiPessiCouplingConstraint(std::shared_ptr<ObstacleState> horizon_state)
    : StateInputConstraint(ConstraintOrder::Linear), horizon_state_(horizon_state) {}

OptiPessiCouplingConstraint* OptiPessiCouplingConstraint::clone() const {
    return new OptiPessiCouplingConstraint(horizon_state_);
}

bool OptiPessiCouplingConstraint::isActive(scalar_t time) const {
    double init_time = 0.0;
    if (horizon_state_) {
        std::lock_guard<std::mutex> lock(horizon_state_->mutex);
        init_time = horizon_state_->init_time;
    }
    return std::abs(time - init_time) < 1e-6;
}

size_t OptiPessiCouplingConstraint::getNumConstraints(scalar_t t) const {
    return 8;
}

vector_t OptiPessiCouplingConstraint::getValue(scalar_t t, const vector_t& x, const vector_t& u, const PreComputation& preComp) const {
    vector_t u_op = u.head(8);
    vector_t u_pe = u.tail(8);
    return u_op - u_pe;
}

VectorFunctionLinearApproximation OptiPessiCouplingConstraint::getLinearApproximation(scalar_t t, const vector_t& x, const vector_t& u, const PreComputation& preComp) const {
    VectorFunctionLinearApproximation linearApproximation;

    linearApproximation.f = getValue(t, x, u, preComp);
    linearApproximation.dfdx = matrix_t::Zero(8, 20);

    linearApproximation.dfdu = matrix_t::Zero(8, 16);
    linearApproximation.dfdu.leftCols(8) = matrix_t::Identity(8, 8);
    linearApproximation.dfdu.rightCols(8) = -matrix_t::Identity(8, 8);

    return linearApproximation;
}

} // namespace quadruped
} // namespace ocs2