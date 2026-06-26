#pragma once

#include <ocs2_core/constraint/StateInputConstraint.h>
#include <memory>
#include "opti_pessi_control/OptiPessiSharedState.h"

namespace ocs2 {
namespace quadruped {

class OptiPessiCouplingConstraint : public StateInputConstraint {
public:
    explicit OptiPessiCouplingConstraint(std::shared_ptr<ObstacleState> horizon_state);
    ~OptiPessiCouplingConstraint() override = default;

    OptiPessiCouplingConstraint* clone() const override;

    bool isActive(scalar_t time) const override;

    size_t getNumConstraints(scalar_t t) const override;

    vector_t getValue(scalar_t t, const vector_t& x, const vector_t& u, const PreComputation& preComp) const override;

    VectorFunctionLinearApproximation getLinearApproximation(scalar_t t, const vector_t& x, const vector_t& u, const PreComputation& preComp) const override;

private:
    std::shared_ptr<ObstacleState> horizon_state_;
};

} // namespace quadruped
} // namespace ocs2
