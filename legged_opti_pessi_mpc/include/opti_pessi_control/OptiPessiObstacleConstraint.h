#pragma once

#include <ocs2_core/constraint/StateInputConstraint.h>
#include <memory>
#include "opti_pessi_control/OptiPessiSharedState.h"

namespace ocs2 {
namespace quadruped {

class OptiPessiObstacleConstraint : public StateInputConstraint {
public:
    explicit OptiPessiObstacleConstraint(std::shared_ptr<ObstacleState> obs_state);
    ~OptiPessiObstacleConstraint() override = default;

    OptiPessiObstacleConstraint* clone() const override;

    size_t getNumConstraints(scalar_t t) const override;

    vector_t getValue(scalar_t t, const vector_t& x, const vector_t& u, const PreComputation& preComp) const override;

    VectorFunctionLinearApproximation getLinearApproximation(scalar_t t, const vector_t& x, const vector_t& u, const PreComputation& preComp) const override;

private:
    scalar_t R0_;
    std::shared_ptr<ObstacleState> obs_state_;
};

} // namespace quadruped
} // namespace ocs2
