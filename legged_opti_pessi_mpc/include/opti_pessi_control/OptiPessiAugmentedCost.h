#pragma once

#include <ocs2_core/cost/StateInputCost.h>
#include <ocs2_core/reference/TargetTrajectories.h>

namespace ocs2 {
namespace quadruped {

class OptiPessiAugmentedCost : public StateInputCost {
public:
    OptiPessiAugmentedCost();
    ~OptiPessiAugmentedCost() override = default;

    OptiPessiAugmentedCost* clone() const override;

    scalar_t getValue(scalar_t t, const vector_t& x, const vector_t& u, 
                      const TargetTrajectories& targetTrajectories, const PreComputation& preComp) const override;

    ScalarFunctionQuadraticApproximation getQuadraticApproximation(
        scalar_t t, const vector_t& x, const vector_t& u, 
        const TargetTrajectories& targetTrajectories, const PreComputation& preComp) const override;

private:
    matrix_t Q_;
    matrix_t R_;
};

} // namespace quadruped
} // namespace ocs2
