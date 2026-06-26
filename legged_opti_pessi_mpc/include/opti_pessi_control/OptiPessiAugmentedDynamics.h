#pragma once

#include <ocs2_core/dynamics/SystemDynamicsBase.h>

namespace ocs2 {
namespace quadruped {

class OptiPessiAugmentedDynamics : public SystemDynamicsBase {
public:
    OptiPessiAugmentedDynamics() = default;
    ~OptiPessiAugmentedDynamics() override = default;

    OptiPessiAugmentedDynamics* clone() const override;

    size_t getStateDim() const { return 20; }
    size_t getInputDim() const { return 16; }

    vector_t computeFlowMap(scalar_t t, const vector_t& x, const vector_t& u, const PreComputation& preComp) override;

    VectorFunctionLinearApproximation linearApproximation(scalar_t t, const vector_t& x, const vector_t& u, const PreComputation& preComp) override;

private:
    void computeSingleLipmJacobians(const vector_t& x, const vector_t& u, matrix_t& A, matrix_t& B);
    vector_t computeSingleLipmDynamics(const vector_t& x, const vector_t& u);
};

} // namespace quadruped
} // namespace ocs2
