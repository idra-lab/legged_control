#include "opti_pessi_interface/constraint/FirstInputConsensusConstraint.h"

namespace opti_pessi {

FirstInputConsensusConstraint::FirstInputConsensusConstraint(OptiPessiModelParameters params, const std::string& libraryFolder,
                                                             bool recompile)
    : ocs2::StateInputConstraintCppAd(ocs2::ConstraintOrder::Linear), params_(std::move(params)) {
  numConstraints_ = static_cast<size_t>(RobotU::DIM);
  initialize(static_cast<size_t>(params_.stateDim()), static_cast<size_t>(params_.inputDim()), 0, "opti_pessi_first_input_consensus",
             libraryFolder, recompile, true);
}

ocs2::ad_vector_t FirstInputConsensusConstraint::constraintFunction(ocs2::ad_scalar_t, const ocs2::ad_vector_t&,
                                                                    const ocs2::ad_vector_t& input, const ocs2::ad_vector_t&) const {
  ocs2::ad_vector_t g(static_cast<int>(numConstraints_));
  for (int i = 0; i < RobotU::DIM; ++i) {
    g(i) = input(i) - input(RobotU::DIM + i);
  }
  return g;
}

}  // namespace opti_pessi
