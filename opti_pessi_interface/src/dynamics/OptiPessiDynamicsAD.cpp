#include "opti_pessi_interface/dynamics/OptiPessiDynamicsAD.h"

#include "opti_pessi_interface/LipKinematics.h"

namespace opti_pessi {

OptiPessiDynamicsAD::OptiPessiDynamicsAD(OptiPessiModelParameters params, const std::string& libraryFolder, bool recompile)
    : params_(std::move(params)) {
  initialize(static_cast<size_t>(params_.stateDim()), static_cast<size_t>(params_.inputDim()), "opti_pessi_dynamics", libraryFolder,
             recompile, true);
}

ocs2::ad_vector_t OptiPessiDynamicsAD::systemFlowMap(ocs2::ad_scalar_t /*time*/, const ocs2::ad_vector_t& state,
                                                     const ocs2::ad_vector_t& input, const ocs2::ad_vector_t& /*parameters*/) const {
  using ocs2::ad_scalar_t;
  using ocs2::ad_vector_t;

  const ad_scalar_t w = ad_scalar_t(params_.omega());
  const ad_scalar_t mass = ad_scalar_t(params_.mass);
  const ad_scalar_t inertia = ad_scalar_t(params_.inertia);

  ad_vector_t xNext = state;

  auto advanceBranch = [&](int xOffset, int uOffset) {
    const ad_vector_t x = state.segment(xOffset, RobotX::DIM);
    const ad_vector_t u = input.segment(uOffset, RobotU::DIM);
    xNext.segment(xOffset, RobotX::DIM) = lipMap(x, u, w, mass, inertia);
  };

  advanceBranch(0, 0);                          // optimistic
  advanceBranch(RobotX::DIM, RobotU::DIM);      // pessimistic

  // Clock accumulates the pessimistic phase durations: T_{i+1} = T_i + dt_i^pessi.
  xNext(CLOCK_INDEX) = state(CLOCK_INDEX) + input(RobotU::DIM + RobotU::DT);

  // dt = 1.0 with EULER makes  x_{i+1} = x_i + flowMap  reproduce the exact discrete map.
  return xNext - state;
}

}  // namespace opti_pessi
