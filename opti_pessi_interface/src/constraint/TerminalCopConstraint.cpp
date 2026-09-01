#include "opti_pessi_interface/constraint/TerminalCopConstraint.h"

#include "opti_pessi_interface/LipKinematics.h"

namespace opti_pessi {

TerminalCopConstraint::TerminalCopConstraint(OptiPessiModelParameters params, const std::string& libraryFolder, bool recompile)
    : ocs2::StateInputConstraintCppAd(ocs2::ConstraintOrder::Linear), params_(std::move(params)) {
  numConstraints_ = 4;  // 2 branches x 2 planar components
  initialize(static_cast<size_t>(params_.stateDim()), static_cast<size_t>(params_.inputDim()), 0, "opti_pessi_terminal_cop",
             libraryFolder, recompile, true);
}

ocs2::ad_vector_t TerminalCopConstraint::constraintFunction(ocs2::ad_scalar_t, const ocs2::ad_vector_t& state,
                                                             const ocs2::ad_vector_t& input, const ocs2::ad_vector_t&) const {
  using Scalar = ocs2::ad_scalar_t;
  using Vec = ocs2::ad_vector_t;
  using Vec2 = Eigen::Matrix<Scalar, 2, 1>;

  Vec g(static_cast<int>(numConstraints_));
  const Scalar w = Scalar(params_.omega());
  const Scalar mass = Scalar(params_.mass);
  const Scalar inertia = Scalar(params_.inertia);
  int idx = 0;

  auto appendBranch = [&](const Vec& x, const Vec& u) {
    const Vec xNext = lipMap(x, u, w, mass, inertia);
    const Vec2 p0Next(xNext(RobotX::P0X), xNext(RobotX::P0Y));
    const Vec2 p1Next(xNext(RobotX::P1X), xNext(RobotX::P1Y));
    const auto cop = computeCop(p0Next, p1Next, u(RobotU::ALPHA));
    g(idx++) = xNext(RobotX::CX) - cop(0);
    g(idx++) = xNext(RobotX::CY) - cop(1);
  };

  appendBranch(state.head(RobotX::DIM), input.head(RobotU::DIM));
  appendBranch(state.segment(RobotX::DIM, RobotX::DIM), input.segment(RobotU::DIM, RobotU::DIM));
  return g;
}

}  // namespace opti_pessi
