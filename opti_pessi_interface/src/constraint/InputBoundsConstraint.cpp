#include "opti_pessi_interface/constraint/InputBoundsConstraint.h"

#include "opti_pessi_interface/LipKinematics.h"

namespace opti_pessi {

namespace {

/** 8 box bounds on (alpha, beta, gamma, dt) + 2 friction cones. */
constexpr int kRowsPerBranch = 10;

}  // namespace

InputBoundsConstraint::InputBoundsConstraint(OptiPessiModelParameters params, const std::string& libraryFolder, bool recompile)
    : ocs2::StateInputConstraintCppAd(ocs2::ConstraintOrder::Linear), params_(std::move(params)) {
  // optimistic + pessimistic, plus one non-negativity row per pessimistic slack variable
  numConstraints_ = static_cast<size_t>(2 * kRowsPerBranch + params_.numObstacles());
  initialize(static_cast<size_t>(params_.stateDim()), static_cast<size_t>(params_.inputDim()), 0, "opti_pessi_input_bounds",
             libraryFolder, recompile, true);
}

ocs2::ad_vector_t InputBoundsConstraint::constraintFunction(ocs2::ad_scalar_t, const ocs2::ad_vector_t& state,
                                                             const ocs2::ad_vector_t& input, const ocs2::ad_vector_t&) const {
  using Scalar = ocs2::ad_scalar_t;
  using Vec = ocs2::ad_vector_t;
  using Vec2 = Eigen::Matrix<Scalar, 2, 1>;

  Vec g(static_cast<int>(numConstraints_));
  int idx = 0;

  const Scalar epsAlpha = Scalar(params_.alphaReduction);

  auto appendBranch = [&](const Vec& x, const Vec& u) {
    const Scalar alpha = u(RobotU::ALPHA);
    const Scalar dt = u(RobotU::DT);
    const Scalar beta = u(RobotU::BETA);
    const Scalar gamma = u(RobotU::GAMMA);

    // Box bounds. The alpha margin keeps the CoP strictly inside the support segment so both feet
    // keep a positive normal load.
    g(idx++) = alpha - epsAlpha;
    g(idx++) = Scalar(1) - epsAlpha - alpha;
    g(idx++) = beta;
    g(idx++) = Scalar(1) - beta;
    g(idx++) = gamma;
    g(idx++) = Scalar(1) - gamma;
    g(idx++) = dt - Scalar(params_.dtMin);
    g(idx++) = Scalar(params_.dtMax) - dt;

    // Circular friction cones, normal loads split by alpha. Normalized by the squared cone limit so
    // these rows are the same order of magnitude as the box bounds.
    const Vec2 c(x(RobotX::CX), x(RobotX::CY));
    const Vec2 p0(x(RobotX::P0X), x(RobotX::P0Y));
    const Vec2 p1(x(RobotX::P1X), x(RobotX::P1Y));
    Vec2 f0, f1;
    computeTangentialForces(c, p0, p1, alpha, beta, gamma, Scalar(params_.omega()), Scalar(params_.mass), f0, f1);
    const Scalar fn0 = alpha * Scalar(params_.frictionCoefficient) * Scalar(params_.mass) * Scalar(params_.gravity);
    const Scalar fn1 = (Scalar(1) - alpha) * Scalar(params_.frictionCoefficient) * Scalar(params_.mass) * Scalar(params_.gravity);
    g(idx++) = Scalar(1) - (f0(0) * f0(0) + f0(1) * f0(1)) / (fn0 * fn0 + Scalar(1e-3));
    g(idx++) = Scalar(1) - (f1(0) * f1(0) + f1(1) * f1(1)) / (fn1 * fn1 + Scalar(1e-3));
  };

  appendBranch(state.head(RobotX::DIM), input.head(RobotU::DIM));
  appendBranch(state.segment(RobotX::DIM, RobotX::DIM), input.segment(RobotU::DIM, RobotU::DIM));

  // Slack variables live once in the augmented input (not per branch): s_j >= 0.
  const int numObstacles = params_.numObstacles();
  const int slackBase = pessiSlackOffset(numObstacles);
  for (int j = 0; j < numObstacles; ++j) {
    g(idx++) = input(slackBase + j);
  }

  return g;
}

}  // namespace opti_pessi
