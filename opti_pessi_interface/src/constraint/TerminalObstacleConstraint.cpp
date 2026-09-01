#include "opti_pessi_interface/constraint/TerminalObstacleConstraint.h"

#include <algorithm>

#include "opti_pessi_interface/LipKinematics.h"

namespace opti_pessi {

TerminalObstacleConstraint::TerminalObstacleConstraint(OptiPessiModelParameters params,
                                                       const OptiPessiReferenceManager& referenceManager,
                                                       const std::string& libraryFolder, bool recompile)
    : ocs2::StateConstraintCppAd(ocs2::ConstraintOrder::Linear),
      params_(std::move(params)),
      referenceManagerPtr_(&referenceManager) {
  numConstraints_ = static_cast<size_t>(2 * params_.numObstacles());  // optimistic + pessimistic

  // Radius of the disk about the CoM that contains every hip.
  for (const auto& hip : params_.hipOffsets) {
    hullRadius_ = std::max(hullRadius_, hip.norm());
  }

  initialize(static_cast<size_t>(params_.stateDim()), static_cast<size_t>(2 * params_.numObstacles() + 1),
             "opti_pessi_terminal_obstacle", libraryFolder, recompile, true);
}

vector_t TerminalObstacleConstraint::getParameters(scalar_t, const ocs2::PreComputation&) const {
  const int numObstacles = params_.numObstacles();
  vector_t p = vector_t::Zero(2 * numObstacles + 1);
  const matrix_t& obstacles = referenceManagerPtr_->getObstacles();
  for (int j = 0; j < numObstacles; ++j) {
    p(2 * j) = obstacles(j, 0);
    p(2 * j + 1) = obstacles(j, 1);
  }
  p(2 * numObstacles) = referenceManagerPtr_->getPessiScale();
  return p;
}

ocs2::ad_vector_t TerminalObstacleConstraint::constraintFunction(ocs2::ad_scalar_t, const ocs2::ad_vector_t& state,
                                                                 const ocs2::ad_vector_t& parameters) const {
  using Scalar = ocs2::ad_scalar_t;
  using Vec2 = Eigen::Matrix<Scalar, 2, 1>;

  ocs2::ad_vector_t g(static_cast<int>(numConstraints_));
  int idx = 0;
  const int numObstacles = params_.numObstacles();
  const Scalar pessiScale = parameters(2 * numObstacles);
  const Scalar elapsed = state(CLOCK_INDEX);

  auto appendBranch = [&](const ocs2::ad_vector_t& x, const Scalar& keepOutGrowth) {
    const Vec2 c(x(RobotX::CX), x(RobotX::CY));
    for (int j = 0; j < numObstacles; ++j) {
      const Vec2 o(parameters(2 * j), parameters(2 * j + 1));
      const Scalar required = Scalar(params_.obstacleRadius) + keepOutGrowth + Scalar(hullRadius_);
      // Squared form keeps this smooth at c == o.
      g(idx++) = (c - o).squaredNorm() - required * required;
    }
  };

  appendBranch(state.head(RobotX::DIM), Scalar(0));
  appendBranch(state.segment(RobotX::DIM, RobotX::DIM), pessiScale * Scalar(params_.obstacleMaxSpeed) * elapsed);
  return g;
}

}  // namespace opti_pessi
