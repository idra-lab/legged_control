#include "opti_pessi_interface/cost/OptiPessiCost.h"

#include "opti_pessi_interface/LipKinematics.h"

namespace opti_pessi {

namespace {

/** Parameter layout of the stage cost: [goal(2), hip of next foot 0(2), hip of next foot 1(2)]. */
constexpr int kStageCostParamDim = 6;

vector_t goalAndStanceHips(const OptiPessiModelParameters& params, const OptiPessiReferenceManager& referenceManager, int knot) {
  const auto pair = gaitPair(referenceManager.getGaitOffset() + knot);
  vector_t p(kStageCostParamDim);
  p.segment(0, 2) = referenceManager.getGoal();
  p.segment(2, 2) = hipOf(params, pair[0]);
  p.segment(4, 2) = hipOf(params, pair[1]);
  return p;
}

/**
 * Running cost of a single branch, evaluated entirely at this knot -- no composition with the
 * dynamics. See the note at the top of StageInequalityConstraint.cpp for why that matters.
 *
 * The reference writes its foothold term on knot i+1; written on knot i and summed over knots
 * 0..N it is the same family of terms.
 */
template <typename Scalar>
Scalar oneBranchStateCost(const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& x,
                          const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& parameters,
                          const OptiPessiModelParameters& params) {
  using Vec2 = Eigen::Matrix<Scalar, 2, 1>;

  const Vec2 c(x(RobotX::CX), x(RobotX::CY));
  const Scalar theta = x(RobotX::TH);
  const Vec2 dc(x(RobotX::DCX), x(RobotX::DCY));
  const Scalar dtheta = x(RobotX::DTH);
  const Vec2 cGoal(parameters(0), parameters(1));

  Scalar cost = runningStateCost(c, theta, dc, dtheta, cGoal, params);

  // Keep the stance feet under their nominal hips.
  const Vec2 hip0(parameters(2), parameters(3));
  const Vec2 hip1(parameters(4), parameters(5));
  const Vec2 e0 = Vec2(x(RobotX::P0X), x(RobotX::P0Y)) - (c + applyR(theta, hip0));
  const Vec2 e1 = Vec2(x(RobotX::P1X), x(RobotX::P1Y)) - (c + applyR(theta, hip1));
  cost += Scalar(params.wp) * (e0.dot(e0) + e1.dot(e1));

  return cost;
}

}  // namespace

OptiPessiStageCost::OptiPessiStageCost(OptiPessiModelParameters params, const OptiPessiReferenceManager& referenceManager,
                                       const std::string& libraryFolder, bool recompile)
    : params_(std::move(params)), referenceManagerPtr_(&referenceManager) {
  initialize(static_cast<size_t>(params_.stateDim()), static_cast<size_t>(params_.inputDim()), kStageCostParamDim,
             "opti_pessi_stage_cost", libraryFolder, recompile, true);
}

vector_t OptiPessiStageCost::getParameters(scalar_t time, const ocs2::TargetTrajectories&, const ocs2::PreComputation&) const {
  const int i = intervalIndex(time, params_.N);
  return goalAndStanceHips(params_, *referenceManagerPtr_, i);
}

ocs2::ad_scalar_t OptiPessiStageCost::costFunction(ocs2::ad_scalar_t, const ocs2::ad_vector_t& state, const ocs2::ad_vector_t& input,
                                                   const ocs2::ad_vector_t& parameters) const {
  using Scalar = ocs2::ad_scalar_t;
  const ocs2::ad_vector_t xOpti = state.head(RobotX::DIM);
  const Scalar alpha = input(RobotU::ALPHA);
  const Scalar dt = input(RobotU::DT);
  Scalar cost = oneBranchStateCost(xOpti, parameters, params_);
  cost += Scalar(params_.wa) * (alpha - Scalar(0.5)) * (alpha - Scalar(0.5));
  cost += Scalar(params_.wdt) * (dt - Scalar(params_.dtCost0)) * (dt - Scalar(params_.dtCost0));

  // No keep-out slack penalty here -- see the slack note in definitions.h. Every term above is
  // O(1)-weighted (wc = 1, wdc = 2, wp = 0.5, wa = 0.5, wdt = 1e-2); a 1e5 penalty alongside them
  // does not soften a constraint, it buys a different optimum.

  return cost;
}

OptiPessiFinalCost::OptiPessiFinalCost(OptiPessiModelParameters params, const OptiPessiReferenceManager& referenceManager,
                                       const std::string& libraryFolder, bool recompile)
    : params_(std::move(params)), referenceManagerPtr_(&referenceManager) {
  initialize(static_cast<size_t>(params_.stateDim()), kStageCostParamDim, "opti_pessi_final_cost", libraryFolder, recompile, true);
}

vector_t OptiPessiFinalCost::getParameters(scalar_t, const ocs2::TargetTrajectories&, const ocs2::PreComputation&) const {
  return goalAndStanceHips(params_, *referenceManagerPtr_, params_.N);
}

ocs2::ad_scalar_t OptiPessiFinalCost::costFunction(ocs2::ad_scalar_t, const ocs2::ad_vector_t& state,
                                                   const ocs2::ad_vector_t& parameters) const {
  return oneBranchStateCost(ocs2::ad_vector_t(state.head(RobotX::DIM)), parameters, params_);
}

}  // namespace opti_pessi
