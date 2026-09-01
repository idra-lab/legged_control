#include "opti_pessi_interface/cost/OptiPessiCost.h"

#include "opti_pessi_interface/LipKinematics.h"

namespace opti_pessi {

namespace {

/** Parameter layout of the stage cost: [goal(2), hip of next foot 0(2), hip of next foot 1(2)]. */
constexpr int kStageCostParamDim = 6;

vector_t goalAndNextHips(const OptiPessiModelParameters& params, const OptiPessiReferenceManager& referenceManager, int nextKnot) {
  const auto pair = gaitPair(referenceManager.getGaitOffset() + nextKnot);
  vector_t p(kStageCostParamDim);
  p.segment(0, 2) = referenceManager.getGoal();
  p.segment(2, 2) = hipOf(params, pair[0]);
  p.segment(4, 2) = hipOf(params, pair[1]);
  return p;
}

/** Running cost of a single branch, given that branch's (x, u) slice. */
template <typename Scalar>
Scalar oneBranchCost(const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& x, const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& u,
                     const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& parameters, const OptiPessiModelParameters& params) {
  using Vec2 = Eigen::Matrix<Scalar, 2, 1>;

  const Vec2 c(x(RobotX::CX), x(RobotX::CY));
  const Scalar theta = x(RobotX::TH);
  const Vec2 dc(x(RobotX::DCX), x(RobotX::DCY));
  const Scalar dtheta = x(RobotX::DTH);
  const Vec2 cGoal(parameters(0), parameters(1));

  Scalar cost = runningStateCost(c, theta, dc, dtheta, cGoal, params);

  const Scalar alpha = u(RobotU::ALPHA);
  const Scalar dt = u(RobotU::DT);
  cost += Scalar(params.wa) * (alpha - Scalar(0.5)) * (alpha - Scalar(0.5));
  cost += Scalar(params.wdt) * (dt - Scalar(params.dtCost0)) * (dt - Scalar(params.dtCost0));

  // Foothold-to-hip term is evaluated at knot i+1, so the next state has to be formed here.
  const auto xNext = lipMap(x, u, Scalar(params.omega()), Scalar(params.mass), Scalar(params.inertia));
  const Vec2 cN(xNext(RobotX::CX), xNext(RobotX::CY));
  const Scalar thN = xNext(RobotX::TH);
  const Vec2 hip0(parameters(2), parameters(3));
  const Vec2 hip1(parameters(4), parameters(5));
  const Vec2 hip0w = cN + applyR(thN, hip0);
  const Vec2 hip1w = cN + applyR(thN, hip1);
  const Vec2 p0n(xNext(RobotX::P0X), xNext(RobotX::P0Y));
  const Vec2 p1n(xNext(RobotX::P1X), xNext(RobotX::P1Y));
  const Vec2 e0 = p0n - hip0w;
  const Vec2 e1 = p1n - hip1w;
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
  return goalAndNextHips(params_, *referenceManagerPtr_, i + 1);
}

ocs2::ad_scalar_t OptiPessiStageCost::costFunction(ocs2::ad_scalar_t, const ocs2::ad_vector_t& state, const ocs2::ad_vector_t& input,
                                                   const ocs2::ad_vector_t& parameters) const {
  // Optimistic branch only.
  const ocs2::ad_vector_t xOpti = state.head(RobotX::DIM);
  const ocs2::ad_vector_t uOpti = input.head(RobotU::DIM);
  return oneBranchCost(xOpti, uOpti, parameters, params_);
}

OptiPessiFinalCost::OptiPessiFinalCost(OptiPessiModelParameters params, const OptiPessiReferenceManager& referenceManager,
                                       const std::string& libraryFolder, bool recompile)
    : params_(std::move(params)), referenceManagerPtr_(&referenceManager) {
  initialize(static_cast<size_t>(params_.stateDim()), 2, "opti_pessi_final_cost", libraryFolder, recompile, true);
}

vector_t OptiPessiFinalCost::getParameters(scalar_t, const ocs2::TargetTrajectories&, const ocs2::PreComputation&) const {
  return referenceManagerPtr_->getGoal();
}

ocs2::ad_scalar_t OptiPessiFinalCost::costFunction(ocs2::ad_scalar_t, const ocs2::ad_vector_t& state,
                                                   const ocs2::ad_vector_t& parameters) const {
  using Scalar = ocs2::ad_scalar_t;
  using Vec2 = Eigen::Matrix<Scalar, 2, 1>;

  const Vec2 cGoal(parameters(0), parameters(1));
  const Vec2 c(state(RobotX::CX), state(RobotX::CY));
  const Scalar theta = state(RobotX::TH);
  const Vec2 dc(state(RobotX::DCX), state(RobotX::DCY));
  const Scalar dtheta = state(RobotX::DTH);
  return runningStateCost(c, theta, dc, dtheta, cGoal, params_);
}

}  // namespace opti_pessi
