#include "opti_pessi_interface/cost/OptiPessiCost.h"

#include "opti_pessi_interface/LipKinematics.h"

namespace opti_pessi {

namespace {

/** Parameter layout of the stage cost: [goal(2), hip of NEXT foot 0(2), hip of NEXT foot 1(2)]. */
constexpr int kStageCostParamDim = 6;

/** Parameter layout of the final cost: [goal(2)]. The foothold term does not reach knot N. */
constexpr int kFinalCostParamDim = 2;

vector_t goalAndNextStanceHips(const OptiPessiModelParameters& params, const OptiPessiReferenceManager& referenceManager,
                               int interval) {
  const auto pair = gaitPair(referenceManager.getGaitOffset() + interval + 1);
  vector_t p(kStageCostParamDim);
  p.segment(0, 2) = referenceManager.getGoal();
  p.segment(2, 2) = hipOf(params, pair[0]);
  p.segment(4, 2) = hipOf(params, pair[1]);
  return p;
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
  return goalAndNextStanceHips(params_, *referenceManagerPtr_, i);
}

ocs2::ad_scalar_t OptiPessiStageCost::costFunction(ocs2::ad_scalar_t, const ocs2::ad_vector_t& state, const ocs2::ad_vector_t& input,
                                                   const ocs2::ad_vector_t& parameters) const {
  using Scalar = ocs2::ad_scalar_t;
  using Vec2 = Eigen::Matrix<Scalar, 2, 1>;

  const ocs2::ad_vector_t x = state.head(RobotX::DIM);
  const ocs2::ad_vector_t u = input.head(RobotU::DIM);

  const Vec2 c(x(RobotX::CX), x(RobotX::CY));
  const Vec2 dc(x(RobotX::DCX), x(RobotX::DCY));
  const Vec2 cGoal(parameters(0), parameters(1));
  Scalar cost = runningStateCost(c, Scalar(x(RobotX::TH)), dc, Scalar(x(RobotX::DTH)), cGoal, params_);

  const Scalar alpha = u(RobotU::ALPHA);
  const Scalar dt = u(RobotU::DT);
  cost += Scalar(params_.wa) * (alpha - Scalar(0.5)) * (alpha - Scalar(0.5));
  cost += Scalar(params_.wdt) * (dt - Scalar(params_.dtCost0)) * (dt - Scalar(params_.dtCost0));

  // Foothold term, written on the SUCCESSOR knot exactly as the reference writes it
  // (ocp_quadruped.py:64-66: hips taken at x[:, i+1] with the gait pair of knot i+1, feet taken at
  // x[-4:, i+1]). The successor is recomputed here by stepping the dynamics; the landing feet of
  // lipMap are just u's foothold entries, but the hips they are measured against are not.
  const ocs2::ad_vector_t xNext = lipMap(x, u, Scalar(params_.omega()), Scalar(params_.mass), Scalar(params_.inertia));
  const Vec2 cNext(xNext(RobotX::CX), xNext(RobotX::CY));
  const Scalar thetaNext = xNext(RobotX::TH);
  const Vec2 hip0(parameters(2), parameters(3));
  const Vec2 hip1(parameters(4), parameters(5));
  const Vec2 e0 = Vec2(xNext(RobotX::P0X), xNext(RobotX::P0Y)) - (cNext + applyR(thetaNext, hip0));
  const Vec2 e1 = Vec2(xNext(RobotX::P1X), xNext(RobotX::P1Y)) - (cNext + applyR(thetaNext, hip1));
  cost += Scalar(params_.wp) * (e0.dot(e0) + e1.dot(e1));

  // No keep-out slack penalty here -- see the slack note in definitions.h. Every term above is
  // O(1)-weighted (wc = 1, wdc = 2, wp = 0.5, wa = 0.5, wdt = 1e-2); a 1e5 penalty alongside them
  // does not soften a constraint, it buys a different optimum.

  return cost;
}

OptiPessiFinalCost::OptiPessiFinalCost(OptiPessiModelParameters params, const OptiPessiReferenceManager& referenceManager,
                                       const std::string& libraryFolder, bool recompile)
    : params_(std::move(params)), referenceManagerPtr_(&referenceManager) {
  initialize(static_cast<size_t>(params_.stateDim()), kFinalCostParamDim, "opti_pessi_final_cost", libraryFolder, recompile, true);
}

vector_t OptiPessiFinalCost::getParameters(scalar_t, const ocs2::TargetTrajectories&, const ocs2::PreComputation&) const {
  return referenceManagerPtr_->getGoal();
}

ocs2::ad_scalar_t OptiPessiFinalCost::costFunction(ocs2::ad_scalar_t, const ocs2::ad_vector_t& state,
                                                   const ocs2::ad_vector_t& parameters) const {
  using Scalar = ocs2::ad_scalar_t;
  using Vec2 = Eigen::Matrix<Scalar, 2, 1>;

  // Knot N carries no input, so it carries no alpha/dt/foothold term -- the reference's running_cost
  // adds those only for i < N (ocp_quadruped.py:60-67).
  const ocs2::ad_vector_t x = state.head(RobotX::DIM);
  const Vec2 c(x(RobotX::CX), x(RobotX::CY));
  const Vec2 dc(x(RobotX::DCX), x(RobotX::DCY));
  const Vec2 cGoal(parameters(0), parameters(1));
  return runningStateCost(c, Scalar(x(RobotX::TH)), dc, Scalar(x(RobotX::DTH)), cGoal, params_);
}

}  // namespace opti_pessi
