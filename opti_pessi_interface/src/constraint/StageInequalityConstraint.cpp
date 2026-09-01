#include "opti_pessi_interface/constraint/StageInequalityConstraint.h"

#include "opti_pessi_interface/LipKinematics.h"

namespace opti_pessi {

namespace {

/** 8 input bounds + 2 friction cones + 3 velocity bounds + 8 reachability/separation. */
constexpr int kRobotInequalities = 21;

/** Per obstacle: (4 hips + 1 obstacle) mid-step + (4 hips + 2 feet + 1 obstacle) landing. */
constexpr int kCollisionInequalitiesPerObstacle = 12;

/** Parameters: 12 hips-and-signs + 2 per obstacle centre + 1 pessimistic scale. */
constexpr int kHipsAndSignsDim = 12;

int stageInequalityCount(const OptiPessiModelParameters& params) {
  const int perBranch = kRobotInequalities + kCollisionInequalitiesPerObstacle * params.numObstacles();
  return 2 * perBranch;  // optimistic + pessimistic
}

int parameterDim(const OptiPessiModelParameters& params) {
  return kHipsAndSignsDim + 2 * params.numObstacles() + 1;
}

/**
 * Hip offsets of the current and next stance pairs plus their left/right signs, so the AD library
 * stays gait-agnostic and only the parameter vector changes as the trot alternates.
 */
vector_t hipsAndSignsParameters(const OptiPessiModelParameters& params, const OptiPessiReferenceManager& referenceManager, int i) {
  const auto current = gaitPair(referenceManager.getGaitOffset() + i);
  const auto next = gaitPair(referenceManager.getGaitOffset() + i + 1);
  vector_t p(kHipsAndSignsDim);
  p.segment(0, 2) = hipOf(params, current[0]);
  p.segment(2, 2) = hipOf(params, current[1]);
  p.segment(4, 2) = hipOf(params, next[0]);
  p.segment(6, 2) = hipOf(params, next[1]);
  p(8) = isLeft(current[0]) ? 1.0 : -1.0;
  p(9) = isLeft(current[1]) ? 1.0 : -1.0;
  p(10) = isLeft(next[0]) ? 1.0 : -1.0;
  p(11) = isLeft(next[1]) ? 1.0 : -1.0;
  return p;
}

/** Robot inequalities of a single branch. All entries are written in the form g >= 0. */
template <typename Vec>
void appendRobotInequalities(Vec& g, int& idx, const Vec& x, const Vec& u, const Vec& xNext, const OptiPessiModelParameters& params,
                             const Vec& hipsAndSigns) {
  using Scalar = typename Vec::Scalar;
  using Vec2 = Eigen::Matrix<Scalar, 2, 1>;

  const Scalar epsAlpha = Scalar(params.alphaReduction);
  const Scalar alpha = u(RobotU::ALPHA);
  const Scalar dt = u(RobotU::DT);
  const Scalar beta = u(RobotU::BETA);
  const Scalar gamma = u(RobotU::GAMMA);

  // Input bounds.
  g(idx++) = alpha - epsAlpha;
  g(idx++) = Scalar(1) - epsAlpha - alpha;
  g(idx++) = beta;
  g(idx++) = Scalar(1) - beta;
  g(idx++) = gamma;
  g(idx++) = Scalar(1) - gamma;
  g(idx++) = dt - Scalar(params.dtMin);
  g(idx++) = Scalar(params.dtMax) - dt;

  // Circular friction cones. Normal loads are split by alpha: fn0 = alpha*m*g, fn1 = (1-alpha)*m*g.
  // Normalized by the squared cone limit so all rows of g have comparable magnitude.
  const Vec2 c(x(RobotX::CX), x(RobotX::CY));
  const Vec2 p0(x(RobotX::P0X), x(RobotX::P0Y));
  const Vec2 p1(x(RobotX::P1X), x(RobotX::P1Y));
  Vec2 f0, f1;
  computeTangentialForces(c, p0, p1, alpha, beta, gamma, Scalar(params.omega()), Scalar(params.mass), f0, f1);
  const Scalar fn0 = alpha * Scalar(params.frictionCoefficient) * Scalar(params.mass) * Scalar(params.gravity);
  const Scalar fn1 = (Scalar(1) - alpha) * Scalar(params.frictionCoefficient) * Scalar(params.mass) * Scalar(params.gravity);
  const Scalar fn0sq = fn0 * fn0 + Scalar(1e-3);
  const Scalar fn1sq = fn1 * fn1 + Scalar(1e-3);
  g(idx++) = Scalar(1) - (f0(0) * f0(0) + f0(1) * f0(1)) / fn0sq;
  g(idx++) = Scalar(1) - (f1(0) * f1(0) + f1(1) * f1(1)) / fn1sq;

  // Velocity bounds at knot i+1, expressed in the body frame of knot i.
  const Scalar theta = x(RobotX::TH);
  const Vec2 dcNext(xNext(RobotX::DCX), xNext(RobotX::DCY));
  const Vec2 dcBody = applyR01(theta, dcNext);
  g(idx++) = Scalar(params.dcxMax) * Scalar(params.dcxMax) - dcBody(0) * dcBody(0);
  g(idx++) = Scalar(params.dcyMax) * Scalar(params.dcyMax) - dcBody(1) * dcBody(1);
  g(idx++) = Scalar(params.dthetaMax) * Scalar(params.dthetaMax) - xNext(RobotX::DTH) * xNext(RobotX::DTH);

  // Foot reachability and left/right separation, all in the body frame of the NEXT base pose.
  const Vec2 cN(xNext(RobotX::CX), xNext(RobotX::CY));
  const Scalar thN = xNext(RobotX::TH);
  const Vec2 hip0(hipsAndSigns(0), hipsAndSigns(1));
  const Vec2 hip1(hipsAndSigns(2), hipsAndSigns(3));
  const Vec2 hip0Next(hipsAndSigns(4), hipsAndSigns(5));
  const Vec2 hip1Next(hipsAndSigns(6), hipsAndSigns(7));
  const Scalar side0 = hipsAndSigns(8);
  const Scalar side1 = hipsAndSigns(9);
  const Scalar side0Next = hipsAndSigns(10);
  const Scalar side1Next = hipsAndSigns(11);
  const Scalar rHipSq = Scalar(params.footHipMax) * Scalar(params.footHipMax);

  auto reach = [&](const Vec2& pWorld, const Vec2& hipBody, const Scalar& side) {
    const Vec2 pBody = applyR01(thN, pWorld - cN);
    const Vec2 e = pBody - hipBody;
    g(idx++) = rHipSq - e.dot(e);
    g(idx++) = side * pBody(1);
  };
  const Vec2 p0Next(xNext(RobotX::P0X), xNext(RobotX::P0Y));
  const Vec2 p1Next(xNext(RobotX::P1X), xNext(RobotX::P1Y));
  reach(p0, hip0, side0);
  reach(p1, hip1, side1);
  reach(p0Next, hip0Next, side0Next);
  reach(p1Next, hip1Next, side1Next);
}

/**
 * Separating-hyperplane certificate: every hip (and optionally the two next feet) must lie in
 * a^T y + b <= 0, while the obstacle centre must satisfy a^T o + b >= dMin. Together with
 * ||a|| = 1 this proves the convex hull of the robot points is at least dMin from the centre.
 */
template <typename Vec, typename Scalar>
void appendCollisionInequalities(Vec& g, int& idx, const Eigen::Matrix<Scalar, 2, 1>& com, const Scalar& theta,
                                 const Eigen::Matrix<Scalar, 2, 1>& a, const Scalar& b, const Eigen::Matrix<Scalar, 2, 1>& o,
                                 const Scalar& dMin, const OptiPessiModelParameters& params, bool withFeet,
                                 const Eigen::Matrix<Scalar, 2, 1>& p0, const Eigen::Matrix<Scalar, 2, 1>& p1) {
  using Vec2 = Eigen::Matrix<Scalar, 2, 1>;
  for (int f = 0; f < 4; ++f) {
    const auto& hipOffset = params.hipOffsets[static_cast<size_t>(f)];
    const Vec2 hip(Scalar(hipOffset(0)), Scalar(hipOffset(1)));
    const Vec2 hipWorld = com + applyR(theta, hip);
    g(idx++) = -(a.dot(hipWorld) + b);
  }
  if (withFeet) {
    g(idx++) = -(a.dot(p0) + b);
    g(idx++) = -(a.dot(p1) + b);
  }
  g(idx++) = a.dot(o) + b - dMin - Scalar(1e-3);
}

}  // namespace

StageInequalityConstraint::StageInequalityConstraint(OptiPessiModelParameters params, const OptiPessiReferenceManager& referenceManager,
                                                     const std::string& libraryFolder, bool recompile)
    : ocs2::StateInputConstraintCppAd(ocs2::ConstraintOrder::Linear),
      params_(std::move(params)),
      referenceManagerPtr_(&referenceManager) {
  numConstraints_ = static_cast<size_t>(stageInequalityCount(params_));
  initialize(static_cast<size_t>(params_.stateDim()), static_cast<size_t>(params_.inputDim()),
             static_cast<size_t>(parameterDim(params_)), "opti_pessi_stage_inequality", libraryFolder, recompile, true);
}

vector_t StageInequalityConstraint::getParameters(scalar_t time, const ocs2::PreComputation&) const {
  const int i = intervalIndex(time, params_.N);
  const int numObstacles = params_.numObstacles();
  vector_t p = vector_t::Zero(parameterDim(params_));
  p.head(kHipsAndSignsDim) = hipsAndSignsParameters(params_, *referenceManagerPtr_, i);
  const matrix_t& obstacles = referenceManagerPtr_->getObstacles();
  for (int j = 0; j < numObstacles; ++j) {
    p(kHipsAndSignsDim + 2 * j) = obstacles(j, 0);
    p(kHipsAndSignsDim + 2 * j + 1) = obstacles(j, 1);
  }
  p(kHipsAndSignsDim + 2 * numObstacles) = referenceManagerPtr_->getPessiScale();
  return p;
}

ocs2::ad_vector_t StageInequalityConstraint::constraintFunction(ocs2::ad_scalar_t, const ocs2::ad_vector_t& state,
                                                                 const ocs2::ad_vector_t& input,
                                                                 const ocs2::ad_vector_t& parameters) const {
  using Scalar = ocs2::ad_scalar_t;
  using Vec = ocs2::ad_vector_t;
  using Vec2 = Eigen::Matrix<Scalar, 2, 1>;

  Vec g(static_cast<int>(numConstraints_));
  g.setZero();
  int idx = 0;

  const int numObstacles = params_.numObstacles();
  const Vec hipsAndSigns = parameters.head(kHipsAndSignsDim);
  const Scalar w = Scalar(params_.omega());
  const Scalar mass = Scalar(params_.mass);
  const Scalar inertia = Scalar(params_.inertia);

  // uBranch is the compact per-branch input: [u(8), hyperplanes(6 * numObstacles)].
  auto appendBranch = [&](const Vec& x, const Vec& uBranch, const Scalar& keepOutGrowth) {
    const Vec xNext = lipMap(x, uBranch, w, mass, inertia);
    appendRobotInequalities(g, idx, x, uBranch, xNext, params_, hipsAndSigns);

    const Vec2 c(x(RobotX::CX), x(RobotX::CY));
    const Scalar theta = x(RobotX::TH);
    const Vec2 cN(xNext(RobotX::CX), xNext(RobotX::CY));
    const Scalar thN = xNext(RobotX::TH);
    const Vec2 cMid = Scalar(0.5) * (c + cN);
    const Scalar thMid = Scalar(0.5) * (theta + thN);
    const Vec2 p0Next(xNext(RobotX::P0X), xNext(RobotX::P0Y));
    const Vec2 p1Next(xNext(RobotX::P1X), xNext(RobotX::P1Y));

    const int hyperplaneOffset = RobotU::DIM;
    for (int j = 0; j < numObstacles; ++j) {
      const int base = hyperplaneOffset + kHyperplaneVarsPerObs * j;
      // Unit normals by construction -- see the note on kHyperplaneVarsPerObs in definitions.h.
      const Scalar phiMid = uBranch(base + Hyperplane::PHI_MID);
      const Scalar phiLand = uBranch(base + Hyperplane::PHI_LAND);
      const Vec2 aMid(wrapCos(phiMid), wrapSin(phiMid));
      const Vec2 aLand(wrapCos(phiLand), wrapSin(phiLand));
      const Scalar bMid = uBranch(base + Hyperplane::B_MID);
      const Scalar bLand = uBranch(base + Hyperplane::B_LAND);
      const Vec2 o(parameters(kHipsAndSignsDim + 2 * j), parameters(kHipsAndSignsDim + 2 * j + 1));
      const Scalar dMin = Scalar(params_.obstacleRadius) + keepOutGrowth;
      appendCollisionInequalities(g, idx, cMid, thMid, aMid, bMid, o, dMin, params_, false, p0Next, p1Next);
      appendCollisionInequalities(g, idx, cN, thN, aLand, bLand, o, dMin, params_, true, p0Next, p1Next);
    }
  };

  // Elapsed pessimistic time at the END of this interval: clock at knot i plus this interval's dt.
  const Scalar elapsed = state(CLOCK_INDEX);
  const Scalar dtPessi = input(RobotU::DIM + RobotU::DT);
  const Scalar pessiScale = parameters(kHipsAndSignsDim + 2 * numObstacles);

  const int hyperplaneBlock = kHyperplaneVarsPerObs * numObstacles;
  Vec uOpti(RobotU::DIM + hyperplaneBlock);
  uOpti.head(RobotU::DIM) = input.head(RobotU::DIM);
  uOpti.tail(hyperplaneBlock) = input.segment(optiHyperplaneOffset(), hyperplaneBlock);

  Vec uPessi(RobotU::DIM + hyperplaneBlock);
  uPessi.head(RobotU::DIM) = input.segment(RobotU::DIM, RobotU::DIM);
  uPessi.tail(hyperplaneBlock) = input.segment(pessiHyperplaneOffset(numObstacles), hyperplaneBlock);

  // Optimistic: frozen obstacle disk. Pessimistic: worst-case reachable disk.
  appendBranch(state.head(RobotX::DIM), uOpti, Scalar(0));
  appendBranch(state.segment(RobotX::DIM, RobotX::DIM), uPessi,
               pessiScale * Scalar(params_.obstacleMaxSpeed) * (elapsed + dtPessi));

  return g;
}

}  // namespace opti_pessi
