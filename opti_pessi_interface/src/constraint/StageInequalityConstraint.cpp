#include "opti_pessi_interface/constraint/StageInequalityConstraint.h"

#include "opti_pessi_interface/LipKinematics.h"

namespace opti_pessi {

namespace {

/**
 * STEP-BY-STEP TRANSCRIPTION -- read this before changing anything here.
 *
 * Every row below is written on INTERVAL i and is a function of (x_i, u_i) only. Where the reference
 * writes a bound on knot i+1, this file recomputes that knot explicitly,
 *
 *     x_{i+1} = lipMap(x_i, u_i),
 *
 * inside the constraint, exactly as ocp_quadruped.py does when it substitutes the dynamics into its
 * own bounds. Nothing is carried in the state to avoid the composition: the branch state is the
 * reference's R^10 and nothing else (see definitions.h).
 *
 * Consequences worth knowing:
 *   - the Jacobians see cosh/sinh of the decision variable dt, products with alpha, and absolute
 *     world positions. The QP Hessian is correspondingly worse conditioned than a knot-local form,
 *     which is what hpipm's reg_prim in config/task.info is there to absorb.
 *   - interval 0 is now constrained and MUST be: its rows bound x_1 = lipMap(x_0, u_0), the state the
 *     applied input actually lands the robot in, and x_0 itself (the measurement) appears in them
 *     only through the map. There is no longer any reason to skip it -- see isActive().
 */

/**
 * 3 velocity/yaw-rate bounds at knot i+1 + 8 reachability rows: this interval's stance feet AND the
 * commanded next footholds, each {distance to own hip, left/right side}, all measured against knot
 * i+1's base (ocp_quadruped.py:116-128). The stance-foot family is what caps stride length, and
 * hence top speed.
 */
constexpr int kPathRowsPerBranch = 11;

/**
 * Per obstacle, two independent separating planes (see definitions.h):
 *   mid-step: 4 hips on the safe side + the obstacle on the far side       = 5
 *   landing:  4 hips + 2 landing feet on the safe side + the obstacle      = 7
 */
constexpr int kCollisionRowsPerObstacle = 12;

/**
 * Parameters: hip offsets and left/right signs for THIS interval's stance pair and for the NEXT
 * knot's stance pair (the trot alternates), then the obstacle centres and the pessimistic scale.
 */
constexpr int kHipsAndSignsDim = 12;

int rowsPerBranch(const OptiPessiModelParameters& params) {
  return kPathRowsPerBranch + kCollisionRowsPerObstacle * params.numObstacles();
}

int stageInequalityCount(const OptiPessiModelParameters& params) {
  return 2 * rowsPerBranch(params);  // optimistic + pessimistic
}

int parameterDim(const OptiPessiModelParameters& params) {
  return kHipsAndSignsDim + 2 * params.numObstacles() + 1;
}

/**
 * Hip offsets of this interval's stance pair and of the next knot's stance pair, plus their
 * left/right signs, so the generated AD library stays gait-agnostic and only the parameter vector
 * changes as the trot alternates.
 */
vector_t hipsAndSignsParameters(const OptiPessiModelParameters& params, const OptiPessiReferenceManager& referenceManager, int i) {
  const auto stance = gaitPair(referenceManager.getGaitOffset() + i);
  const auto next = gaitPair(referenceManager.getGaitOffset() + i + 1);
  vector_t p(kHipsAndSignsDim);
  p.segment(0, 2) = hipOf(params, stance[0]);
  p.segment(2, 2) = hipOf(params, stance[1]);
  p(4) = isLeft(stance[0]) ? 1.0 : -1.0;
  p(5) = isLeft(stance[1]) ? 1.0 : -1.0;
  p.segment(6, 2) = hipOf(params, next[0]);
  p.segment(8, 2) = hipOf(params, next[1]);
  p(10) = isLeft(next[0]) ? 1.0 : -1.0;
  p(11) = isLeft(next[1]) ? 1.0 : -1.0;
  return p;
}

/**
 * Velocity/yaw-rate bounds and foot reachability, all evaluated on the successor knot. Rows are
 * g >= 0.
 *
 * The velocity box is rotated into THIS knot's body frame while bounding the NEXT knot's velocity,
 * which is what the reference does (ocp_quadruped.py:265, `dc_bound(x[3:5, i+1], ..., x[2, i])`).
 */
template <typename Vec>
void appendPathRows(Vec& g, int& idx, const Vec& x, const Vec& xNext, const OptiPessiModelParameters& params,
                    const Vec& hipsAndSigns) {
  using Scalar = typename Vec::Scalar;
  using Vec2 = Eigen::Matrix<Scalar, 2, 1>;

  // CoM velocity box at knot i+1, expressed in knot i's body frame, plus the yaw-rate bound.
  const Vec2 dcNext(xNext(RobotX::DCX), xNext(RobotX::DCY));
  const Vec2 dcBody = applyR01(Scalar(x(RobotX::TH)), dcNext);
  g(idx++) = Scalar(params.dcxMax) * Scalar(params.dcxMax) - dcBody(0) * dcBody(0);
  g(idx++) = Scalar(params.dcyMax) * Scalar(params.dcyMax) - dcBody(1) * dcBody(1);
  g(idx++) = Scalar(params.dthetaMax) * Scalar(params.dthetaMax) - xNext(RobotX::DTH) * xNext(RobotX::DTH);

  // Both foot families are measured against the base the robot has moved to, (c_{i+1}, theta_{i+1}).
  const Vec2 cNext(xNext(RobotX::CX), xNext(RobotX::CY));
  const Scalar thetaNext = xNext(RobotX::TH);
  const Scalar rHipSq = Scalar(params.footHipMax) * Scalar(params.footHipMax);

  auto reach = [&](const Vec2& footWorld, const Vec2& hipBody, const Scalar& side) {
    const Vec2 footBody = applyR01(thetaNext, footWorld - cNext);
    const Vec2 e = footBody - hipBody;
    g(idx++) = rHipSq - e.dot(e);
    g(idx++) = side * footBody(1);
  };

  // The feet the robot stands on during THIS phase must still be within reach of the base it has
  // moved to by the end of it. This is what caps stride length.
  const Vec2 hip0(hipsAndSigns(0), hipsAndSigns(1));
  const Vec2 hip1(hipsAndSigns(2), hipsAndSigns(3));
  reach(Vec2(x(RobotX::P0X), x(RobotX::P0Y)), hip0, Scalar(hipsAndSigns(4)));
  reach(Vec2(x(RobotX::P1X), x(RobotX::P1Y)), hip1, Scalar(hipsAndSigns(5)));

  // The commanded landing footholds must be reachable from that same base.
  const Vec2 nextHip0(hipsAndSigns(6), hipsAndSigns(7));
  const Vec2 nextHip1(hipsAndSigns(8), hipsAndSigns(9));
  reach(Vec2(xNext(RobotX::P0X), xNext(RobotX::P0Y)), nextHip0, Scalar(hipsAndSigns(10)));
  reach(Vec2(xNext(RobotX::P1X), xNext(RobotX::P1Y)), nextHip1, Scalar(hipsAndSigns(11)));
}

/**
 * Separating-hyperplane certificate at a pose: every hip must lie in a^T y + b <= 0 while the
 * obstacle centre satisfies a^T o + b >= dMin. With a = (cos phi, sin phi) the unit-norm condition
 * holds identically, so this proves the robot's convex hull clears the disk of radius dMin about
 * the centre.
 */
template <typename Vec, typename Scalar>
void appendHipRows(Vec& g, int& idx, const Eigen::Matrix<Scalar, 2, 1>& com, const Scalar& theta,
                   const Eigen::Matrix<Scalar, 2, 1>& a, const Scalar& b, const OptiPessiModelParameters& params) {
  using Vec2 = Eigen::Matrix<Scalar, 2, 1>;
  for (int f = 0; f < 4; ++f) {
    const auto& hipOffset = params.hipOffsets[static_cast<size_t>(f)];
    const Vec2 hip(Scalar(hipOffset(0)), Scalar(hipOffset(1)));
    g(idx++) = -(a.dot(com + applyR(theta, hip)) + b);
  }
}

/**
 * The obstacle centre must clear the plane by dMin.
 *
 * Hard -- see the slack note in definitions.h for why this row is not softened with a decision
 * variable. The reference uses the SAME dMin for the mid-step and the landing plane of an interval
 * (mpc_formulation.md sec. 7.2), so both callers pass the same value.
 */
template <typename Vec, typename Scalar>
void appendObstacleRow(Vec& g, int& idx, const Eigen::Matrix<Scalar, 2, 1>& a, const Scalar& b,
                       const Eigen::Matrix<Scalar, 2, 1>& o, const Scalar& dMin) {
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

bool StageInequalityConstraint::isActive(scalar_t /*time*/) const {
  // Active on every interval, interval 0 included. These rows bound x_{i+1} = lipMap(x_i, u_i), so
  // on interval 0 they bound x_1 -- the state the applied input lands in -- and never x_0, which is
  // pinned to the measurement. Skipping interval 0 would leave the executed step unconstrained; it
  // is what the reference constrains first (ocp_quadruped.py:262-284, loop from i = 0).
  return true;
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
  const Scalar pessiScale = parameters(kHipsAndSignsDim + 2 * numObstacles);
  const Scalar w = Scalar(params_.omega());
  const Scalar mass = Scalar(params_.mass);
  const Scalar inertia = Scalar(params_.inertia);

  auto appendBranch = [&](const Vec& x, const Vec& u, int hyperplaneOffset, const Scalar& keepOutGrowth) {
    // Step the dynamics explicitly: this is the successor knot every row below is written on.
    const Vec xNext = lipMap(x, u, w, mass, inertia);

    appendPathRows(g, idx, x, xNext, params_, hipsAndSigns);

    const Vec2 c(x(RobotX::CX), x(RobotX::CY));
    const Scalar theta = x(RobotX::TH);
    const Vec2 cNext(xNext(RobotX::CX), xNext(RobotX::CY));
    const Scalar thetaNext = xNext(RobotX::TH);
    const Vec2 p0Next(xNext(RobotX::P0X), xNext(RobotX::P0Y));
    const Vec2 p1Next(xNext(RobotX::P1X), xNext(RobotX::P1Y));

    // Mid-step pose: halfway along this interval (ocp_quadruped.py:279-280).
    const Vec2 cMid = Scalar(0.5) * (c + cNext);
    const Scalar thetaMid = Scalar(0.5) * (theta + thetaNext);

    for (int j = 0; j < numObstacles; ++j) {
      const int base = hyperplaneOffset + kHyperplaneVarsPerObs * j;
      const Vec2 o(parameters(kHipsAndSignsDim + 2 * j), parameters(kHipsAndSignsDim + 2 * j + 1));
      const Scalar dMin = Scalar(params_.obstacleRadius) + keepOutGrowth;

      // Mid-step plane: hips only, at the half-way pose (reference: collision_avoidance without p).
      const Scalar phiMid = input(base + Hyperplane::MID_PHI);
      const Vec2 aMid(wrapCos(phiMid), wrapSin(phiMid));  // unit by construction
      const Scalar bMid = input(base + Hyperplane::MID_B);
      appendHipRows(g, idx, cMid, thetaMid, aMid, bMid, params_);
      appendObstacleRow(g, idx, aMid, bMid, o, dMin);

      // Landing plane: hips AND both landing feet, at the successor knot's pose.
      const Scalar phiLand = input(base + Hyperplane::LAND_PHI);
      const Vec2 aLand(wrapCos(phiLand), wrapSin(phiLand));
      const Scalar bLand = input(base + Hyperplane::LAND_B);
      appendHipRows(g, idx, cNext, thetaNext, aLand, bLand, params_);
      g(idx++) = -(aLand.dot(p0Next) + bLand);
      g(idx++) = -(aLand.dot(p1Next) + bLand);
      appendObstacleRow(g, idx, aLand, bLand, o, dMin);
    }
  };

  // The clock holds the pessimistic elapsed time accumulated UP TO this knot; the rows below live at
  // knot i+1, so the worst-case disk has had dt_i^pessi more to grow. That matches the reference,
  // which accumulates d_min += dt[i] * v_obstacle before using it on interval i
  // (ocp_quadruped.py:510). The optimistic branch sees the frozen disk.
  const Scalar elapsedNext = state(CLOCK_INDEX) + input(RobotU::DIM + RobotU::DT);
  appendBranch(state.head(RobotX::DIM), input.head(RobotU::DIM), optiHyperplaneOffset(), Scalar(0));
  appendBranch(state.segment(RobotX::DIM, RobotX::DIM), input.segment(RobotU::DIM, RobotU::DIM),
               pessiHyperplaneOffset(numObstacles), pessiScale * Scalar(params_.obstacleMaxSpeed) * elapsedNext);

  return g;
}

}  // namespace opti_pessi
