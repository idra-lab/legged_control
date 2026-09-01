#include "opti_pessi_interface/constraint/StageInequalityConstraint.h"

#include "opti_pessi_interface/LipKinematics.h"

namespace opti_pessi {

namespace {

/**
 * KNOT-LOCAL TRANSCRIPTION -- read this before changing anything here.
 *
 * Every row below is a function of the state at THIS knot and the input at this knot. Nothing is
 * composed with the dynamics.
 *
 * The reference implementation instead re-derived the next state as lipMap(x_i, u_i) inside each
 * constraint, because the Python original writes its bounds on x[:, i+1] and OCS2's stage-constraint
 * API only sees (t, x_i, u_i). Both express the same feasible set -- the shooting defect forces
 * x_{i+1} = lipMap(x_i, u_i) at any solution -- but they are wildly different optimization problems:
 * composing with the dynamics pushes cosh/sinh of the decision variable dt, products with alpha, and
 * the absolute world position through every Jacobian the QP sees. The resulting Hessian is so badly
 * conditioned that the solver needed hpipm reg_prim ~ 1e2 to take a step at all, and still converged
 * to points with cost ~1e6.
 *
 * Multiple shooting already carries x_{i+1} as a decision variable, so the constraint can simply be
 * imposed at the knot where the quantity lives -- which is exactly what CasADi's transcription does,
 * and why it converges where the composed form does not. Bounds that the Python writes on knot i+1
 * are therefore written here on knot i and imposed over knots 1..N.
 */

/**
 * 3 velocity/yaw-rate bounds + 8 reachability rows: the current stance feet AND the previous stance
 * feet, each {distance to own hip, left/right side}, all measured against THIS knot's base. The
 * previous-foot family is the reference's "planted feet must remain reachable once the CoM has
 * moved" (ocp_quadruped.py:118-121); it is what actually limits stride length, and hence top speed.
 */
constexpr int kPathRowsPerBranch = 11;

/** Per obstacle: 4 hips + 2 stance feet on the safe side, plus the obstacle on the far side. */
constexpr int kCollisionRowsPerObstacle = 7;

/**
 * Parameters: hip offsets and left/right signs for THIS knot's stance pair and for the previous
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
 * Hip offsets of THIS knot's stance pair plus their left/right signs, so the generated AD library
 * stays gait-agnostic and only the parameter vector changes as the trot alternates.
 */
vector_t hipsAndSignsParameters(const OptiPessiModelParameters& params, const OptiPessiReferenceManager& referenceManager, int i) {
  const auto stance = gaitPair(referenceManager.getGaitOffset() + i);
  const auto previous = gaitPair(referenceManager.getGaitOffset() + i - 1);
  vector_t p(kHipsAndSignsDim);
  p.segment(0, 2) = hipOf(params, stance[0]);
  p.segment(2, 2) = hipOf(params, stance[1]);
  p(4) = isLeft(stance[0]) ? 1.0 : -1.0;
  p(5) = isLeft(stance[1]) ? 1.0 : -1.0;
  p.segment(6, 2) = hipOf(params, previous[0]);
  p.segment(8, 2) = hipOf(params, previous[1]);
  p(10) = isLeft(previous[0]) ? 1.0 : -1.0;
  p(11) = isLeft(previous[1]) ? 1.0 : -1.0;
  return p;
}

/** Velocity/yaw-rate bounds and foot reachability, all evaluated at this knot. Rows are g >= 0. */
template <typename Vec>
void appendPathRows(Vec& g, int& idx, const Vec& x, const OptiPessiModelParameters& params, const Vec& hipsAndSigns) {
  using Scalar = typename Vec::Scalar;
  using Vec2 = Eigen::Matrix<Scalar, 2, 1>;

  // CoM velocity box in this knot's own body frame, plus the yaw-rate bound.
  const Scalar theta = x(RobotX::TH);
  const Vec2 dc(x(RobotX::DCX), x(RobotX::DCY));
  const Vec2 dcBody = applyR01(theta, dc);
  g(idx++) = Scalar(params.dcxMax) * Scalar(params.dcxMax) - dcBody(0) * dcBody(0);
  g(idx++) = Scalar(params.dcyMax) * Scalar(params.dcyMax) - dcBody(1) * dcBody(1);
  g(idx++) = Scalar(params.dthetaMax) * Scalar(params.dthetaMax) - x(RobotX::DTH) * x(RobotX::DTH);

  // Stance feet must be within reach of their own hips, and on the correct side of the body axis.
  const Vec2 c(x(RobotX::CX), x(RobotX::CY));
  const Vec2 hip0(hipsAndSigns(0), hipsAndSigns(1));
  const Vec2 hip1(hipsAndSigns(2), hipsAndSigns(3));
  const Scalar side0 = hipsAndSigns(4);
  const Scalar side1 = hipsAndSigns(5);
  const Scalar rHipSq = Scalar(params.footHipMax) * Scalar(params.footHipMax);

  auto reach = [&](const Vec2& footWorld, const Vec2& hipBody, const Scalar& side) {
    const Vec2 footBody = applyR01(theta, footWorld - c);
    const Vec2 e = footBody - hipBody;
    g(idx++) = rHipSq - e.dot(e);
    g(idx++) = side * footBody(1);
  };
  reach(Vec2(x(RobotX::P0X), x(RobotX::P0Y)), hip0, side0);
  reach(Vec2(x(RobotX::P1X), x(RobotX::P1Y)), hip1, side1);

  // The feet the robot was standing on during the previous phase must still be within reach of the
  // base it has now moved to. This is what caps stride length.
  const Vec2 prevHip0(hipsAndSigns(6), hipsAndSigns(7));
  const Vec2 prevHip1(hipsAndSigns(8), hipsAndSigns(9));
  reach(Vec2(x(RobotX::PP0X), x(RobotX::PP0Y)), prevHip0, hipsAndSigns(10));
  reach(Vec2(x(RobotX::PP1X), x(RobotX::PP1Y)), prevHip1, hipsAndSigns(11));
}

/**
 * Separating-hyperplane certificate at this knot: every hip and both stance feet must lie in
 * a^T y + b <= 0 while the obstacle centre satisfies a^T o + b >= dMin. With a = (cos phi, sin phi)
 * the unit-norm condition holds identically, so this proves the robot's convex hull clears the disk
 * of radius dMin about the centre.
 */
template <typename Vec, typename Scalar>
void appendCollisionRows(Vec& g, int& idx, const Eigen::Matrix<Scalar, 2, 1>& com, const Scalar& theta,
                         const Eigen::Matrix<Scalar, 2, 1>& p0, const Eigen::Matrix<Scalar, 2, 1>& p1,
                         const Eigen::Matrix<Scalar, 2, 1>& a, const Scalar& b, const Eigen::Matrix<Scalar, 2, 1>& o,
                         const Scalar& dMin, const OptiPessiModelParameters& params) {
  using Vec2 = Eigen::Matrix<Scalar, 2, 1>;
  for (int f = 0; f < 4; ++f) {
    const auto& hipOffset = params.hipOffsets[static_cast<size_t>(f)];
    const Vec2 hip(Scalar(hipOffset(0)), Scalar(hipOffset(1)));
    g(idx++) = -(a.dot(com + applyR(theta, hip)) + b);
  }
  g(idx++) = -(a.dot(p0) + b);
  g(idx++) = -(a.dot(p1) + b);
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

bool StageInequalityConstraint::isActive(scalar_t time) const {
  // Knot 0 is pinned to the measurement. Constraining it would make the whole OCP infeasible
  // whenever the robot is measured even slightly outside its own bounds, which is exactly the
  // situation in which the controller most needs to produce an answer. The reference writes these
  // bounds on knot i+1 for i = 0..N-1, i.e. knots 1..N, for the same reason.
  return time >= 0.5;
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

  auto appendBranch = [&](const Vec& x, int hyperplaneOffset, const Scalar& keepOutGrowth) {
    appendPathRows(g, idx, x, params_, hipsAndSigns);

    const Vec2 c(x(RobotX::CX), x(RobotX::CY));
    const Scalar theta = x(RobotX::TH);
    const Vec2 p0(x(RobotX::P0X), x(RobotX::P0Y));
    const Vec2 p1(x(RobotX::P1X), x(RobotX::P1Y));

    for (int j = 0; j < numObstacles; ++j) {
      const int base = hyperplaneOffset + kHyperplaneVarsPerObs * j;
      const Scalar phi = input(base + Hyperplane::PHI);
      const Vec2 a(wrapCos(phi), wrapSin(phi));  // unit by construction
      const Scalar b = input(base + Hyperplane::B);
      const Vec2 o(parameters(kHipsAndSignsDim + 2 * j), parameters(kHipsAndSignsDim + 2 * j + 1));
      const Scalar dMin = Scalar(params_.obstacleRadius) + keepOutGrowth;
      appendCollisionRows(g, idx, c, theta, p0, p1, a, b, o, dMin, params_);
    }
  };

  // The clock holds the pessimistic elapsed time already accumulated up to this knot, which is what
  // inflates the worst-case reachable disk. The optimistic branch sees the frozen disk.
  const Scalar elapsed = state(CLOCK_INDEX);
  appendBranch(state.head(RobotX::DIM), optiHyperplaneOffset(), Scalar(0));
  appendBranch(state.segment(RobotX::DIM, RobotX::DIM), pessiHyperplaneOffset(numObstacles),
               pessiScale * Scalar(params_.obstacleMaxSpeed) * elapsed);

  return g;
}

}  // namespace opti_pessi
