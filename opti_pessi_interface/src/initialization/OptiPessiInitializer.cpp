#include "opti_pessi_interface/initialization/OptiPessiInitializer.h"

#include <algorithm>
#include <cmath>

#include "opti_pessi_interface/LipKinematics.h"

namespace opti_pessi {

vector_t packInitialState(const vector_t& robotState) {
  vector_t x = vector_t::Zero(AUG_STATE_DIM);
  x.segment(0, RobotX::DIM) = robotState;
  x.segment(RobotX::DIM, RobotX::DIM) = robotState;
  return x;  // clock starts at zero
}

vector_t extractRobotState(const vector_t& augmentedState) {
  return augmentedState.head(RobotX::DIM);
}

vector_t extractRobotInput(const vector_t& augmentedInput) {
  return augmentedInput.head(RobotU::DIM);
}

vector_t defaultRobotInput(const OptiPessiModelParameters& params, const vector_t& robotState, int knotParity,
                           const vector_t& initBias) {
  vector_t u = vector_t::Zero(RobotU::DIM);
  const auto next = gaitPair(knotParity + 1);
  const vector_t c = robotState.head(2);
  const vector_t dc = robotState.segment(RobotX::DCX, 2);
  const scalar_t theta = robotState(RobotX::TH);
  const scalar_t w = params.omega();
  const scalar_t dt = params.dtCost0;

  // Project the CoM forward with the LIP velocity term before placing the footholds.
  const vector_t cNextApprox = c + (std::sinh(w * dt) / w) * dc;
  u.segment(RobotU::P0X, 2) = cNextApprox + applyR(theta, hipOf(params, next[0])) + initBias;
  u.segment(RobotU::P1X, 2) = cNextApprox + applyR(theta, hipOf(params, next[1])) + initBias;
  u(RobotU::ALPHA) = 0.5;
  u(RobotU::DT) = params.dtCost0;
  u(RobotU::BETA) = 0.5;
  u(RobotU::GAMMA) = 0.5;
  return u;
}

void seedHyperplanes(vector_t& input, int hyperplaneOffset, const OptiPessiModelParameters& params, const vector_t& robotState,
                     const OptiPessiReferenceManager& referenceManager) {
  const vector_t c = robotState.head(2);
  const scalar_t theta = robotState(RobotX::TH);
  const matrix_t& obstacles = referenceManager.getObstacles();

  for (int j = 0; j < params.numObstacles(); ++j) {
    // Normal points from the robot towards the obstacle.
    vector_t a(2);
    a(0) = obstacles(j, 0) - c(0);
    a(1) = obstacles(j, 1) - c(1);
    const scalar_t norm = a.norm();
    if (norm < 1e-9) {
      a << 1.0, 0.0;
    } else {
      a /= norm;
    }

    // Offset placed just past the most advanced robot point, so a^T y + b <= 0 holds for all hips.
    scalar_t maxProjection = a.dot(c);
    for (int f = 0; f < 4; ++f) {
      const vector_t hipWorld = c + applyR(theta, hipOf(params, static_cast<Foot>(f)));
      maxProjection = std::max(maxProjection, a.dot(hipWorld));
    }
    const scalar_t b = -maxProjection;

    const int base = hyperplaneOffset + kHyperplaneVarsPerObs * j;
    input(base + Hyperplane::PHI) = std::atan2(a(1), a(0));
    input(base + Hyperplane::B) = b;
  }
}

vector_t augmentedLipStep(const OptiPessiModelParameters& params, const vector_t& augmentedState, const vector_t& augmentedInput) {
  const scalar_t w = params.omega();
  vector_t xNext = augmentedState;
  xNext.segment(0, RobotX::DIM) =
      lipMapScalar(augmentedState.head(RobotX::DIM), augmentedInput.head(RobotU::DIM), w, params.mass, params.inertia);
  xNext.segment(RobotX::DIM, RobotX::DIM) = lipMapScalar(augmentedState.segment(RobotX::DIM, RobotX::DIM),
                                                         augmentedInput.segment(RobotU::DIM, RobotU::DIM), w, params.mass,
                                                         params.inertia);
  xNext(CLOCK_INDEX) = augmentedState(CLOCK_INDEX) + augmentedInput(RobotU::DIM + RobotU::DT);
  return xNext;
}

namespace {

/** Fills one augmented input with the nominal inputs and seeded hyperplanes of both branches. */
vector_t nominalAugmentedInput(const OptiPessiModelParameters& params, const OptiPessiReferenceManager& referenceManager,
                               const vector_t& augmentedState, int knotParity) {
  const int numObstacles = params.numObstacles();
  vector_t u = vector_t::Zero(augInputDim(numObstacles));

  const vector_t xOpti = augmentedState.segment(0, RobotX::DIM);
  const vector_t xPessi = augmentedState.segment(RobotX::DIM, RobotX::DIM);

  u.segment(0, RobotU::DIM) = defaultRobotInput(params, xOpti, knotParity, referenceManager.getInitBias());
  u.segment(RobotU::DIM, RobotU::DIM) = defaultRobotInput(params, xPessi, knotParity, referenceManager.getInitBias());
  seedHyperplanes(u, optiHyperplaneOffset(), params, xOpti, referenceManager);
  seedHyperplanes(u, pessiHyperplaneOffset(numObstacles), params, xPessi, referenceManager);
  return u;
}

}  // namespace

OptiPessiInitializer::OptiPessiInitializer(OptiPessiModelParameters params, const OptiPessiReferenceManager& referenceManager)
    : params_(std::move(params)), referenceManagerPtr_(&referenceManager) {}

void OptiPessiInitializer::compute(scalar_t time, const vector_t& state, scalar_t /*nextTime*/, vector_t& input,
                                   vector_t& nextState) {
  const int i = intervalIndex(time, params_.N);
  const int knotParity = referenceManagerPtr_->getGaitOffset() + i;
  input = nominalAugmentedInput(params_, *referenceManagerPtr_, state, knotParity);
  nextState = augmentedLipStep(params_, state, input);
}

ocs2::PrimalSolution shiftPrimalSolution(const ocs2::PrimalSolution& solution, const vector_t& robotState) {
  ocs2::PrimalSolution out = solution;
  const int numStates = static_cast<int>(solution.stateTrajectory_.size());
  const int numInputs = static_cast<int>(solution.inputTrajectory_.size());
  if (numStates < 2 || numInputs < 1) {
    return out;
  }

  // Shift the inputs (hyperplane variables included, they live in the input) one knot forward,
  // repeating the last one.
  for (int i = 0; i < numInputs - 1; ++i) {
    out.inputTrajectory_[static_cast<size_t>(i)] = solution.inputTrajectory_[static_cast<size_t>(i + 1)];
  }
  out.inputTrajectory_[static_cast<size_t>(numInputs - 1)] = solution.inputTrajectory_[static_cast<size_t>(numInputs - 1)];

  // Shift the STORED states rather than re-integrating them.
  //
  // Re-integrating (x_{i+1} = lipMap(x_i, u_i) from the new measured x_0) looks more principled --
  // it makes the guess satisfy the dynamics exactly -- but it is wrong here: the LIP is
  // exponentially unstable, |dc_{i+1}/dc_i| = cosh(omega*dt) ~ 2.96, and the footholds in u are
  // ABSOLUTE world positions. Any mismatch between the measured state and the previous plan is
  // therefore amplified ~3x per knot, ~700x across the horizon, and compounds every MPC step. The
  // solver then cannot repair the guess and simply returns it, and the closed loop diverges.
  //
  // The reference implementation shifts the stored trajectory (ocp_quadruped.py:455-457,
  // `set_initial(x[:, i], x_guess[:, i + 1])`), which stays near the previous solution. Only the
  // first knot is replaced by the measurement; the solver closes the resulting defects itself.
  for (int i = 0; i < numStates - 1; ++i) {
    out.stateTrajectory_[static_cast<size_t>(i)] = solution.stateTrajectory_[static_cast<size_t>(i + 1)];
  }
  out.stateTrajectory_[static_cast<size_t>(numStates - 1)] = solution.stateTrajectory_[static_cast<size_t>(numStates - 1)];
  out.stateTrajectory_[0] = packInitialState(robotState);
  return out;
}

ocs2::PrimalSolution rolloutGuess(const OptiPessiModelParameters& params, const OptiPessiReferenceManager& referenceManager,
                                  const vector_t& robotState) {
  ocs2::PrimalSolution solution;
  vector_t x = packInitialState(robotState);
  const int N = params.N;

  for (int i = 0; i <= N; ++i) {
    solution.timeTrajectory_.push_back(static_cast<scalar_t>(i));
    solution.stateTrajectory_.push_back(x);
    if (i == N) {
      break;
    }
    const int knotParity = referenceManager.getGaitOffset() + i;
    const vector_t u = nominalAugmentedInput(params, referenceManager, x, knotParity);
    solution.inputTrajectory_.push_back(u);
    x = augmentedLipStep(params, x, u);
  }
  if (!solution.inputTrajectory_.empty()) {
    solution.inputTrajectory_.push_back(solution.inputTrajectory_.back());
  }
  return solution;
}

}  // namespace opti_pessi
