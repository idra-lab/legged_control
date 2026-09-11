#include "opti_pessi_interface/simulation/ClosedLoopSimulation.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <iostream>

#include "opti_pessi_interface/LipKinematics.h"
#include "opti_pessi_interface/initialization/OptiPessiInitializer.h"
#include "opti_pessi_interface/simulation/ObstaclePlant.h"

namespace opti_pessi {

namespace {

vector_t worldHip(const vector_t& robotState, const vector_t& hipBody) {
  return robotState.head(2) + applyR(robotState(RobotX::TH), hipBody);
}

/**
 * Emergency step used when the solver produces nothing usable: a capture-point (deadbeat) stop.
 *
 * The LIP's divergent mode is the DCM xi = c + dc/omega, which evolves as
 * xi_{i+1} = e^{omega*dt} (xi_i - z) + z. Placing the CoP z at the DCM therefore leaves the DCM
 * stationary instead of letting it grow by cosh(omega*dt) ~ 3 per phase. The previous heuristic
 * (z = c - 0.25*dc) is not stabilizing, and open-loop divergence turned a single failed solve into
 * a run-ending blow-up within four steps.
 *
 * The CoP is clamped to the current support segment (all that alpha can express), and the next
 * footholds are placed under the DCM so the following support polygon can actually contain it.
 *
 * NOTE the limit of this: the clamp is not cosmetic. Once the capture point leaves the support
 * segment the CoP cannot reach it and the step is no longer deadbeat -- it only slows the growth
 * (1.70x per phase measured, against 2.76x open loop). This routine cannot rescue an already-fast
 * state, and calling it repeatedly is divergence with extra steps; kDivergedSpeed is what stops it.
 */
vector_t fallbackInput(const OptiPessiModelParameters& params, const vector_t& robotState, int step) {
  vector_t u = vector_t::Zero(RobotU::DIM);
  const auto next = gaitPair(step + 1);
  const vector_t c = robotState.head(2);
  const vector_t dc = robotState.segment(RobotX::DCX, 2);
  const scalar_t theta = robotState(RobotX::TH);
  const scalar_t w = params.omega();

  // Divergent component of the LIP state.
  const vector_t dcm = c + dc / w;

  // Step under the capture point so the next stance can arrest the motion.
  u.segment(RobotU::P0X, 2) = dcm + applyR(theta, hipOf(params, next[0]));
  u.segment(RobotU::P1X, 2) = dcm + applyR(theta, hipOf(params, next[1]));

  // Put this phase's CoP as close to the capture point as the current support segment allows.
  const vector_t p0 = robotState.segment(RobotX::P0X, 2);
  const vector_t p1 = robotState.segment(RobotX::P1X, 2);
  const vector_t d = p1 - p0;
  const scalar_t denominator = d.dot(d);
  scalar_t alpha = 0.5;
  if (denominator > 1e-9) {
    alpha = (dcm - p0).dot(d) / denominator;
  }
  u(RobotU::ALPHA) = std::min(std::max(alpha, params.alphaReduction), scalar_t(1) - params.alphaReduction);
  u(RobotU::DT) = params.dtMin;  // shortest phase: re-plan as soon as possible
  u(RobotU::BETA) = 0.5;
  u(RobotU::GAMMA) = 0.5;
  return u;
}

/**
 * Clamps the free scalars of an input to their bounds, leaving the footholds alone. This is the
 * reference's `check_bounds_and_saturate` (mpc_utils.py:15-22): when a solve fails, its first input
 * is usually still informative, so it is saturated and applied rather than discarded.
 */
vector_t saturateRobotInput(vector_t u, const OptiPessiModelParameters& params) {
  u(RobotU::ALPHA) = std::min(std::max(u(RobotU::ALPHA), params.alphaReduction), scalar_t(1) - params.alphaReduction);
  u(RobotU::BETA) = std::min(std::max(u(RobotU::BETA), scalar_t(0)), scalar_t(1));
  u(RobotU::GAMMA) = std::min(std::max(u(RobotU::GAMMA), scalar_t(0)), scalar_t(1));
  u(RobotU::DT) = std::min(std::max(u(RobotU::DT), params.dtMin), params.dtMax);
  return u;
}

/**
 * Speed past which the capture-point fallback has provably lost authority, so the run is over.
 *
 * The fallback can only arrest the DCM if the capture point c + dc/omega lies inside the current
 * support segment, whose half-length is O(footHipMax) = 0.1 m. With omega = sqrt(g/h) = 5.08 that
 * caps the recoverable speed at well under 1 m/s; beyond it alpha saturates, the CoP cannot reach
 * the capture point, and the DCM grows every phase (measured: 1.70x per step at dt = dtMin, against
 * e^{omega*dt} = 2.76 open loop -- the fallback removes part of the divergence, never all of it).
 *
 * 10 m/s is ~6.7x dcxMax: not a borderline call, just the point past which continuing produces
 * nothing but garbage. Without this the run ground on to 141 m/s, logging a warning per step and
 * writing the result to the trajectory as if it meant something.
 */
constexpr scalar_t kDivergedSpeed = 10.0;

/** Numerical divergence: nothing sensible can be done, the run must stop. */
bool hasDiverged(const vector_t& robotState, const vector_t& robotInput) {
  return !robotState.allFinite() || !robotInput.allFinite() || robotState.head(2).norm() > 50.0 ||
         robotState.segment(RobotX::DCX, 2).norm() > kDivergedSpeed;
}

}  // namespace

ClosedLoopResult runClosedLoopSimulation(OptiPessiInterface& interface, ocs2::IpmMpc& mpc, bool verbose,
                                         bool realTimeIteration) {
  const OptiPessiModelParameters& params = interface.modelParameters();
  auto referenceManagerPtr = interface.getOptiPessiReferenceManagerPtr();
  const int maxSteps = 1 + static_cast<int>(std::ceil(params.simTime / params.dtMin));

  matrix_t X = matrix_t::Zero(RobotX::DIM, maxSteps + 1);
  matrix_t U = matrix_t::Zero(RobotU::DIM, maxSteps);
  matrix_t hip0 = matrix_t::Zero(2, maxSteps);
  matrix_t hip1 = matrix_t::Zero(2, maxSteps);
  vector_t solveTimes = vector_t::Zero(maxSteps);
  vector_t appliedPessiScale = vector_t::Zero(maxSteps);
  int relaxedSteps = 0;
  int fallbackSteps = 0;

  // Step 0 stands on (FR, RL); seed those stance feet under their nominal hips.
  X.col(0) = params.initialState.head(RobotX::DIM);
  X.block(RobotX::P0X, 0, 2, 1) = worldHip(X.col(0), hipOf(params, Foot::FR));
  X.block(RobotX::P1X, 0, 2, 1) = worldHip(X.col(0), hipOf(params, Foot::RL));

  ObstaclePlant plant = makeObstaclePlant(params);
  std::vector<matrix_t> obstacleTrajectories(static_cast<size_t>(params.numObstacles()), matrix_t::Zero(2, maxSteps + 1));
  for (int j = 0; j < params.numObstacles(); ++j) {
    obstacleTrajectories[static_cast<size_t>(j)](0, 0) = plant.positions(j, 0);
    obstacleTrajectories[static_cast<size_t>(j)](1, 0) = plant.positions(j, 1);
  }

  scalar_t t = 0.0;
  int n = 0;
  bool collision = false;
  bool haveWarmStart = false;
  ocs2::PrimalSolution warmStart;

  // NO RTI INITIALIZATION SOLVE -- it was tried and it is worse.
  //
  // The textbook remedy for RTI's cold first step is to converge step 0 (here:
  // SqpSolver::setSqpIterationOverride(sqpIteration) for the first solve, cleared afterwards) so the
  // single-step sequence starts on the manifold. Measured on S1 it made the run WORSE, not better:
  // without it the loop ran 108 steps collision-free, with it the loop collided at step 22. A
  // converged step-0 plan is a plan for the pessimistic keep-out at T = 0; one Newton step per
  // control step cannot keep up with that disk as it inflates, so the better the start, the further
  // the iterate has fallen behind by the time the obstacle matters.

  const scalar_t goalToleranceSq = params.goalTolerance * params.goalTolerance;
  auto goalDistanceSq = [&](int step) {
    return std::pow(X(RobotX::CX, step) - params.goal(0), 2) + std::pow(X(RobotX::CY, step) - params.goal(1), 2);
  };

  while (t < params.simTime && goalDistanceSq(n) > goalToleranceSq) {
    referenceManagerPtr->setGaitOffset(n);
    referenceManagerPtr->setObstacles(plant.positions);
    referenceManagerPtr->setGoal(params.goal);

    if (verbose) {
      std::printf("Sim step: %d  time: %.2f\n", n, t);
      std::fflush(stdout);
    }

    const auto tic = std::chrono::steady_clock::now();
    scalar_t acceptedScale = 1.0;
    const SolveOutcome outcome = interface.solveControlStep(mpc, X.col(n), haveWarmStart ? &warmStart : nullptr,
                                                            realTimeIteration, verbose, acceptedScale);
    const auto toc = std::chrono::steady_clock::now();
    solveTimes(n) = std::chrono::duration<scalar_t>(toc - tic).count();
    appliedPessiScale(n) = outcome.ok ? acceptedScale : 0.0;

    const auto stancePair = gaitPair(n);
    hip0.col(n) = worldHip(X.col(n), hipOf(params, stancePair[0]));
    hip1.col(n) = worldHip(X.col(n), hipOf(params, stancePair[1]));

    if (outcome.ok && acceptedScale < 1.0) {
      ++relaxedSteps;
      std::printf("  step %d: accepted a RELAXED solve, pessiScale=%.2f -- not robust to the full v_obs bound\n", n,
                  acceptedScale);
    }

    vector_t appliedInput = outcome.appliedInput;
    vector_t successorState = outcome.successorState;

    if (!outcome.ok) {
      ++fallbackSteps;
      // Prefer the failed solve's own first input, saturated to its bounds (what the reference
      // does) -- a failed iterate is usually still informative. Fall back to the capture-point stop
      // when that input is missing or would itself leave the state out of bounds.
      bool usedSaturatedSolve = false;
      if (outcome.appliedInput.allFinite() && outcome.appliedInput.size() == RobotU::DIM &&
          outcome.appliedInput(RobotU::DT) > 0.0) {
        const vector_t candidateInput = saturateRobotInput(outcome.appliedInput, params);
        const vector_t candidateState =
            lipMapScalar(X.col(n), candidateInput, params.omega(), params.mass, params.inertia);
        if (!isInsane(candidateState, candidateInput, params)) {
          appliedInput = candidateInput;
          successorState = candidateState;
          usedSaturatedSolve = true;
        }
      }
      if (!usedSaturatedSolve) {
        appliedInput = fallbackInput(params, X.col(n), n);
        successorState = lipMapScalar(X.col(n), appliedInput, params.omega(), params.mass, params.inertia);
      }

      // Match the reference behaviour: a failed solve saturates the input to its bounds, integrates
      // the discrete dynamics, WARNS about any state-bound violation, and carries on
      // (mpc_utils.py:152-189). Only genuine numerical divergence aborts the run -- bailing out on a
      // merely out-of-bounds state would end the simulation on a single bad solve.
      if (hasDiverged(successorState, appliedInput)) {
        std::printf("\tABORTING: state diverged at step %d: c=(%.2f,%.2f) v=(%.2f,%.2f) |v|=%.2f\n", n,
                    successorState(RobotX::CX), successorState(RobotX::CY), successorState(RobotX::DCX),
                    successorState(RobotX::DCY), successorState.segment(RobotX::DCX, 2).norm());
        break;
      }
      if (isInsane(successorState, appliedInput, params)) {
        std::printf("\tCONSTRAINT VIOLATION at step %d (fallback applied): v=(%.2f,%.2f) dtheta=%.2f\n", n,
                    successorState(RobotX::DCX), successorState(RobotX::DCY), successorState(RobotX::DTH));
      }
      stepObstacles(plant, params, X.col(n).head(2), appliedInput(RobotU::DT), n);
      if (params.numObstacles() > 0) {
        vector_t obstacle(2);
        obstacle << plant.positions(0, 0), plant.positions(0, 1);
        collision = hipsAndFeetCollide(successorState.head(2), successorState(RobotX::TH), successorState.tail(4), obstacle, params,
                                       params.obstacleRadius);
      }
    } else {
      stepObstacles(plant, params, X.col(n).head(2), appliedInput(RobotU::DT), n);
    }

    X.col(n + 1) = successorState;
    U.col(n) = appliedInput;
    for (int j = 0; j < params.numObstacles(); ++j) {
      obstacleTrajectories[static_cast<size_t>(j)](0, n + 1) = plant.positions(j, 0);
      obstacleTrajectories[static_cast<size_t>(j)](1, n + 1) = plant.positions(j, 1);
    }

    // Only a feasible plan is worth carrying forward; warm-starting from a least-infeasible one
    // propagates its garbage tail into the next solve.
    // Measured, not assumed: relaxing this to `outcome.ok` (i.e. warm-starting from a merely
    // applicable plan, as the CasADi reference does unconditionally) was tried on S4 and changed
    // nothing -- the warm-started solves came back with a horizon defect of 12.77, were rejected,
    // and the run ended at the identical step with the identical collision.
    //
    // Under RTI the rule inverts: the iterate is ALWAYS shifted and reused, trustworthy or not.
    // A real-time iteration is one Newton step of a sequence that converges along the closed loop;
    // throwing the step away because it is not yet feasible restarts that sequence from scratch
    // every control step, which is the one thing the scheme cannot afford.
    const bool keepWarmStart =
        outcome.planTrustworthy() || (realTimeIteration && !outcome.solution.stateTrajectory_.empty());
    if (keepWarmStart) {
      warmStart = shiftPrimalSolution(outcome.solution, successorState);
      haveWarmStart = true;
    } else {
      haveWarmStart = false;
    }

    t += appliedInput(RobotU::DT);
    n += 1;
    if (collision || n >= maxSteps) {
      break;
    }
  }

  std::cout << "Final simulation time: " << t << "\n";
  std::cout << "Final distance from goal: " << std::sqrt(goalDistanceSq(n)) << "\n";

  ClosedLoopResult result;
  result.stateTrajectory = X.topLeftCorner(RobotX::DIM, n + 1);
  result.inputTrajectory = U.leftCols(n);
  result.stanceHip0 = hip0.leftCols(n);
  result.stanceHip1 = hip1.leftCols(n);
  result.obstacleTrajectories.resize(static_cast<size_t>(params.numObstacles()));
  for (int j = 0; j < params.numObstacles(); ++j) {
    result.obstacleTrajectories[static_cast<size_t>(j)] = obstacleTrajectories[static_cast<size_t>(j)].leftCols(n + 1);
  }
  result.solveTimes = solveTimes.head(n);
  result.appliedPessiScale = appliedPessiScale.head(n);
  result.relaxedSteps = relaxedSteps;
  result.fallbackSteps = fallbackSteps;
  result.collision = collision;
  return result;
}

}  // namespace opti_pessi
