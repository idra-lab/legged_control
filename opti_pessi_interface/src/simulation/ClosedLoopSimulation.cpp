#include "opti_pessi_interface/simulation/ClosedLoopSimulation.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <iostream>

#include <ocs2_oc/oc_data/PerformanceIndex.h>

#include "opti_pessi_interface/LipKinematics.h"
#include "opti_pessi_interface/initialization/OptiPessiInitializer.h"
#include "opti_pessi_interface/simulation/ObstaclePlant.h"

namespace opti_pessi {

namespace {

/** Largest one-step dynamics residual accepted before the solve is treated as failed. */
constexpr scalar_t kDynamicsResidualTolerance = 0.02;

/**
 * Continuation schedule for the pessimistic keep-out growth (see ClosedLoopSimulation.h). Each
 * relaxed solve warm-starts the next, walking back up to the nominal pessiScale = 1.
 */
constexpr std::array<scalar_t, 5> kPessiContinuation = {0.0, 0.25, 0.5, 0.75, 1.0};

vector_t worldHip(const vector_t& robotState, const vector_t& hipBody) {
  return robotState.head(2) + applyR(robotState(RobotX::TH), hipBody);
}

/**
 * Saturated open-loop input used when the solver fails: shortest allowed phase, footholds under the
 * next hips, and the CoP placed to decelerate the CoM. Keeps the simulation advancing rather than
 * aborting, mirroring the Python fallback path.
 */
vector_t fallbackInput(const OptiPessiModelParameters& params, const vector_t& robotState, int step) {
  vector_t u = vector_t::Zero(RobotU::DIM);
  const auto next = gaitPair(step + 1);
  const vector_t c = robotState.head(2);
  const vector_t dc = robotState.segment(RobotX::DCX, 2);
  const scalar_t theta = robotState(RobotX::TH);
  const scalar_t w = params.omega();
  const scalar_t dt = params.dtMin;

  const vector_t cNextApprox = c + (std::sinh(w * dt) / w) * dc;
  u.segment(RobotU::P0X, 2) = cNextApprox + applyR(theta, hipOf(params, next[0]));
  u.segment(RobotU::P1X, 2) = cNextApprox + applyR(theta, hipOf(params, next[1]));

  // Project the braking CoP target onto the current support segment to get alpha.
  const vector_t p0 = robotState.segment(RobotX::P0X, 2);
  const vector_t p1 = robotState.segment(RobotX::P1X, 2);
  const vector_t copTarget = c - scalar_t(0.25) * dc;
  const vector_t d = p1 - p0;
  const scalar_t denominator = d.dot(d);
  scalar_t alpha = 0.5;
  if (denominator > 1e-9) {
    alpha = (copTarget - p0).dot(d) / denominator;
  }
  u(RobotU::ALPHA) = std::min(std::max(alpha, params.alphaReduction), scalar_t(1) - params.alphaReduction);
  u(RobotU::DT) = dt;
  u(RobotU::BETA) = 0.5;
  u(RobotU::GAMMA) = 0.5;
  return u;
}

/** Guards against the solver returning a formally converged but physically nonsensical iterate. */
bool isInsane(const vector_t& robotState, const vector_t& robotInput, const OptiPessiModelParameters& params) {
  if (!robotState.allFinite() || !robotInput.allFinite()) {
    return true;
  }
  if (robotState.head(2).norm() > 10.0 || robotState.segment(RobotX::DCX, 2).norm() > params.dcxMax + 0.2 ||
      std::abs(robotState(RobotX::DTH)) > params.dthetaMax + 0.2) {
    return true;
  }
  if (robotInput(RobotU::DT) < params.dtMin - 0.05 || robotInput(RobotU::DT) > params.dtMax + 0.05) {
    return true;
  }
  return false;
}

struct SolveOutcome {
  ocs2::PrimalSolution solution;
  vector_t appliedInput = vector_t::Zero(RobotU::DIM);
  vector_t successorState = vector_t::Zero(RobotX::DIM);
  scalar_t dynamicsResidual = 1e9;   // |x_1^solver - lipMap(x_0, u_0)|
  scalar_t horizonResidual = 1e9;    // worst defect over the whole horizon
  bool ok = false;
};

SolveOutcome extractSolve(ocs2::IpmSolver& solver, const OptiPessiModelParameters& params, const vector_t& robotState,
                          bool verbose) {
  SolveOutcome out;
  const scalar_t finalTime = static_cast<scalar_t>(params.N);

  try {
    out.solution = solver.primalSolution(finalTime);
  } catch (const std::exception& e) {
    std::cout << "Solver failed: " << e.what() << "\n";
    return out;
  }
  if (out.solution.stateTrajectory_.size() < 2 || out.solution.inputTrajectory_.empty()) {
    std::cout << "Solver failed: empty primal solution\n";
    return out;
  }

  out.appliedInput = extractRobotInput(out.solution.inputTrajectory_.front());
  out.successorState = lipMapScalar(robotState, out.appliedInput, params.omega(), params.mass, params.inertia);

  const vector_t solverSuccessor = extractRobotState(out.solution.stateTrajectory_[1]);
  out.dynamicsResidual = (solverSuccessor - out.successorState).norm();

  out.horizonResidual = 0.0;
  const int stateDim = params.stateDim();
  const int inputDim = params.inputDim();
  const int numInputs = static_cast<int>(out.solution.inputTrajectory_.size());
  const int numIntervals = std::min(numInputs, static_cast<int>(out.solution.stateTrajectory_.size()) - 1);
  for (int k = 0; k < numIntervals; ++k) {
    const vector_t& xk = out.solution.stateTrajectory_[static_cast<size_t>(k)];
    const vector_t& uk = out.solution.inputTrajectory_[static_cast<size_t>(k)];
    const vector_t& xk1 = out.solution.stateTrajectory_[static_cast<size_t>(k + 1)];
    if (xk.size() != stateDim || uk.size() != inputDim || xk1.size() != stateDim) {
      out.horizonResidual = 1e9;
      break;
    }
    out.horizonResidual = std::max(out.horizonResidual, (augmentedLipStep(params, xk, uk) - xk1).norm());
  }

  if (verbose) {
    const auto& perf = solver.getPerformanceIndeces();
    std::printf("    cost=%.4e dynSSE=%.3e eqSSE=%.3e ineqSSE=%.3e\n", perf.cost, perf.dynamicsViolationSSE,
                perf.equalityConstraintsSSE, perf.inequalityConstraintsSSE);
    std::printf("    dynRes=%.3e horizon=%.3e cy=%.3f vx=%.2f vy=%.2f dt=%.3f\n", out.dynamicsResidual, out.horizonResidual,
                out.successorState(RobotX::CY), out.successorState(RobotX::DCX), out.successorState(RobotX::DCY),
                out.appliedInput(RobotU::DT));
  }

  if (!out.appliedInput.allFinite() || !out.successorState.allFinite() || isInsane(out.successorState, out.appliedInput, params)) {
    if (verbose) {
      std::printf("    rejected as insane: c=(%.2f,%.2f) v=(%.2f,%.2f) dtheta=%.2f dt=%.2f\n", out.successorState(RobotX::CX),
                  out.successorState(RobotX::CY), out.successorState(RobotX::DCX), out.successorState(RobotX::DCY),
                  out.successorState(RobotX::DTH), out.appliedInput(RobotU::DT));
    }
    return out;
  }

  out.ok = out.dynamicsResidual < kDynamicsResidualTolerance;
  return out;
}

}  // namespace

ClosedLoopResult runClosedLoopSimulation(OptiPessiInterface& interface, ocs2::IpmSolver& solver, bool verbose) {
  const OptiPessiModelParameters& params = interface.modelParameters();
  auto referenceManagerPtr = interface.getOptiPessiReferenceManagerPtr();
  const scalar_t finalTime = interface.finalTime();
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
  X.col(0) = params.initialState;
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

  const scalar_t goalToleranceSq = params.goalTolerance * params.goalTolerance;
  auto goalDistanceSq = [&](int step) {
    return std::pow(X(RobotX::CX, step) - params.goal(0), 2) + std::pow(X(RobotX::CY, step) - params.goal(1), 2);
  };

  while (t < params.simTime && goalDistanceSq(n) > goalToleranceSq) {
    referenceManagerPtr->setGaitOffset(n);
    referenceManagerPtr->setObstacles(plant.positions);
    referenceManagerPtr->setGoal(params.goal);

    const vector_t augmentedInitialState = packInitialState(X.col(n));
    if (verbose) {
      std::printf("Sim step: %d  time: %.2f\n", n, t);
      std::fflush(stdout);
    }

    const auto tic = std::chrono::steady_clock::now();

    // One solve attempt at the given keep-out scale, optionally warm-started.
    auto attempt = [&](scalar_t pessiScale, const ocs2::PrimalSolution* guess) {
      referenceManagerPtr->setPessiScale(pessiScale);
      SolveOutcome result;
      try {
        if (guess != nullptr) {
          solver.run(0.0, augmentedInitialState, finalTime, *guess);
        } else {
          solver.reset();
          solver.run(0.0, augmentedInitialState, finalTime);
        }
        result = extractSolve(solver, params, X.col(n), verbose);
      } catch (const std::exception& e) {
        std::cout << "Solver failed (pessiScale=" << pessiScale << "): " << e.what() << "\n";
      }
      return result;
    };

    // 1. Nominal problem, warm-started from the shifted previous solution.
    SolveOutcome outcome = attempt(1.0, haveWarmStart ? &warmStart : nullptr);
    scalar_t acceptedScale = 1.0;

    // 2. A bad warm start can poison the solve; retry cold at the nominal scale.
    if (!outcome.ok && haveWarmStart) {
      outcome = attempt(1.0, nullptr);
    }

    // 3. Continuation: solve an easy relaxed problem and walk the keep-out growth back up to 1.
    if (!outcome.ok) {
      SolveOutcome best;
      scalar_t bestScale = 0.0;
      const ocs2::PrimalSolution* guess = nullptr;
      SolveOutcome previous;
      for (const scalar_t scale : kPessiContinuation) {
        const SolveOutcome step = attempt(scale, guess);
        if (!step.ok) {
          break;
        }
        previous = step;
        guess = &previous.solution;
        best = step;
        bestScale = scale;
      }
      if (best.ok) {
        outcome = best;
        acceptedScale = bestScale;
      }
    }

    referenceManagerPtr->setPessiScale(1.0);
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
      appliedInput = fallbackInput(params, X.col(n), n);
      successorState = lipMapScalar(X.col(n), appliedInput, params.omega(), params.mass, params.inertia);
      if (isInsane(successorState, appliedInput, params)) {
        break;
      }
      if (successorState(RobotX::DTH) * successorState(RobotX::DTH) > params.dthetaMax * params.dthetaMax + 1e-5) {
        std::cout << "\tCONSTRAINT VIOLATION: dtheta out of bounds " << successorState(RobotX::DTH) << "\n";
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

    if (outcome.ok) {
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
  result.stateTrajectory = X.leftCols(n + 1);
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
