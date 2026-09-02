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
 * Largest violation of the APPLIED step's own path inequalities before the solve is rejected.
 *
 * Only knot 0's input is executed; violations further along the horizon are plan-quality issues that
 * the next re-solve gets to fix, and rejecting on them throws away usable solves (the reference
 * applies whatever the NLP returns). The bound is deliberately loose for the same reason.
 *
 * "The applied step" means knot 1 here -- the state u_0 lands the robot in. See the long comment at
 * the measurement site in extractSolve() for why knot 0 cannot be the gate.
 *
 * Note this is measured directly off the returned trajectory, NOT read from the solver's
 * PerformanceIndex: with computeLagrangeMultipliers = false the reported equality/inequality SSE is
 * slack-contaminated and can read 1e11 on a solution whose true worst violation is 0.03.
 */
constexpr scalar_t kAppliedViolationTolerance = 0.05;

/**
 * Largest violation anywhere along the horizon before the PLAN (as opposed to the applied step) is
 * considered untrustworthy.
 *
 * A plan can be fine at knot 0 and hopeless further out -- that is precisely what happens when the
 * pessimistic keep-out has inflated past what the robot can outrun, and the solver converges to a
 * least-infeasible point whose cost is ~1e6 instead of ~20. Such a plan must not become the next
 * warm start, and it is the signal to fall back to the keep-out continuation.
 */
constexpr scalar_t kPlanViolationTolerance = 0.05;

/**
 * Continuation schedule for the pessimistic keep-out growth (see ClosedLoopSimulation.h). Each
 * relaxed solve warm-starts the next, walking back up to the nominal pessiScale = 1.
 */
constexpr std::array<scalar_t, 5> kPessiContinuation = {0.0, 0.25, 0.5, 0.75, 1.0};

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
  scalar_t dynamicsResidual = 1e9;    // |x_1^solver - lipMap(x_0, u_0)|
  scalar_t horizonResidual = 1e9;     // worst defect over the whole horizon
  scalar_t constraintViolation = 1e9; // worst path-inequality violation at the APPLIED knot
  scalar_t horizonViolation = 1e9;    // worst path-inequality violation anywhere on the horizon
  bool ok = false;                    // the applied step is usable
  /** The whole plan is feasible, so it is safe to warm-start the next solve from it. */
  bool planTrustworthy() const { return ok && horizonViolation < kPlanViolationTolerance; }
};

/** Worst violation (as a positive number) of the problem's path inequalities at one knot. */
scalar_t appliedConstraintViolation(const ocs2::OptimalControlProblem& problem, scalar_t time, const vector_t& state,
                                    const vector_t& input) {
  if (problem.inequalityConstraintPtr == nullptr || problem.inequalityConstraintPtr->empty()) {
    return 0.0;
  }
  const ocs2::PreComputation preComputation;
  scalar_t worst = 0.0;
  for (const auto& g : problem.inequalityConstraintPtr->getValue(time, state, input, preComputation)) {
    if (g.size() > 0) {
      worst = std::max(worst, -std::min(scalar_t(0), g.minCoeff()));
    }
  }
  return worst;
}

SolveOutcome extractSolve(ocs2::IpmSolver& solver, const ocs2::OptimalControlProblem& problem,
                          const OptiPessiModelParameters& params, const vector_t& robotState, bool verbose) {
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

  // Measured at knot 1, NOT knot 0 -- knot 0 reports almost nothing.
  //
  // StageInequalityConstraint::isActive is `time >= 0.5`, and for a good reason: knot 0's path rows
  // constrain x_0, which is the measurement, so enforcing them would make the OCP infeasible exactly
  // when the controller is needed most. But the collection honours isActive, so evaluating it at
  // t = 0 leaves only the input box bounds -- no velocity bound, no reachability, no collision row.
  // It read 0.000e+00 on healthy steps and 9.26e-08 on a plan whose horizon violation was 1.4e+04,
  // which is how the first inputs of runaway plans came through this gate marked clean.
  //
  // The rows that actually guard the executed step are the ones at knot 1: they constrain
  // x_1 = lipMap(x_0, u_0), the state this step is about to enter. dynamicsResidual separately
  // verifies that the solver's x_1 agrees with that map, so knot 1 is the honest applied-step gate.
  if (out.solution.inputTrajectory_.size() > 1) {
    out.constraintViolation =
        appliedConstraintViolation(problem, 1.0, out.solution.stateTrajectory_[1], out.solution.inputTrajectory_[1]);
  } else {
    out.constraintViolation =
        appliedConstraintViolation(problem, 0.0, out.solution.stateTrajectory_.front(), out.solution.inputTrajectory_.front());
  }
  out.horizonViolation = 0.0;
  {
    const int knots = std::min(static_cast<int>(out.solution.inputTrajectory_.size()),
                               static_cast<int>(out.solution.stateTrajectory_.size()) - 1);
    for (int k = 0; k < knots; ++k) {
      out.horizonViolation = std::max(out.horizonViolation,
                                      appliedConstraintViolation(problem, static_cast<scalar_t>(k),
                                                                 out.solution.stateTrajectory_[k],
                                                                 out.solution.inputTrajectory_[k]));
    }
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
    std::printf("    cost=%.4e appliedViolation=%.3e horizonViolation=%.3e\n", solver.getPerformanceIndeces().cost,
                out.constraintViolation, out.horizonViolation);
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

  out.ok = out.dynamicsResidual < kDynamicsResidualTolerance && out.constraintViolation < kAppliedViolationTolerance;
  if (!out.ok && verbose && out.constraintViolation >= kAppliedViolationTolerance) {
    std::printf("    rejected: applied step violates its own constraints by %.3e\n", out.constraintViolation);
  }
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

  // Step 0 stands on (FR, RL); seed those stance feet under their nominal hips. With no previous
  // phase to inherit from, the "previous" footholds start equal to the current ones.
  X.col(0).head(10) = params.initialState.head(10);
  X.block(RobotX::P0X, 0, 2, 1) = worldHip(X.col(0), hipOf(params, Foot::FR));
  X.block(RobotX::P1X, 0, 2, 1) = worldHip(X.col(0), hipOf(params, Foot::RL));
  X.block(RobotX::PP0X, 0, 4, 1) = X.block(RobotX::P0X, 0, 4, 1);
  // Likewise the "previous" pose starts equal to the current one, so step 0's mid-step plane
  // degenerates to the knot pose instead of separating against a spurious midpoint at the origin.
  X(RobotX::PCX, 0) = X(RobotX::CX, 0);
  X(RobotX::PCY, 0) = X(RobotX::CY, 0);
  X(RobotX::PTH, 0) = X(RobotX::TH, 0);

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
        result = extractSolve(solver, interface.getOptimalControlProblem(), params, X.col(n), verbose);
      } catch (const std::exception& e) {
        std::cout << "Solver failed (pessiScale=" << pessiScale << "): " << e.what() << "\n";
      }
      return result;
    };

    // 1. Nominal problem, warm-started from the shifted previous solution.
    SolveOutcome outcome = attempt(1.0, haveWarmStart ? &warmStart : nullptr);
    scalar_t acceptedScale = 1.0;

    // 2. A bad warm start can poison the solve; retry cold at the nominal scale.
    if (!outcome.planTrustworthy() && haveWarmStart) {
      const SolveOutcome cold = attempt(1.0, nullptr);
      if (cold.planTrustworthy() || (!outcome.ok && cold.ok)) {
        outcome = cold;
      }
    }

    // 3. Keep-out continuation.
    //
    // Triggered on plan quality, not just on the applied step: once the worst-case disk has inflated
    // past what the robot can outrun, the nominal problem is genuinely infeasible and the solver
    // returns a least-infeasible plan that is fine at knot 0 but hopeless further out. Applying its
    // first input and warm-starting from it walks the closed loop into a state the LIP cannot
    // recover from. Solving a relaxed problem instead yields a feasible plan and a usable warm
    // start, at the cost of robustness -- which is measured and reported, never hidden.
    if (!outcome.planTrustworthy()) {
      SolveOutcome best;
      scalar_t bestScale = 0.0;
      const ocs2::PrimalSolution* guess = nullptr;
      SolveOutcome previous;
      for (const scalar_t scale : kPessiContinuation) {
        const SolveOutcome step = attempt(scale, guess);
        if (!step.planTrustworthy()) {
          break;  // the relaxed problems only get harder from here
        }
        previous = step;
        guess = &previous.solution;
        best = step;
        bestScale = scale;
      }
      // Keep the nominal result if the continuation found nothing better.
      if (best.planTrustworthy()) {
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
    if (outcome.planTrustworthy()) {
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
  // Only the 10 physical degrees of freedom are logged; the trailing previous-foothold entries are
  // bookkeeping for the constraints and would confuse the plotting/comparison scripts.
  result.stateTrajectory = X.topLeftCorner(10, n + 1);
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
