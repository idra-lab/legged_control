#include "opti_pessi_interface/OptiPessiInterface.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <stdexcept>

#include <boost/filesystem.hpp>
#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/penalties/Penalties.h>
#include <ocs2_core/soft_constraint/StateInputSoftConstraint.h>
#include <ocs2_core/soft_constraint/StateSoftConstraint.h>
#include <ocs2_ipm/IpmMpc.h>
#include <ocs2_oc/oc_data/PerformanceIndex.h>
#include <ocs2_oc/rollout/RolloutSettings.h>

#include "opti_pessi_interface/constraint/FirstInputConsensusConstraint.h"
#include "opti_pessi_interface/constraint/InputBoundsConstraint.h"
#include "opti_pessi_interface/constraint/StageInequalityConstraint.h"
#include "opti_pessi_interface/constraint/TerminalCopConstraint.h"
#include "opti_pessi_interface/constraint/TerminalObstacleConstraint.h"
#include "opti_pessi_interface/LipKinematics.h"
#include "opti_pessi_interface/cost/OptiPessiCost.h"
#include "opti_pessi_interface/dynamics/OptiPessiDynamicsAD.h"
#include "opti_pessi_interface/initialization/OptiPessiInitializer.h"

namespace opti_pessi {

namespace {

/**
 * ocs2::ipm::loadSettings does not read the nested HPIPM settings, so the one value this problem
 * needs is loaded separately. Primal regularization keeps the QP subproblems solvable: the
 * hyperplane angles enter the collision rows through cos/sin of a decision variable, so the Hessian
 * is routinely indefinite.
 */
/**
 * ocs2::{ipm,sqp}::loadSettings do not read the nested HPIPM settings, so the one value this problem
 * needs is loaded separately -- and separately PER BACKEND, because the two want opposite things.
 *
 * IPM wants heavy primal regularization: the hyperplane angles enter the collision rows through
 * cos/sin of a decision variable, so its Hessian is routinely indefinite and reg_prim ~ 1e1 is what
 * lets it take a step at all.
 *
 * SQP wants almost none. Measured on S1: with the IPM's reg_prim = 1e1 every single QP came back
 * `HPIPM flag 1 -- maximum number of iterations reached` (268/268 solves, every step on the
 * fallback); at the HPIPM default 1e-6 the same run solved 1771 QPs against 45 failures. The reason
 * is structural -- SQP's QP has no inequality rows at all (see SolverBackend.h), so it is a nearly
 * unconstrained problem whose conditioning a large diagonal shift destroys rather than rescues.
 */
template <typename Settings>
void loadHpipmRegPrim(const std::string& optipessiFile, Settings& settings, const std::string& key, bool verbose) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(optipessiFile, pt);
  settings.hpipmSettings.reg_prim = 1e-6;
  ocs2::loadData::loadPtreeValue(pt, settings.hpipmSettings.reg_prim, key, verbose);
}

/** Reads the relaxed-barrier parameters used by the SQP backend's softened inequality rows. */
void loadRelaxedBarrierSettings(const std::string& optipessiFile, scalar_t& mu, scalar_t& delta, bool verbose) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(optipessiFile, pt);
  ocs2::loadData::loadPtreeValue(pt, mu, "sqp.relaxedBarrier.mu", verbose);
  ocs2::loadData::loadPtreeValue(pt, delta, "sqp.relaxedBarrier.delta", verbose);
}

std::unique_ptr<ocs2::PenaltyBase> makeRelaxedBarrier(scalar_t mu, scalar_t delta) {
  return std::make_unique<ocs2::RelaxedBarrierPenalty>(ocs2::RelaxedBarrierPenalty::Config(mu, delta));
}

/** Largest one-step dynamics residual accepted before the solve is treated as failed. */
constexpr scalar_t kDynamicsResidualTolerance = 0.02;

/**
 * Largest violation of the APPLIED step's own path inequalities before the solve is rejected.
 *
 * Only knot 0's input is executed; violations further along the horizon are plan-quality issues that
 * the next re-solve gets to fix, and rejecting on them throws away usable solves (the reference
 * applies whatever the NLP returns). The bound is deliberately loose for the same reason.
 *
 * "The applied step" means interval 0, whose rows bound x_1 = lipMap(x_0, u_0) -- the state u_0
 * lands the robot in.
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

}  // namespace

SolveOutcome evaluateSolve(const ocs2::PrimalSolution& solution, const ocs2::OptimalControlProblem& problem,
                           const OptiPessiModelParameters& params, const vector_t& robotState) {
  SolveOutcome out;
  out.solution = solution;
  if (out.solution.stateTrajectory_.size() < 2 || out.solution.inputTrajectory_.empty()) {
    return out;
  }

  // Measured on interval 0 -- the interval whose input is about to be applied.
  //
  // Since the path rows are written on the successor knot (they bound x_1 = lipMap(x_0, u_0), not
  // x_0), interval 0 is active and its rows are exactly the ones that guard the executed step. That
  // was not true of the earlier knot-local transcription, which had to skip interval 0 and gate on
  // knot 1 instead.
  out.constraintViolation =
      appliedConstraintViolation(problem, 0.0, out.solution.stateTrajectory_.front(), out.solution.inputTrajectory_.front());
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

  if (!out.appliedInput.allFinite() || !out.successorState.allFinite() || isInsane(out.successorState, out.appliedInput, params)) {
    return out;
  }

  out.ok = out.dynamicsResidual < kDynamicsResidualTolerance && out.constraintViolation < kAppliedViolationTolerance;
  return out;
}

namespace {

SolveOutcome extractSolve(ocs2::SolverBase& solver, const ocs2::OptimalControlProblem& problem,
                          const OptiPessiModelParameters& params, const vector_t& robotState, bool verbose) {
  ocs2::PrimalSolution solution;
  try {
    solution = solver.primalSolution(static_cast<scalar_t>(params.N));
  } catch (const std::exception& e) {
    std::cout << "Solver failed: " << e.what() << "\n";
    return SolveOutcome();
  }
  if (solution.stateTrajectory_.size() < 2 || solution.inputTrajectory_.empty()) {
    std::cout << "Solver failed: empty primal solution\n";
    return SolveOutcome();
  }

  const SolveOutcome out = evaluateSolve(solution, problem, params, robotState);

  if (verbose) {
    std::printf("    cost=%.4e appliedViolation=%.3e horizonViolation=%.3e\n", solver.getPerformanceIndeces().cost,
                out.constraintViolation, out.horizonViolation);
    std::printf("    dynRes=%.3e horizon=%.3e cy=%.3f vx=%.2f vy=%.2f dt=%.3f\n", out.dynamicsResidual, out.horizonResidual,
                out.successorState(RobotX::CY), out.successorState(RobotX::DCX), out.successorState(RobotX::DCY),
                out.appliedInput(RobotU::DT));
    if (!out.appliedInput.allFinite() || !out.successorState.allFinite() || isInsane(out.successorState, out.appliedInput, params)) {
      std::printf("    rejected as insane: c=(%.2f,%.2f) v=(%.2f,%.2f) dtheta=%.2f dt=%.2f\n", out.successorState(RobotX::CX),
                  out.successorState(RobotX::CY), out.successorState(RobotX::DCX), out.successorState(RobotX::DCY),
                  out.successorState(RobotX::DTH), out.appliedInput(RobotU::DT));
    } else if (!out.ok && out.constraintViolation >= kAppliedViolationTolerance) {
      std::printf("    rejected: applied step violates its own constraints by %.3e\n", out.constraintViolation);
    }
  }
  return out;
}

}  // namespace

OptiPessiInterface::OptiPessiInterface(const std::string& optipessiFile, const std::string& scenarioFile,
                                       const std::string& libraryFolder, bool recompile, bool verbose) {
  
  // check that optipessi file exists
  boost::filesystem::path optipessiFilePath(optipessiFile);
  if (boost::filesystem::exists(optipessiFilePath)) {
    std::cerr << "[OptiPessiInterface] Loading task file: " << optipessiFilePath << std::endl;
  } else {
    throw std::invalid_argument("[OptiPessiInterface] Task file not found: " + optipessiFilePath.string());
  }

  // check that scenario file exists
  boost::filesystem::path scenarioFilePath(scenarioFile);
  if (boost::filesystem::exists(scenarioFilePath)) {
    std::cerr << "[OptiPessiInterface] Loading scenario file: " << scenarioFilePath << std::endl;
  } else {
    throw std::invalid_argument("[OptiPessiInterface] Scenario file not found: " + scenarioFilePath.string());
  }
  
  params_ = loadOptiPessiModelParameters(optipessiFile, scenarioFile, verbose);

  ipmSettings_ = ocs2::ipm::loadSettings(optipessiFile, "ipm", verbose);
  loadHpipmRegPrim(optipessiFile, ipmSettings_, "hpipm.reg_prim", verbose);
  sqpSettings_ = ocs2::sqp::loadSettings(optipessiFile, "sqp", verbose);
  loadHpipmRegPrim(optipessiFile, sqpSettings_, "sqp.hpipm.reg_prim", verbose);
  loadRelaxedBarrierSettings(optipessiFile, barrierMu_, barrierDelta_, verbose);
  mpcSettings_ = ocs2::mpc::loadSettings(optipessiFile, "mpc", verbose);
  rolloutSettings_ = ocs2::rollout::loadSettings(optipessiFile, "rollout", verbose);

  // setupOptimalControlProblem(libraryFolder, recompile);
}

void OptiPessiInterface::setupOptimalControlProblem(const std::string& libraryFolder, bool recompile, SolverBackend backend) {

  backend_ = backend;
  setupReferenceManager(params_);

  problemPtr_ = std::make_unique<ocs2::OptimalControlProblem>();

  // Dynamics: two LIP branches plus the elapsed-time clock.
  problemPtr_->dynamicsPtr = std::make_unique<OptiPessiDynamicsAD>(params_, libraryFolder, recompile);

  // Cost: optimistic branch only.
  problemPtr_->costPtr->add("stageCost",
                            std::make_unique<OptiPessiStageCost>(params_, *referenceManagerPtr_, libraryFolder, recompile));
  problemPtr_->finalCostPtr->add("finalCost",
                                 std::make_unique<OptiPessiFinalCost>(params_, *referenceManagerPtr_, libraryFolder, recompile));

  // Hard inequalities, both branches. Split so the input bounds and friction cones stay active at
  // knot 0 while the path constraints (which knot 0 cannot influence) start at knot 1.
  problemPtr_->inequalityConstraintPtr->add("inputBounds",
                                            std::make_unique<InputBoundsConstraint>(params_, libraryFolder, recompile));
  problemPtr_->inequalityConstraintPtr->add(
      "pathConstraints", std::make_unique<StageInequalityConstraint>(params_, *referenceManagerPtr_, libraryFolder, recompile));

  // The hyperplane collision constraint cannot reach the final knot (no input there), so the far end
  // of the horizon gets a conservative circular keep-out instead.
  problemPtr_->finalInequalityConstraintPtr->add(
      "terminalObstacle", std::make_unique<TerminalObstacleConstraint>(params_, *referenceManagerPtr_, libraryFolder, recompile));

  // Equalities: terminal CoP equilibrium and non-anticipativity. There is deliberately no
  // ||a|| = 1 constraint -- the hyperplane normals are parameterized by angle, so it holds
  // identically (see definitions.h). Both backends carry these as hard equalities.
  problemPtr_->equalityConstraintPtr->add("terminalCop",
                                          std::make_unique<TerminalCopConstraint>(params_, libraryFolder, recompile));
  problemPtr_->equalityConstraintPtr->add("firstInputConsensus",
                                          std::make_unique<FirstInputConsensusConstraint>(params_, libraryFolder, recompile));

  // SQP has no inequality machinery: ocs2::SqpSolver assembles its QP from dynamics, cost and
  // state-input equalities only, and silently discards every inequality row it computed. So under
  // that backend the same three families are registered a SECOND time as relaxed-barrier soft
  // constraints -- that copy is what actually steers the SQP iterate. The hard copies above stay
  // registered either way: the closed loop evaluates them directly to measure the true violation of
  // the step it is about to apply, and that gate must not go blind when the backend changes.
  //
  // Cost of the duplication under SQP: one extra CppAD Jacobian evaluation per term per knot per
  // iteration (the hard rows are still linearized by the transcription before being thrown away).
  // With realTimeIteration that is one iteration per control step, which is why it is affordable.
  if (backend_ == SolverBackend::Sqp) {
    // recompile = false: these wrap the same CppAD models under the same library names, which the
    // hard copies above have just generated. Regenerating them a second time would only cost build
    // time.
    problemPtr_->softConstraintPtr->add(
        "inputBoundsSoft",
        std::make_unique<ocs2::StateInputSoftConstraint>(std::make_unique<InputBoundsConstraint>(params_, libraryFolder, false),
                                                         makeRelaxedBarrier(barrierMu_, barrierDelta_)));
    problemPtr_->softConstraintPtr->add(
        "pathConstraintsSoft",
        std::make_unique<ocs2::StateInputSoftConstraint>(
            std::make_unique<StageInequalityConstraint>(params_, *referenceManagerPtr_, libraryFolder, false),
            makeRelaxedBarrier(barrierMu_, barrierDelta_)));
    problemPtr_->finalSoftConstraintPtr->add(
        "terminalObstacleSoft",
        std::make_unique<ocs2::StateSoftConstraint>(
            std::make_unique<TerminalObstacleConstraint>(params_, *referenceManagerPtr_, libraryFolder, false),
            makeRelaxedBarrier(barrierMu_, barrierDelta_)));
  }

  // Rollout
  rolloutPtr_ = std::make_unique<ocs2::TimeTriggeredRollout>(*problemPtr_->dynamicsPtr, rolloutSettings_);

  // Initialization
  initializerPtr_ = std::make_unique<OptiPessiInitializer>(params_, *referenceManagerPtr_);
}

// Note the parameter is deliberately named `params`, not `params_`: a parameter shadowing the
// member of the same name makes it impossible to tell at a glance which one the body reads.
void OptiPessiInterface::setupReferenceManager(const OptiPessiModelParameters& params) {
  // Reference manager: goal, measured obstacles, trot parity, retry homotopy knobs.
  referenceManagerPtr_ = std::make_shared<OptiPessiReferenceManager>(params);

  // OCS2 requires a target trajectory even though the cost tracks the goal through the reference
  // manager rather than through TargetTrajectories. Provide a constant one over the whole horizon.
  vector_t xRef = vector_t::Zero(params.stateDim());
  xRef.head(2) = params.goal;
  xRef.segment(RobotX::DIM, 2) = params.goal;
  const vector_t uRef = vector_t::Zero(params.inputDim());
  referenceManagerPtr_->setTargetTrajectories(
      ocs2::TargetTrajectories({0.0, finalTime()}, {xRef, xRef}, {uRef, uRef}));
}

void OptiPessiInterface::setRobotModel(scalar_t mass, scalar_t inertia) {
  if (problemPtr_ != nullptr) {
    throw std::logic_error("[OptiPessiInterface] setRobotModel() must be called before setupOptimalControlProblem().");
  }
  params_.mass = mass;
  params_.inertia = inertia;
}

/** Guards against the solver returning a formally converged but physically nonsensical iterate. */
bool isInsane(const vector_t& robotState, const vector_t& robotInput, const OptiPessiModelParameters& params) {
  if (!robotState.allFinite() || !robotInput.allFinite()) {
    return true;
  }
  // The position guard is relative to the robot's own feet, neither the origin nor the goal (which a controller
  // receives at run time and may put any distance away): stance feet or next footholds metres from the CoM mean a CoM
  // launched away from its support.
  constexpr scalar_t kMaxFootDistance = 2.0;  // [m]; hips are ~0.3 m from the CoM
  const auto footTooFar = [&](const vector_t& vector, int index) {
    return (vector.segment(index, 2) - robotState.head(2)).norm() > kMaxFootDistance;
  };
  if (footTooFar(robotState, RobotX::P0X) || footTooFar(robotState, RobotX::P1X) || footTooFar(robotInput, RobotU::P0X) ||
      footTooFar(robotInput, RobotU::P1X) || robotState.segment(RobotX::DCX, 2).norm() > params.dcxMax + 0.2 ||
      std::abs(robotState(RobotX::DTH)) > params.dthetaMax + 0.2) {
    return true;
  }
  if (robotInput(RobotU::DT) < params.dtMin - 0.05 || robotInput(RobotU::DT) > params.dtMax + 0.05) {
    return true;
  }
  return false;
}

bool SolveOutcome::planTrustworthy() const {
  return ok && horizonViolation < kPlanViolationTolerance;
}

SolveOutcome OptiPessiInterface::solveControlStep(ocs2::IpmMpc& mpc, const vector_t& robotState,
                                                  const ocs2::PrimalSolution* warmStart, bool realTimeIteration,
                                                  bool verbose, scalar_t& acceptedScale) {
  return solveWithRetries(*mpc.getSolverPtr(), *problemPtr_, params_, *referenceManagerPtr_, robotState, warmStart, realTimeIteration,
                          verbose, acceptedScale);
}

SolveOutcome solveWithRetries(ocs2::IpmSolver& solver, const ocs2::OptimalControlProblem& problem, const OptiPessiModelParameters& params,
                              OptiPessiReferenceManager& referenceManager, const vector_t& robotState,
                              const ocs2::PrimalSolution* warmStart, bool realTimeIteration, bool verbose, scalar_t& acceptedScale) {
  const vector_t augmentedInitialState = packInitialState(robotState);
  const scalar_t finalTime = static_cast<scalar_t>(params.N);

  // One solve attempt at the given keep-out scale, optionally warm-started.
  auto attempt = [&](scalar_t pessiScale, const ocs2::PrimalSolution* guess) {
    referenceManager.setPessiScale(pessiScale);
    SolveOutcome result;
    try {
      if (guess != nullptr) {
        solver.run(0.0, augmentedInitialState, finalTime, *guess);
      } else {
        solver.reset();
        solver.run(0.0, augmentedInitialState, finalTime);
      }
      result = extractSolve(solver, problem, params, robotState, verbose);
    } catch (const std::exception& e) {
      std::cout << "Solver failed (pessiScale=" << pessiScale << "): " << e.what() << "\n";
    }
    return result;
  };

  // 1. Nominal problem, warm-started from the shifted previous solution.
  SolveOutcome outcome = attempt(1.0, warmStart);
  acceptedScale = 1.0;

  // 2. A bad warm start can poison the solve; retry cold at the nominal scale.
  //    Skipped under RTI: one solve per control step is the contract.
  if (!realTimeIteration && !outcome.planTrustworthy() && warmStart != nullptr) {
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
  //
  // Skipped under RTI for the same reason as the cold retry: it costs up to five extra solves.
  if (!realTimeIteration && !outcome.planTrustworthy()) {
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

  referenceManager.setPessiScale(1.0);
  return outcome;
}

/**
 * This is the reference's `check_bounds_and_saturate` (mpc_utils.py:15-22): when a solve fails, its first input is
 * usually still informative, so it is saturated and applied rather than discarded.
 */
vector_t saturateRobotInput(vector_t u, const OptiPessiModelParameters& params) {
  u(RobotU::ALPHA) = std::min(std::max(u(RobotU::ALPHA), params.alphaReduction), scalar_t(1) - params.alphaReduction);
  u(RobotU::BETA) = std::min(std::max(u(RobotU::BETA), scalar_t(0)), scalar_t(1));
  u(RobotU::GAMMA) = std::min(std::max(u(RobotU::GAMMA), scalar_t(0)), scalar_t(1));
  u(RobotU::DT) = std::min(std::max(u(RobotU::DT), params.dtMin), params.dtMax);
  return u;
}

/**
 * A clamped failed solve can throw the robot faster than any later solve recovers from, and it breaks up the
 * capture-point stops around it: in Gazebo, saturated steps predicting 1.2 m/s lateral against dcyMax = 0.25 started a
 * fall, and on S1 with the obstacle dead ahead they took the speed 0.26 -> 0.58 -> 0.65 -> 1.43 m/s in three steps.
 */
bool saturatedStepUsable(const vector_t& robotState, const vector_t& saturatedInput, const OptiPessiModelParameters& params) {
  using vector2_t = Eigen::Matrix<scalar_t, 2, 1>;
  constexpr scalar_t kSpeedMargin = 0.1;    // [m/s] past dcxMax / dcyMax
  constexpr scalar_t kYawRateMargin = 0.2;  // [rad/s] past dthetaMax
  const vector_t successor = lipMapScalar(robotState, saturatedInput, params.omega(), params.mass, params.inertia);
  // The OCP bounds the successor's velocity in this knot's body frame (StageInequalityConstraint).
  const vector2_t dcBody = applyR01(robotState(RobotX::TH), vector2_t(successor(RobotX::DCX), successor(RobotX::DCY)));
  return std::abs(dcBody(0)) <= params.dcxMax + kSpeedMargin && std::abs(dcBody(1)) <= params.dcyMax + kSpeedMargin &&
         std::abs(successor(RobotX::DTH)) <= params.dthetaMax + kYawRateMargin && !isInsane(successor, saturatedInput, params);
}

/**
 * Emergency step used when the solver produces nothing usable: a capture-point (deadbeat) stop.
 *
 * The LIP's divergent mode is the DCM xi = c + dc/omega, which evolves as xi(t) = z + e^{omega t} (xi_0 - z) under a
 * constant CoP z. This phase's CoP goes as close to the DCM as the current support segment allows (all that alpha can
 * express). The next footholds go under the DCM as it will be at TOUCHDOWN: the midpoint of a diagonal pair is the body
 * centre, so with alpha = 1/2 the next phase's CoP sits on the DCM, the DCM stops, and the speed decays as e^{-omega t}.
 *
 * Placing them under the phase-start DCM instead -- as this routine once did -- lands every support segment behind
 * the DCM, which has moved out by e^{omega dt} ~ 2.8 times its distance to the CoP by then. Repeated stops then speed
 * the robot up: in Gazebo 16 consecutive stops took it from 0.84 to 2.5 m/s and over.
 *
 * Limits: the touchdown point stays within kMaxCaptureStep of the touchdown CoM, so a fast state gets a long step
 * rather than a deadbeat one; and beta/gamma, closest to an even split within [0, 1], make the tangential forces cancel
 * the yaw rate over the phase.
 */
vector_t fallbackInput(const OptiPessiModelParameters& params, const vector_t& robotState, int phase) {
  using vector2_t = Eigen::Matrix<scalar_t, 2, 1>;
  constexpr scalar_t kMaxCaptureStep = 0.25;  // [m] touchdown point from the touchdown CoM
  const auto next = gaitPair(phase + 1);
  const vector2_t c(robotState(RobotX::CX), robotState(RobotX::CY));
  const vector2_t dc(robotState(RobotX::DCX), robotState(RobotX::DCY));
  const vector2_t p0(robotState(RobotX::P0X), robotState(RobotX::P0Y));
  const vector2_t p1(robotState(RobotX::P1X), robotState(RobotX::P1Y));
  const scalar_t theta = robotState(RobotX::TH);
  const scalar_t dtheta = robotState(RobotX::DTH);
  const scalar_t w = params.omega();
  const scalar_t dt = params.dtMin;  // shortest phase: re-plan as soon as possible

  // This phase's CoP: as close to the DCM as the support segment allows.
  const vector2_t dcm = c + dc / w;
  const vector2_t d = p1 - p0;
  scalar_t alpha = 0.5;
  if (d.squaredNorm() > 1e-9) {
    alpha = (dcm - p0).dot(d) / d.squaredNorm();
  }
  alpha = std::min(std::max(alpha, params.alphaReduction), scalar_t(1) - params.alphaReduction);
  const vector2_t cop = computeCop(p0, p1, alpha);

  // CoM and DCM at touchdown under that CoP; the next pair lands centred under the DCM.
  const scalar_t ch = std::cosh(w * dt);
  const scalar_t sh = std::sinh(w * dt);
  const vector2_t cTouchdown = ch * c + (sh / w) * dc + (1.0 - ch) * cop;
  const vector2_t dcmTouchdown = cop + std::exp(w * dt) * (dcm - cop);
  vector2_t captureStep = dcmTouchdown - cTouchdown;
  if (captureStep.norm() > kMaxCaptureStep) {
    captureStep *= kMaxCaptureStep / captureStep.norm();
  }
  const vector2_t target = cTouchdown + captureStep;
  const scalar_t thetaTouchdown = theta + dt * dtheta;

  vector_t u = vector_t::Zero(RobotU::DIM);
  u.segment(RobotU::P0X, 2) = target + applyR(thetaTouchdown, hipOf(params, next[0]));
  u.segment(RobotU::P1X, 2) = target + applyR(thetaTouchdown, hipOf(params, next[1]));
  u(RobotU::ALPHA) = alpha;
  u(RobotU::DT) = dt;

  // Yaw: the torque of the tangential forces (see yawTorque) is affine in the split, tau = tau0 + a beta + b gamma.
  // Take the split closest to (1/2, 1/2) whose torque stops the yaw rate over the phase, dtheta + dt tau / I = 0.
  const vector2_t force = params.mass * (w * w) * (c - cop);
  const vector2_t r0 = p0 - c;
  const vector2_t r1 = p1 - c;
  const scalar_t a = force.x() * (r1.y() - r0.y());
  const scalar_t b = force.y() * (r0.x() - r1.x());
  const scalar_t tauEvenSplit = r1.x() * force.y() - r1.y() * force.x() + 0.5 * (a + b);
  const scalar_t tauTarget = -params.inertia * dtheta / dt;
  scalar_t beta = 0.5;
  scalar_t gamma = 0.5;
  if (a * a + b * b > 1e-9) {
    const scalar_t s = (tauTarget - tauEvenSplit) / (a * a + b * b);
    beta += s * a;
    gamma += s * b;
  }
  u(RobotU::BETA) = std::min(std::max(beta, scalar_t(0)), scalar_t(1));
  u(RobotU::GAMMA) = std::min(std::max(gamma, scalar_t(0)), scalar_t(1));
  return u;
}

}  // namespace opti_pessi
