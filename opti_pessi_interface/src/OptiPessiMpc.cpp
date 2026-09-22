#include "opti_pessi_interface/OptiPessiMpc.h"

#include <algorithm>

#include "opti_pessi_interface/LipKinematics.h"
#include "opti_pessi_interface/OptiPessiInterface.h"
#include "opti_pessi_interface/initialization/OptiPessiInitializer.h"

namespace opti_pessi {

OptiPessiMpc::OptiPessiMpc(ocs2::mpc::Settings mpcSettings, ocs2::ipm::Settings settings,
                           const ocs2::OptimalControlProblem& optimalControlProblem, const ocs2::Initializer& initializer,
                           std::shared_ptr<OptiPessiReferenceManager> referenceManagerPtr, OptiPessiModelParameters params)
    : MPC_BASE(std::move(mpcSettings)),
      solverPtr_(std::make_unique<ocs2::IpmSolver>(std::move(settings), optimalControlProblem, initializer)),
      referenceManagerPtr_(std::move(referenceManagerPtr)),
      params_(std::move(params)),
      evaluationProblemPtr_(std::make_unique<ocs2::OptimalControlProblem>(optimalControlProblem)) {}

void OptiPessiMpc::reset() {
  MPC_BASE::reset();
  hasSolution_ = false;
  nominalPublished_ = false;
  shiftRejected_ = false;
  solverStateRejected_ = false;
}

bool OptiPessiMpc::run(scalar_t currentTime, const vector_t& currentState) {
  return MPC_BASE::run(currentTime, currentState) && nominalPublished_;
}

/**
 * One MPC step. The controller re-pushes the SAME phase until its clock advances, so this is called
 * repeatedly on one problem and the warm start is keyed on the gait offset, not on wall-clock time.
 *
 * A solve that evaluateSolve() rejects never becomes a warm start: warm-starting from it left the solver stuck on its own
 * failed output (fewer iterations every solve, same violated plan) until the robot fell. Nor is it executed: while its
 * phase has no nominal plan it is published as "failed", without inputs, and the controller stops the robot in stance
 * and restarts the MPC cold. A phase that has a nominal plan keeps it.
 */
void OptiPessiMpc::calculateController(scalar_t /*initTime*/, const vector_t& initState, scalar_t /*finalTime*/) {
  const int gaitOffset = referenceManagerPtr_->getGaitOffset();
  const vector_t robotState = extractRobotState(initState);

  // lastSolution_ was accepted for this very problem, so the phase already has a published nominal plan. The start state
  // is compared too because a restart from stance begins at phase 0 again without resetting the MPC.
  const bool samePhase = hasSolution_ && gaitOffset == lastGaitOffset_ && lastStartState_.size() == robotState.size() &&
                         (lastStartState_ - robotState).cwiseAbs().maxCoeff() < 1e-9;

  // Warm start from the last accepted solution: as-is on the same phase, shifted one knot on the next phase unless that
  // shifted guess was already rejected there. Anything else starts cold.
  const char* warmStart = "cold";
  ocs2::PrimalSolution guess;
  const ocs2::PrimalSolution* guessPtr = nullptr;
  bool shifted = false;
  if (!settings().coldStart_ && hasSolution_) {
    if (samePhase) {
      warmStart = "same phase";
      guessPtr = &lastSolution_;
    } else if (gaitOffset == lastGaitOffset_ + 1 && !shiftRejected_) {
      warmStart = "shifted";
      guess = shiftPrimalSolution(lastSolution_, robotState);
      guessPtr = &guess;
      shifted = true;
    }
  }
  // The guess sets only the primal trajectories: the solver warm-starts its slacks and duals from its own previous run.
  // After a rejected run those are cleared, so they are initialized again around the guess.
  if (guessPtr != nullptr && solverStateRejected_) {
    solverPtr_->reset();
  }

  // A single solve (realTimeIteration skips the cold retry and the keep-out continuation).
  scalar_t acceptedScale = 1.0;
  const SolveOutcome outcome = solveWithRetries(*solverPtr_, *evaluationProblemPtr_, params_, *referenceManagerPtr_, robotState, guessPtr,
                                                /*realTimeIteration=*/true, /*verbose=*/false, acceptedScale);

  const auto numKnots = static_cast<size_t>(params_.N);
  const bool complete = outcome.solution.inputTrajectory_.size() >= numKnots;
  const bool accepted = complete && outcome.ok;
  solverStateRejected_ = !accepted;
  if (accepted) {
    lastSolution_ = outcome.solution;
    lastGaitOffset_ = gaitOffset;
    lastStartState_ = robotState;
    hasSolution_ = true;
    shiftRejected_ = false;
  } else if (shifted) {
    shiftRejected_ = true;
  }

  // A rejected solve (or an incomplete one) is published as a failure, without inputs, only while its phase has no
  // nominal plan. A phase that has one keeps it.
  const bool keepNominal = !accepted && samePhase;
  Plan plan;
  if (!keepNominal) {
    plan.phase = static_cast<size_t>(std::max(gaitOffset, 0));
    plan.startState = robotState;
    if (accepted) {
      for (size_t k = 0; k < numKnots; ++k) {
        plan.inputs.push_back(extractRobotInput(outcome.solution.inputTrajectory_[k]));
      }
      plan.source = "nominal";
    } else {
      plan.failed = true;
      plan.source = "failed";
    }
    plan.trustworthy = outcome.planTrustworthy();
  }
  nominalPublished_ = accepted;

  // Statistics first: a controller that reads the plan as soon as it is published then finds the statistics of its solve.
  SolveStatistics statistics;
  statistics.gaitOffset = gaitOffset;
  statistics.warmStart = warmStart;
  statistics.numIterations = solverPtr_->getIterationsLog().size();
  statistics.performance = solverPtr_->getPerformanceIndeces();
  statistics.source = keepNominal ? "rejected" : plan.source;
  statistics.pessiScale = acceptedScale;
  statistics.trustworthy = outcome.planTrustworthy();
  statistics.dynamicsResidual = outcome.dynamicsResidual;
  statistics.appliedViolation = outcome.constraintViolation;
  statistics.horizonViolation = outcome.horizonViolation;
  {
    std::lock_guard<std::mutex> lock(statisticsMutex_);
    lastSolveStatistics_ = statistics;
  }

  if (!keepNominal) {
    std::lock_guard<std::mutex> lock(planMutex_);
    plan.sequence = planSequence_ + 1;
    latestPlan_ = std::move(plan);
    planSequence_ = latestPlan_.sequence;
  }
}

OptiPessiMpc::Plan OptiPessiMpc::getLatestPlan() const {
  std::lock_guard<std::mutex> lock(planMutex_);
  return latestPlan_;
}

OptiPessiMpc::SolveStatistics OptiPessiMpc::getLastSolveStatistics() const {
  std::lock_guard<std::mutex> lock(statisticsMutex_);
  return lastSolveStatistics_;
}

}  // namespace opti_pessi
