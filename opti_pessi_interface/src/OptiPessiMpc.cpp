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
  published_ = false;
}

bool OptiPessiMpc::run(scalar_t currentTime, const vector_t& currentState) {
  return MPC_BASE::run(currentTime, currentState) && published_;
}

/**
 * One MPC step. The controller re-pushes the SAME phase until its clock advances, so this is called
 * repeatedly on one problem and the warm start is keyed on the gait offset, not on wall-clock time.
 *
 * No failure handling: every solve that returns a full input trajectory is published and becomes
 * the next warm start, whether or not evaluateSolve() accepts it.
 */
void OptiPessiMpc::calculateController(scalar_t /*initTime*/, const vector_t& initState, scalar_t /*finalTime*/) {
  const int gaitOffset = referenceManagerPtr_->getGaitOffset();
  const vector_t robotState = extractRobotState(initState);

  // Warm start from the last solution: shifted one knot on the next phase, as-is on the same phase.
  const char* warmStart = "cold";
  ocs2::PrimalSolution guess;
  const ocs2::PrimalSolution* guessPtr = nullptr;
  if (!settings().coldStart_ && hasSolution_) {
    if (gaitOffset == lastGaitOffset_ + 1) {
      warmStart = "shifted";
      guess = shiftPrimalSolution(lastSolution_, robotState);
      guessPtr = &guess;
    } else if (gaitOffset == lastGaitOffset_) {
      warmStart = "same phase";
      guessPtr = &lastSolution_;
    }
  }

  // A single solve (realTimeIteration skips the cold retry and the keep-out continuation), used as it comes.
  scalar_t acceptedScale = 1.0;
  const SolveOutcome outcome = solveWithRetries(*solverPtr_, *evaluationProblemPtr_, params_, *referenceManagerPtr_, robotState, guessPtr,
                                                /*realTimeIteration=*/true, /*verbose=*/false, acceptedScale);

  const auto numKnots = static_cast<size_t>(params_.N);
  hasSolution_ = outcome.solution.inputTrajectory_.size() >= numKnots;
  if (hasSolution_) {
    lastSolution_ = outcome.solution;
  }
  lastGaitOffset_ = gaitOffset;

  // The solver's solution is published whatever its outcome; "unchecked" marks one that did not pass evaluateSolve().
  Plan plan;
  if (hasSolution_) {
    plan.phase = static_cast<size_t>(std::max(gaitOffset, 0));
    plan.startState = robotState;
    for (size_t k = 0; k < numKnots; ++k) {
      plan.inputs.push_back(extractRobotInput(outcome.solution.inputTrajectory_[k]));
    }
    plan.source = outcome.ok ? "nominal" : "unchecked";
    plan.trustworthy = outcome.planTrustworthy();
  }
  const char* const source = plan.source;
  published_ = plan.source != Plan().source;
  if (published_) {
    std::lock_guard<std::mutex> lock(planMutex_);
    plan.sequence = planSequence_ + 1;
    latestPlan_ = std::move(plan);
    planSequence_ = latestPlan_.sequence;
  }

  SolveStatistics statistics;
  statistics.gaitOffset = gaitOffset;
  statistics.warmStart = warmStart;
  statistics.numIterations = solverPtr_->getIterationsLog().size();
  statistics.performance = solverPtr_->getPerformanceIndeces();
  statistics.source = source;
  statistics.pessiScale = acceptedScale;
  statistics.trustworthy = outcome.planTrustworthy();
  statistics.dynamicsResidual = outcome.dynamicsResidual;
  statistics.appliedViolation = outcome.constraintViolation;
  statistics.horizonViolation = outcome.horizonViolation;
  std::lock_guard<std::mutex> lock(statisticsMutex_);
  lastSolveStatistics_ = statistics;
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
