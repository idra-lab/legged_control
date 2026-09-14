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

void OptiPessiMpc::calculateController(scalar_t /*initTime*/, const vector_t& initState, scalar_t /*finalTime*/) {
  const int gaitOffset = referenceManagerPtr_->getGaitOffset();
  const vector_t robotState = extractRobotState(initState);

  // Warm start from the last trustworthy plan: shifted one knot on the next phase, as-is on the same phase.
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

  scalar_t acceptedScale = 1.0;
  const SolveOutcome outcome = solveWithRetries(*solverPtr_, *evaluationProblemPtr_, params_, *referenceManagerPtr_, robotState, guessPtr,
                                                /*realTimeIteration=*/false, /*verbose=*/false, acceptedScale);

  // Only a feasible plan is worth carrying forward (see ClosedLoopSimulation).
  hasSolution_ = outcome.planTrustworthy();
  if (hasSolution_) {
    lastSolution_ = outcome.solution;
  }
  lastGaitOffset_ = gaitOffset;

  // What the controller may apply: the accepted solution, or a failed one whose saturated first step stays in bounds.
  Plan plan;
  const auto numKnots = static_cast<size_t>(params_.N);
  if (outcome.solution.inputTrajectory_.size() >= numKnots) {
    plan.phase = static_cast<size_t>(std::max(gaitOffset, 0));
    plan.startState = robotState;
    for (size_t k = 0; k < numKnots; ++k) {
      plan.inputs.push_back(extractRobotInput(outcome.solution.inputTrajectory_[k]));
    }
    if (outcome.ok) {
      plan.source = acceptedScale < 1.0 ? "relaxed" : "nominal";
      plan.trustworthy = outcome.planTrustworthy();
    } else if (outcome.appliedInput.allFinite() && outcome.appliedInput(RobotU::DT) > 0.0) {
      const vector_t candidate = saturateRobotInput(outcome.appliedInput, params_);
      if (saturatedStepUsable(robotState, candidate, params_)) {
        plan.inputs.front() = candidate;
        plan.source = "saturated";
      }
    }
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
