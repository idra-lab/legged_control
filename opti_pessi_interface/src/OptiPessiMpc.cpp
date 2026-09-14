#include "opti_pessi_interface/OptiPessiMpc.h"

#include "opti_pessi_interface/initialization/OptiPessiInitializer.h"

namespace opti_pessi {

OptiPessiMpc::OptiPessiMpc(ocs2::mpc::Settings mpcSettings, ocs2::ipm::Settings settings,
                           const ocs2::OptimalControlProblem& optimalControlProblem, const ocs2::Initializer& initializer,
                           std::shared_ptr<const OptiPessiReferenceManager> referenceManagerPtr)
    : MPC_BASE(std::move(mpcSettings)),
      solverPtr_(std::make_unique<ocs2::IpmSolver>(std::move(settings), optimalControlProblem, initializer)),
      referenceManagerPtr_(std::move(referenceManagerPtr)) {}

void OptiPessiMpc::reset() {
  MPC_BASE::reset();
  hasSolution_ = false;
}

void OptiPessiMpc::calculateController(scalar_t initTime, const vector_t& initState, scalar_t finalTime) {
  const int gaitOffset = referenceManagerPtr_->getGaitOffset();
  const char* warmStart = "same phase";

  if (settings().coldStart_ || !hasSolution_ || (gaitOffset != lastGaitOffset_ && gaitOffset != lastGaitOffset_ + 1)) {
    warmStart = "cold";
    solverPtr_->reset();
    solverPtr_->run(initTime, initState, finalTime);
  } else if (gaitOffset == lastGaitOffset_ + 1) {
    // Next contact phase: the previous plan starts one knot later.
    warmStart = "shifted";
    const ocs2::PrimalSolution previous = solverPtr_->primalSolution(solverPtr_->getFinalTime());
    solverPtr_->run(initTime, initState, finalTime, shiftPrimalSolution(previous, extractRobotState(initState)));
  } else {
    // Same phase again: the solver's own previous solution is the right guess.
    solverPtr_->run(initTime, initState, finalTime);
  }

  hasSolution_ = true;
  lastGaitOffset_ = gaitOffset;

  SolveStatistics statistics;
  statistics.gaitOffset = gaitOffset;
  statistics.warmStart = warmStart;
  statistics.numIterations = solverPtr_->getIterationsLog().size();
  statistics.performance = solverPtr_->getPerformanceIndeces();
  std::lock_guard<std::mutex> lock(statisticsMutex_);
  lastSolveStatistics_ = statistics;
}

OptiPessiMpc::SolveStatistics OptiPessiMpc::getLastSolveStatistics() const {
  std::lock_guard<std::mutex> lock(statisticsMutex_);
  return lastSolveStatistics_;
}

}  // namespace opti_pessi
