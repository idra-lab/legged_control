#pragma once

#include <memory>
#include <mutex>

#include <ocs2_ipm/IpmSolver.h>
#include <ocs2_mpc/MPC_BASE.h>

#include "opti_pessi_interface/OptiPessiReferenceManager.h"

namespace opti_pessi {

/**
 * IpmMpc for the Opti-Pessi OCP, for use through MPC_MRT_Interface.
 *
 * Every solve starts at knot 0 (see definitions.h), so the solver's own warm start -- its previous solution
 * at the same knots -- only fits while the gait offset stays the same. Once the offset advances to the next
 * contact phase, that guess is one phase stale: its knot-0 footholds are the ones the current stance feet
 * already stand on, and the solver keeps them, putting both feet of each side in the same place. On a phase
 * change the previous solution is therefore shifted one knot first (shiftPrimalSolution), as
 * ClosedLoopSimulation does. A jump of more than one phase starts cold.
 */
class OptiPessiMpc final : public ocs2::MPC_BASE {
 public:
  OptiPessiMpc(ocs2::mpc::Settings mpcSettings, ocs2::ipm::Settings settings, const ocs2::OptimalControlProblem& optimalControlProblem,
               const ocs2::Initializer& initializer, std::shared_ptr<const OptiPessiReferenceManager> referenceManagerPtr);

  ~OptiPessiMpc() override = default;

  ocs2::IpmSolver* getSolverPtr() override { return solverPtr_.get(); }
  const ocs2::IpmSolver* getSolverPtr() const override { return solverPtr_.get(); }

  void reset() override;

  /** Outcome of the latest solve, for diagnostics. */
  struct SolveStatistics {
    int gaitOffset = 0;
    const char* warmStart = "none";  // "cold", "shifted" (new phase) or "same phase"
    size_t numIterations = 0;
    ocs2::PerformanceIndex performance;
  };

  /** Written on the MPC thread after every solve, safe to read from any thread. */
  SolveStatistics getLastSolveStatistics() const;

 protected:
  void calculateController(scalar_t initTime, const vector_t& initState, scalar_t finalTime) override;

 private:
  std::unique_ptr<ocs2::IpmSolver> solverPtr_;
  std::shared_ptr<const OptiPessiReferenceManager> referenceManagerPtr_;
  bool hasSolution_ = false;
  int lastGaitOffset_ = 0;

  mutable std::mutex statisticsMutex_;
  SolveStatistics lastSolveStatistics_;
};

}  // namespace opti_pessi
