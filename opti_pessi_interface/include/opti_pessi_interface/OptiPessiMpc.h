#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <vector>

#include <ocs2_ipm/IpmSolver.h>
#include <ocs2_mpc/MPC_BASE.h>
#include <ocs2_oc/oc_problem/OptimalControlProblem.h>

#include "opti_pessi_interface/OptiPessiModelParameters.h"
#include "opti_pessi_interface/OptiPessiReferenceManager.h"

namespace opti_pessi {

/**
 * IpmMpc for the Opti-Pessi OCP, run through MPC_MRT_Interface.
 *
 * Every solve starts at knot 0 (see definitions.h) and makes the closed loop's recovery attempts (solveWithRetries():
 * cold retry, keep-out continuation), so the solver's own last iterate -- what MPC_MRT_Interface buffers -- is not
 * necessarily the solution that won. The plan to apply is therefore published here (getLatestPlan()): the robot inputs
 * of the accepted solution, or those of a failed solve with its first input saturated when that step stays within
 * bounds, as ClosedLoopSimulation applies it. The MRT buffer only feeds visualization.
 *
 * Warm starts follow ClosedLoopSimulation: only a trustworthy plan is carried forward. On a phase change it is shifted
 * one knot first (shiftPrimalSolution): its knot-0 footholds are the ones the current stance feet already stand on, and
 * reusing it unshifted would put both feet of each side in the same place. Anything else starts cold.
 */
class OptiPessiMpc final : public ocs2::MPC_BASE {
 public:
  OptiPessiMpc(ocs2::mpc::Settings mpcSettings, ocs2::ipm::Settings settings, const ocs2::OptimalControlProblem& optimalControlProblem,
               const ocs2::Initializer& initializer, std::shared_ptr<OptiPessiReferenceManager> referenceManagerPtr,
               OptiPessiModelParameters params);

  ~OptiPessiMpc() override = default;

  ocs2::IpmSolver* getSolverPtr() override { return solverPtr_.get(); }
  const ocs2::IpmSolver* getSolverPtr() const override { return solverPtr_.get(); }

  void reset() override;

  /** Solves, then returns whether a plan was published. */
  bool run(scalar_t currentTime, const vector_t& currentState) override;

  /** A plan for one contact phase, in robot coordinates. */
  struct Plan {
    size_t sequence = 0;           // increments with every published plan; 0 before the first
    size_t phase = 0;              // contact phase (gait offset) it was solved for
    vector_t startState;           // measured 10-dof LIP state it was solved from
    std::vector<vector_t> inputs;  // robot inputs of knots 0..N-1
    bool trustworthy = false;      // the whole horizon is feasible, so its later knots may be applied shifted
    const char* source = "none";   // "nominal", "relaxed" (pessiScale < 1) or "saturated" (failed solve, first input clamped)
  };

  /** Sequence number of the latest published plan, cheap to poll from any thread. */
  size_t getPlanSequence() const { return planSequence_; }

  /** Latest published plan, safe to read from any thread. */
  Plan getLatestPlan() const;

  /** Outcome of the latest solve, for diagnostics. */
  struct SolveStatistics {
    int gaitOffset = 0;
    const char* warmStart = "none";  // "cold", "shifted" (new phase) or "same phase"
    size_t numIterations = 0;        // of the last attempt
    ocs2::PerformanceIndex performance;  // of the last attempt
    const char* source = "none";     // Plan::source of what was published, "none" if nothing
    scalar_t pessiScale = 1.0;       // keep-out scale of the returned outcome
    bool trustworthy = false;
    scalar_t dynamicsResidual = 0.0;  // |x_1^solver - lipMap(x_0, u_0)|
    scalar_t appliedViolation = 0.0;  // worst path-inequality violation of the applied step
    scalar_t horizonViolation = 0.0;  // worst path-inequality violation over the horizon
  };

  /** Written on the MPC thread after every solve, safe to read from any thread. */
  SolveStatistics getLastSolveStatistics() const;

 protected:
  void calculateController(scalar_t initTime, const vector_t& initState, scalar_t finalTime) override;

 private:
  std::unique_ptr<ocs2::IpmSolver> solverPtr_;
  std::shared_ptr<OptiPessiReferenceManager> referenceManagerPtr_;
  OptiPessiModelParameters params_;
  std::unique_ptr<ocs2::OptimalControlProblem> evaluationProblemPtr_;  // own copy: the solver's CppAD models are not shared
  ocs2::PrimalSolution lastSolution_;  // last trustworthy solution, the next warm start
  bool hasSolution_ = false;
  bool published_ = false;
  int lastGaitOffset_ = 0;

  mutable std::mutex planMutex_;
  Plan latestPlan_;
  std::atomic<size_t> planSequence_{0};

  mutable std::mutex statisticsMutex_;
  SolveStatistics lastSolveStatistics_;
};

}  // namespace opti_pessi
