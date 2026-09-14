#pragma once

#include <memory>
#include <string>

#include <ocs2_core/Types.h>
#include <ocs2_core/initialization/Initializer.h>
#include <ocs2_ipm/IpmSettings.h>
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_oc/oc_data/PrimalSolution.h>
#include <ocs2_oc/oc_problem/OptimalControlProblem.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>
#include <ocs2_robotic_tools/common/RobotInterface.h>
#include <ocs2_sqp/SqpSettings.h>

#include "opti_pessi_interface/OptiPessiModelParameters.h"
#include "opti_pessi_interface/OptiPessiReferenceManager.h"
#include "opti_pessi_interface/SolverBackend.h"
#include "opti_pessi_interface/definitions.h"

namespace ocs2 {
class IpmMpc;
class IpmSolver;
}  // namespace ocs2

namespace opti_pessi {

/** Result of one solve, checked against the exact LIP map and the problem's own path inequalities. */
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
  bool planTrustworthy() const;
};

/** Guards against the solver returning a formally converged but physically nonsensical iterate. */
bool isInsane(const vector_t& robotState, const vector_t& robotInput, const OptiPessiModelParameters& params);

/**
 * Checks a primal solution against the exact LIP map and the problem's own path inequalities, measured directly off
 * the trajectory (the solver's PerformanceIndex SSE is slack-contaminated). `problem` must not be in use by another
 * thread: its CppAD models are not thread-safe. `robotState` is the measured 10-dof state the solve started from.
 */
SolveOutcome evaluateSolve(const ocs2::PrimalSolution& solution, const ocs2::OptimalControlProblem& problem,
                           const OptiPessiModelParameters& params, const vector_t& robotState);

/**
 * One control step's solve with the closed loop's recovery attempts, on any IpmSolver bound to `referenceManager`: the
 * nominal problem (warm-started from `warmStart`, or cold), a cold retry, and the keep-out continuation over pessiScale.
 * Leaves the reference manager at the nominal keep-out. `problem` only evaluates the solutions (see evaluateSolve()).
 *
 * @param [in] robotState: measured 10-dof robot state (NOT the augmented state).
 * @param [in] realTimeIteration: skip the cold retry and the continuation (one solve per step).
 * @param [out] acceptedScale: keep-out scale of the returned outcome (1.0 unless a relaxed solve won).
 */
SolveOutcome solveWithRetries(ocs2::IpmSolver& solver, const ocs2::OptimalControlProblem& problem, const OptiPessiModelParameters& params,
                              OptiPessiReferenceManager& referenceManager, const vector_t& robotState,
                              const ocs2::PrimalSolution* warmStart, bool realTimeIteration, bool verbose, scalar_t& acceptedScale);

/** Clamps alpha, beta, gamma and dt of a robot input to their bounds, leaving the footholds alone. */
vector_t saturateRobotInput(vector_t u, const OptiPessiModelParameters& params);

/**
 * Whether a saturated failed step may be applied from `robotState`: its successor must land near the velocity limits the
 * OCP imposes (body-frame speed and yaw rate, as the OCP bounds them) and pass isInsane(). Otherwise fallbackInput() is
 * the safer step.
 */
bool saturatedStepUsable(const vector_t& robotState, const vector_t& saturatedInput, const OptiPessiModelParameters& params);

/**
 * Capture-point stop for contact phase `phase` from `robotState`, over the shortest phase: this phase's CoP as close to
 * the DCM c + dc/omega as the support segment allows, the next footholds centred under the DCM as it will be at
 * touchdown (at most a bounded step away), and the tangential force split that cancels the yaw rate.
 */
vector_t fallbackInput(const OptiPessiModelParameters& params, const vector_t& robotState, int phase);

/**
 * Assembles the Optimistic-Pessimistic optimal control problem for OCS2, following the structure
 * of legged_interface::LeggedInterface.
 *
 * The problem is built over an augmented state that stacks two full LIP trajectories -- an
 * optimistic one that carries the cost and only avoids the currently observed obstacle disk, and a
 * pessimistic one that carries no cost but must stay feasible against the worst-case reachable
 * disk -- coupled by u_0^opti = u_0^pessi. See mpc_formulation.md and definitions.h.
 *
 * This class owns the problem, not the solver: call makeSolver(interface, backend, rti) (see
 * SolverBackend.h), or construct an ocs2::IpmSolver / ocs2::SqpSolver by hand from
 * ipmSettings() / sqpSettings() / getOptimalControlProblem() / getInitializer(), the same way
 * LeggedController constructs its SqpMpc from LeggedInterface.
 */
class OptiPessiInterface : public ocs2::RobotInterface {
 public:
  /**
   * @param taskFile      config/task.info      -- model, cost weights, limits, horizon, solver settings
   * @param scenarioFile  config/scenario_S*.info -- goal, obstacles, obstacle plant, simulation
   * @param libraryFolder folder for the generated CppAD libraries
   * @param recompile     regenerate the CppAD libraries instead of loading them from disk
   * @param verbose       echo the loaded settings
   */
  OptiPessiInterface(const std::string& taskFile, const std::string& scenarioFile, const std::string& libraryFolder,
                     bool recompile, bool verbose);

  ~OptiPessiInterface() override = default;

  /**
   * Builds the OCP. The backend is part of the problem, not just of the solver: with
   * SolverBackend::Sqp the inequality terms are ALSO registered as relaxed-barrier soft constraints,
   * because ocs2::SqpSolver drops hard inequalities before assembling its QP. See SolverBackend.h.
   */
  void setupOptimalControlProblem(const std::string& libraryFolder, bool recompile,
                                  SolverBackend backend = SolverBackend::Ipm);

  void setupReferenceManager(const OptiPessiModelParameters& params);

  /**
   * Replaces the robot mass and yaw inertia loaded from task.info, e.g. with the values of the simulated model. Both
   * are compiled into the CppAD libraries, so call it before setupOptimalControlProblem() (throws otherwise) and
   * generate the libraries into a folder that belongs to these values.
   */
  void setRobotModel(scalar_t mass, scalar_t inertia);

  const ocs2::OptimalControlProblem& getOptimalControlProblem() const override { return *problemPtr_; }
  const ocs2::Initializer& getInitializer() const override { return *initializerPtr_; }
  std::shared_ptr<ocs2::ReferenceManagerInterface> getReferenceManagerPtr() const override { return referenceManagerPtr_; }

  /** Typed accessor, needed to push the goal / obstacles / gait parity in the closed loop. */
  std::shared_ptr<OptiPessiReferenceManager> getOptiPessiReferenceManagerPtr() const { return referenceManagerPtr_; }

  const ocs2::ipm::Settings& ipmSettings() const { return ipmSettings_; }
  const ocs2::sqp::Settings& sqpSettings() const { return sqpSettings_; }
  const ocs2::mpc::Settings& mpcSettings() const { return mpcSettings_; }

  /** The backend setupOptimalControlProblem() was called with. */
  SolverBackend solverBackend() const { return backend_; }
  const ocs2::rollout::Settings& rolloutSettings() const { return rolloutSettings_; }
  const ocs2::RolloutBase& getRollout() const { return *rolloutPtr_; }

  const OptiPessiModelParameters& modelParameters() const { return params_; }

  /** Measured 10-dof robot state the simulation starts from (NOT the augmented state). */
  const vector_t& getInitialState() const { return params_.initialState; }

  int numObstacles() const { return params_.numObstacles(); }
  int stateDim() const { return params_.stateDim(); }
  int inputDim() const { return params_.inputDim(); }

  /** Horizon end, in the knot-index time convention of definitions.h. */
  scalar_t finalTime() const { return static_cast<scalar_t>(params_.N); }

  /**
   * Every solve attempt of one control step, from the robot state at that step: the nominal problem,
   * a cold retry and the keep-out continuation. Leaves the reference manager at the nominal keep-out.
   * Gait offset, obstacles and goal must already be pushed to the reference manager.
   *
   * @param [in] mpc: built from this interface's problem and bound to its reference manager.
   * @param [in] robotState: measured 10-dof robot state (NOT the augmented state).
   * @param [in] warmStart: shifted previous solution, or nullptr to cold-start the nominal solve.
   * @param [in] realTimeIteration: skip the cold retry and the continuation (one solve per step).
   * @param [out] acceptedScale: keep-out scale of the returned outcome (1.0 unless a relaxed solve won).
   */
  SolveOutcome solveControlStep(ocs2::IpmMpc& mpc, const vector_t& robotState, const ocs2::PrimalSolution* warmStart,
                                bool realTimeIteration, bool verbose, scalar_t& acceptedScale);

 private:
  OptiPessiModelParameters params_;

  ocs2::ipm::Settings ipmSettings_;
  ocs2::sqp::Settings sqpSettings_;
  ocs2::mpc::Settings mpcSettings_;
  ocs2::rollout::Settings rolloutSettings_;

  /** Relaxed-barrier parameters used only when the SQP backend softens the inequality rows. */
  scalar_t barrierMu_ = 1e-2;
  scalar_t barrierDelta_ = 1e-3;

  SolverBackend backend_ = SolverBackend::Ipm;

  std::unique_ptr<ocs2::OptimalControlProblem> problemPtr_;
  std::shared_ptr<OptiPessiReferenceManager> referenceManagerPtr_;
  std::unique_ptr<ocs2::RolloutBase> rolloutPtr_;
  std::unique_ptr<ocs2::Initializer> initializerPtr_;
};

}  // namespace opti_pessi
