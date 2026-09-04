#pragma once

#include <memory>
#include <string>

#include <ocs2_core/Types.h>
#include <ocs2_core/initialization/Initializer.h>
#include <ocs2_ipm/IpmSettings.h>
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_oc/oc_problem/OptimalControlProblem.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>
#include <ocs2_robotic_tools/common/RobotInterface.h>

#include "opti_pessi_interface/OptiPessiModelParameters.h"
#include "opti_pessi_interface/OptiPessiReferenceManager.h"
#include "opti_pessi_interface/definitions.h"

namespace opti_pessi {

/**
 * Assembles the Optimistic-Pessimistic optimal control problem for OCS2, following the structure
 * of legged_interface::LeggedInterface.
 *
 * The problem is built over an augmented state that stacks two full LIP trajectories -- an
 * optimistic one that carries the cost and only avoids the currently observed obstacle disk, and a
 * pessimistic one that carries no cost but must stay feasible against the worst-case reachable
 * disk -- coupled by u_0^opti = u_0^pessi. See mpc_formulation.md and definitions.h.
 *
 * This class owns the problem, not the solver: construct an ocs2::IpmSolver from
 * ipmSettings() / getOptimalControlProblem() / getInitializer(), the same way LeggedController
 * constructs its SqpMpc from LeggedInterface.
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

  void setupOptimalControlProblem(const std::string& libraryFolder, bool recompile);

  void setupReferenceManager(const OptiPessiModelParameters& params);

  const ocs2::OptimalControlProblem& getOptimalControlProblem() const override { return *problemPtr_; }
  const ocs2::Initializer& getInitializer() const override { return *initializerPtr_; }
  std::shared_ptr<ocs2::ReferenceManagerInterface> getReferenceManagerPtr() const override { return referenceManagerPtr_; }

  /** Typed accessor, needed to push the goal / obstacles / gait parity in the closed loop. */
  std::shared_ptr<OptiPessiReferenceManager> getOptiPessiReferenceManagerPtr() const { return referenceManagerPtr_; }

  const ocs2::ipm::Settings& ipmSettings() const { return ipmSettings_; }
  const ocs2::mpc::Settings& mpcSettings() const { return mpcSettings_; }
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

 private:
  OptiPessiModelParameters params_;

  ocs2::ipm::Settings ipmSettings_;
  ocs2::mpc::Settings mpcSettings_;
  ocs2::rollout::Settings rolloutSettings_;

  std::unique_ptr<ocs2::OptimalControlProblem> problemPtr_;
  std::shared_ptr<OptiPessiReferenceManager> referenceManagerPtr_;
  std::unique_ptr<ocs2::RolloutBase> rolloutPtr_;
  std::unique_ptr<ocs2::Initializer> initializerPtr_;
};

}  // namespace opti_pessi
