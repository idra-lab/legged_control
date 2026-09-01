#include "opti_pessi_interface/OptiPessiInterface.h"

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_core/misc/LoadData.h>
#include <ocs2_oc/rollout/RolloutSettings.h>

#include "opti_pessi_interface/constraint/FirstInputConsensusConstraint.h"
#include "opti_pessi_interface/constraint/InputBoundsConstraint.h"
#include "opti_pessi_interface/constraint/StageInequalityConstraint.h"
#include "opti_pessi_interface/constraint/TerminalCopConstraint.h"
#include "opti_pessi_interface/constraint/TerminalObstacleConstraint.h"
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
void loadHpipmSettings(const std::string& taskFile, ocs2::ipm::Settings& settings, bool verbose) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  settings.hpipmSettings.reg_prim = 1e-6;
  ocs2::loadData::loadPtreeValue(pt, settings.hpipmSettings.reg_prim, "hpipm.reg_prim", verbose);
}

}  // namespace

OptiPessiInterface::OptiPessiInterface(const std::string& taskFile, const std::string& scenarioFile,
                                       const std::string& libraryFolder, bool recompile, bool verbose) {
  params_ = loadOptiPessiModelParameters(taskFile, scenarioFile, verbose);

  ipmSettings_ = ocs2::ipm::loadSettings(taskFile, "ipm", verbose);
  loadHpipmSettings(taskFile, ipmSettings_, verbose);
  mpcSettings_ = ocs2::mpc::loadSettings(taskFile, "mpc", verbose);
  rolloutSettings_ = ocs2::rollout::loadSettings(taskFile, "rollout", verbose);

  setupOptimalControlProblem(libraryFolder, recompile);
}

void OptiPessiInterface::setupOptimalControlProblem(const std::string& libraryFolder, bool recompile) {
  // Reference manager: goal, measured obstacles, trot parity, retry homotopy knobs.
  referenceManagerPtr_ = std::make_shared<OptiPessiReferenceManager>(params_);

  // OCS2 requires a target trajectory even though the cost tracks the goal through the reference
  // manager rather than through TargetTrajectories. Provide a constant one over the whole horizon.
  vector_t xRef = vector_t::Zero(params_.stateDim());
  xRef.head(2) = params_.goal;
  xRef.segment(RobotX::DIM, 2) = params_.goal;
  const vector_t uRef = vector_t::Zero(params_.inputDim());
  referenceManagerPtr_->setTargetTrajectories(
      ocs2::TargetTrajectories({0.0, finalTime()}, {xRef, xRef}, {uRef, uRef}));

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
  // identically (see definitions.h).
  problemPtr_->equalityConstraintPtr->add("terminalCop",
                                          std::make_unique<TerminalCopConstraint>(params_, libraryFolder, recompile));
  problemPtr_->equalityConstraintPtr->add("firstInputConsensus",
                                          std::make_unique<FirstInputConsensusConstraint>(params_, libraryFolder, recompile));

  // Rollout
  rolloutPtr_ = std::make_unique<ocs2::TimeTriggeredRollout>(*problemPtr_->dynamicsPtr, rolloutSettings_);

  // Initialization
  initializerPtr_ = std::make_unique<OptiPessiInitializer>(params_, *referenceManagerPtr_);
}

}  // namespace opti_pessi
