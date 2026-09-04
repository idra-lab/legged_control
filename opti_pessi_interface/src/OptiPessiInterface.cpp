#include "opti_pessi_interface/OptiPessiInterface.h"

#include <iostream>
#include <stdexcept>

#include <boost/filesystem.hpp>
#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/penalties/Penalties.h>
#include <ocs2_core/soft_constraint/StateInputSoftConstraint.h>
#include <ocs2_core/soft_constraint/StateSoftConstraint.h>
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

}  // namespace opti_pessi
