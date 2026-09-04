#pragma once

#include <string>

#include <ocs2_core/cost/StateCostCppAd.h>
#include <ocs2_core/cost/StateInputCostCppAd.h>

#include "opti_pessi_interface/OptiPessiModelParameters.h"
#include "opti_pessi_interface/OptiPessiReferenceManager.h"
#include "opti_pessi_interface/definitions.h"

namespace opti_pessi {

/**
 * Stage cost, evaluated on the OPTIMISTIC branch only. The pessimistic branch contributes nothing
 * to the objective -- it is a pure feasibility certificate. That asymmetry is the whole point of
 * the formulation, so do not "symmetrize" this.
 *
 * Terms: CoM position error to the goal, CoM speed, yaw rate, heading alignment (weight 0),
 * CoP centering (alpha - 1/2)^2, phase-duration preference (dt - dt*)^2, and the distance from the
 * landing footholds to their nominal hip positions.
 *
 * The first four terms are evaluated at knot i. The foothold term is evaluated at knot i+1,
 * recomputed by stepping the dynamics inside the cost, because the hips it measures against move
 * with the base -- this is the reference's ocp_quadruped.py:64-66.
 */
class OptiPessiStageCost final : public ocs2::StateInputCostCppAd {
 public:
  OptiPessiStageCost(OptiPessiModelParameters params, const OptiPessiReferenceManager& referenceManager,
                     const std::string& libraryFolder, bool recompile);

  OptiPessiStageCost* clone() const override { return new OptiPessiStageCost(*this); }

  ocs2::vector_t getParameters(scalar_t time, const ocs2::TargetTrajectories& targetTrajectories,
                               const ocs2::PreComputation& preComputation) const override;

  ocs2::ad_scalar_t costFunction(ocs2::ad_scalar_t time, const ocs2::ad_vector_t& state, const ocs2::ad_vector_t& input,
                                 const ocs2::ad_vector_t& parameters) const override;

 private:
  OptiPessiStageCost(const OptiPessiStageCost& other) = default;

  OptiPessiModelParameters params_;
  const OptiPessiReferenceManager* referenceManagerPtr_;
};

/**
 * Terminal cost: the state-dependent part of the running cost at knot N, optimistic branch. No
 * foothold, CoP or duration term -- knot N has no input, and the reference adds those only for
 * i < N.
 */
class OptiPessiFinalCost final : public ocs2::StateCostCppAd {
 public:
  OptiPessiFinalCost(OptiPessiModelParameters params, const OptiPessiReferenceManager& referenceManager,
                     const std::string& libraryFolder, bool recompile);

  OptiPessiFinalCost* clone() const override { return new OptiPessiFinalCost(*this); }

  ocs2::vector_t getParameters(scalar_t time, const ocs2::TargetTrajectories& targetTrajectories,
                               const ocs2::PreComputation& preComputation) const override;

  ocs2::ad_scalar_t costFunction(ocs2::ad_scalar_t time, const ocs2::ad_vector_t& state,
                                 const ocs2::ad_vector_t& parameters) const override;

 private:
  OptiPessiFinalCost(const OptiPessiFinalCost& other) = default;

  OptiPessiModelParameters params_;
  const OptiPessiReferenceManager* referenceManagerPtr_;
};

}  // namespace opti_pessi
