#pragma once

#include <ocs2_core/initialization/Initializer.h>
#include <ocs2_oc/oc_data/PrimalSolution.h>

#include "opti_pessi_interface/OptiPessiModelParameters.h"
#include "opti_pessi_interface/OptiPessiReferenceManager.h"
#include "opti_pessi_interface/definitions.h"

namespace opti_pessi {

/**
 * Feasible-by-construction initial guess: place each branch's next footholds under the nominal
 * hips of the next stance pair, keep the CoP centred (alpha = 1/2), take the preferred phase
 * duration, split the tangential forces evenly, and roll the exact LIP map forward. The
 * separating-hyperplane variables are seeded from the robot-to-obstacle direction.
 */
class OptiPessiInitializer final : public ocs2::Initializer {
 public:
  OptiPessiInitializer(OptiPessiModelParameters params, const OptiPessiReferenceManager& referenceManager);

  OptiPessiInitializer* clone() const override { return new OptiPessiInitializer(*this); }

  void compute(scalar_t time, const vector_t& state, scalar_t nextTime, vector_t& input, vector_t& nextState) override;

 private:
  OptiPessiInitializer(const OptiPessiInitializer& other) = default;

  OptiPessiModelParameters params_;
  const OptiPessiReferenceManager* referenceManagerPtr_;
};

// -------------------------------------------------------------------------------------------------
// Free helpers shared by the initializer, the interface and the closed-loop simulation.
// -------------------------------------------------------------------------------------------------

/** Duplicates the measured 10-dof robot state into both branches and zeroes the clock. */
vector_t packInitialState(const vector_t& robotState);

/** Optimistic branch of an augmented state -- this is the plan that is actually executed. */
vector_t extractRobotState(const vector_t& augmentedState);

/** Optimistic branch of an augmented input -- this is the applied control. */
vector_t extractRobotInput(const vector_t& augmentedInput);

/** Nominal per-branch input at a knot: footholds under the next hips, centred CoP, preferred dt. */
vector_t defaultRobotInput(const OptiPessiModelParameters& params, const vector_t& robotState, int knotParity,
                           const vector_t& initBias);

/** Seeds one branch's (a, b) hyperplane block from the current robot-to-obstacle direction. */
void seedHyperplanes(vector_t& input, int hyperplaneOffset, const OptiPessiModelParameters& params, const vector_t& robotState,
                     const OptiPessiReferenceManager& referenceManager);

/** One augmented step: advance both branches through the LIP map and tick the clock. */
vector_t augmentedLipStep(const OptiPessiModelParameters& params, const vector_t& augmentedState, const vector_t& augmentedInput);

/**
 * Shifts the previous solution one knot forward and replaces the first knot with the measurement.
 * States are SHIFTED, not re-integrated -- see the note in the implementation.
 */
ocs2::PrimalSolution shiftPrimalSolution(const ocs2::PrimalSolution& solution, const vector_t& robotState);

/** Cold-start guess: the initializer rolled out over the whole horizon. */
ocs2::PrimalSolution rolloutGuess(const OptiPessiModelParameters& params, const OptiPessiReferenceManager& referenceManager,
                                  const vector_t& robotState);

}  // namespace opti_pessi
