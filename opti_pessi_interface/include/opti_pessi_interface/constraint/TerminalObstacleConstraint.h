#pragma once

#include <string>

#include <ocs2_core/constraint/StateConstraintCppAd.h>

#include "opti_pessi_interface/OptiPessiModelParameters.h"
#include "opti_pessi_interface/OptiPessiReferenceManager.h"
#include "opti_pessi_interface/definitions.h"

namespace opti_pessi {

/**
 * Obstacle keep-out at the FINAL knot, for both branches.
 *
 * The separating-hyperplane constraint cannot reach knot N: its (phi, b) variables live in the
 * input, and there is no input at the final knot. That leaves the far end of the horizon -- where
 * the pessimistic disk has inflated the most -- completely unconstrained, which costs exactly the
 * anticipation the controller needs: without this the closed loop lets the obstacle inside 0.42 m
 * where the reference keeps 0.88 m.
 *
 * Rather than introduce a hyperplane at the terminal knot, this uses the conservative circular
 * bound: the whole robot fits inside a disk of radius rHull about the CoM, so
 *
 *     |c - o| >= dMin + rHull
 *
 * is sufficient for hull-to-disk separation. It is tighter than the hyperplane condition, which is
 * the right way round for a terminal constraint -- it costs a little optimality and buys margin.
 */
class TerminalObstacleConstraint final : public ocs2::StateConstraintCppAd {
 public:
  TerminalObstacleConstraint(OptiPessiModelParameters params, const OptiPessiReferenceManager& referenceManager,
                             const std::string& libraryFolder, bool recompile);

  TerminalObstacleConstraint* clone() const override { return new TerminalObstacleConstraint(*this); }

  size_t getNumConstraints(scalar_t time) const override { return numConstraints_; }

  ocs2::vector_t getParameters(scalar_t time, const ocs2::PreComputation& preComputation) const override;

 protected:
  ocs2::ad_vector_t constraintFunction(ocs2::ad_scalar_t time, const ocs2::ad_vector_t& state,
                                       const ocs2::ad_vector_t& parameters) const override;

 private:
  TerminalObstacleConstraint(const TerminalObstacleConstraint& other) = default;

  OptiPessiModelParameters params_;
  const OptiPessiReferenceManager* referenceManagerPtr_;
  size_t numConstraints_ = 0;
  scalar_t hullRadius_ = 0.0;
};

}  // namespace opti_pessi
