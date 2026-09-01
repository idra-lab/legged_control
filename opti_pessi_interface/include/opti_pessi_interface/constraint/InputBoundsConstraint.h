#pragma once

#include <string>

#include <ocs2_core/constraint/StateInputConstraintCppAd.h>

#include "opti_pessi_interface/OptiPessiModelParameters.h"
#include "opti_pessi_interface/definitions.h"

namespace opti_pessi {

/**
 * The constraints that act on the input of the interval itself, for both branches:
 *
 *   - box bounds: eps <= alpha <= 1-eps, 0 <= beta,gamma <= 1, dtMin <= dt <= dtMax
 *   - circular friction cones with alpha-split normal loads, one per stance foot
 *
 * These stay active at knot 0, unlike the path constraints in StageInequalityConstraint: they
 * constrain the input that is about to be applied, and they are always satisfiable because
 * alpha/beta/gamma are free, so activating them at knot 0 cannot make the OCP infeasible.
 *
 * Everything here is a function of this knot's state and input only -- no composition with the
 * dynamics. See the note at the top of StageInequalityConstraint.cpp.
 */
class InputBoundsConstraint final : public ocs2::StateInputConstraintCppAd {
 public:
  InputBoundsConstraint(OptiPessiModelParameters params, const std::string& libraryFolder, bool recompile);

  InputBoundsConstraint* clone() const override { return new InputBoundsConstraint(*this); }

  size_t getNumConstraints(scalar_t time) const override { return numConstraints_; }

 protected:
  ocs2::ad_vector_t constraintFunction(ocs2::ad_scalar_t time, const ocs2::ad_vector_t& state, const ocs2::ad_vector_t& input,
                                       const ocs2::ad_vector_t& parameters) const override;

 private:
  InputBoundsConstraint(const InputBoundsConstraint& other) = default;

  OptiPessiModelParameters params_;
  size_t numConstraints_ = 0;
};

}  // namespace opti_pessi
