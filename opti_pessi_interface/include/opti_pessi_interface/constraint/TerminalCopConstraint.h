#pragma once

#include <string>

#include <ocs2_core/constraint/StateInputConstraintCppAd.h>

#include "opti_pessi_interface/OptiPessiModelParameters.h"
#include "opti_pessi_interface/definitions.h"

namespace opti_pessi {

/**
 * Terminal equilibrium: the final CoM must sit on the final CoP, c_N = z(p_N, alpha_{N-1}), for
 * both branches. Terminal velocity is deliberately NOT forced to zero (matching the reference,
 * where that constraint is commented out).
 *
 * Because c_N and p_N are produced by the last interval's dynamics, this is expressed as a
 * state-input constraint on the LAST interval rather than as a final constraint -- hence the
 * isActive() gate on the knot index.
 */
class TerminalCopConstraint final : public ocs2::StateInputConstraintCppAd {
 public:
  TerminalCopConstraint(OptiPessiModelParameters params, const std::string& libraryFolder, bool recompile);

  TerminalCopConstraint* clone() const override { return new TerminalCopConstraint(*this); }

  /** Active only on the last interval. Recall that "time" is the knot index (see definitions.h). */
  bool isActive(scalar_t time) const override { return time >= static_cast<scalar_t>(params_.N) - 1.5; }

  size_t getNumConstraints(scalar_t time) const override { return numConstraints_; }

 protected:
  ocs2::ad_vector_t constraintFunction(ocs2::ad_scalar_t time, const ocs2::ad_vector_t& state, const ocs2::ad_vector_t& input,
                                       const ocs2::ad_vector_t& parameters) const override;

 private:
  TerminalCopConstraint(const TerminalCopConstraint& other) = default;

  OptiPessiModelParameters params_;
  size_t numConstraints_ = 0;
};

}  // namespace opti_pessi
