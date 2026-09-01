#pragma once

#include <string>

#include <ocs2_core/constraint/StateInputConstraintCppAd.h>

#include "opti_pessi_interface/OptiPessiModelParameters.h"
#include "opti_pessi_interface/definitions.h"

namespace opti_pessi {

/**
 * Non-anticipativity: u_0^opti = u_0^pessi.
 *
 * This single constraint is what makes the scheme safe. The applied input is optimal for the
 * nominal future AND is the first step of a plan that stays feasible against any obstacle motion
 * with speed at most v_obs -- without paying the conservatism of enforcing the growing tube along
 * the whole executed trajectory.
 *
 * Active only at knot 0. Since "time" is the knot index (see definitions.h), the gate is t < 0.5.
 */
class FirstInputConsensusConstraint final : public ocs2::StateInputConstraintCppAd {
 public:
  FirstInputConsensusConstraint(OptiPessiModelParameters params, const std::string& libraryFolder, bool recompile);

  FirstInputConsensusConstraint* clone() const override { return new FirstInputConsensusConstraint(*this); }

  bool isActive(scalar_t time) const override { return time < 0.5; }

  size_t getNumConstraints(scalar_t time) const override { return numConstraints_; }

 protected:
  ocs2::ad_vector_t constraintFunction(ocs2::ad_scalar_t time, const ocs2::ad_vector_t& state, const ocs2::ad_vector_t& input,
                                       const ocs2::ad_vector_t& parameters) const override;

 private:
  FirstInputConsensusConstraint(const FirstInputConsensusConstraint& other) = default;

  OptiPessiModelParameters params_;
  size_t numConstraints_ = 0;
};

}  // namespace opti_pessi
