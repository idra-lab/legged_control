#pragma once

#include <string>

#include <ocs2_core/constraint/StateInputConstraintCppAd.h>

#include "opti_pessi_interface/OptiPessiModelParameters.h"
#include "opti_pessi_interface/OptiPessiReferenceManager.h"
#include "opti_pessi_interface/definitions.h"

namespace opti_pessi {

/**
 * All path inequalities of the Opti-Pessi OCP, imposed on BOTH branches:
 *
 *   - input bounds: eps <= alpha <= 1-eps, 0 <= beta,gamma <= 1, dtMin <= dt <= dtMax   (8)
 *   - circular friction cones with alpha-split normal loads, one per stance foot        (2)
 *   - body-frame CoM velocity box and yaw-rate bound, evaluated at knot i+1             (3)
 *   - foot reachability (|p_body - hip| <= rHip) and left/right separation (sign of
 *     p_body_y), for both the current feet and the next footholds w.r.t. the next base  (8)
 *   - separating-hyperplane collision avoidance, twice per obstacle per interval
 *     (mid-step with hips only, landing with hips and next feet)                        (12 per obstacle)
 *
 * The two branches differ ONLY in the keep-out radius: the optimistic branch is separated from the
 * frozen disk B(o, r_obs), the pessimistic branch from the worst-case reachable disk
 * B(o, r_obs + v_obs * sum_{k<=i} dt_k^pessi), where the sum is read off the clock state.
 *
 * The unit-norm condition ||a|| = 1 on each hyperplane normal is an equality and lives in
 * HyperplaneNormalConstraint.
 */
class StageInequalityConstraint final : public ocs2::StateInputConstraintCppAd {
 public:
  StageInequalityConstraint(OptiPessiModelParameters params, const OptiPessiReferenceManager& referenceManager,
                            const std::string& libraryFolder, bool recompile);

  StageInequalityConstraint* clone() const override { return new StageInequalityConstraint(*this); }

  size_t getNumConstraints(scalar_t time) const override { return numConstraints_; }

  ocs2::vector_t getParameters(scalar_t time, const ocs2::PreComputation& preComputation) const override;

 protected:
  ocs2::ad_vector_t constraintFunction(ocs2::ad_scalar_t time, const ocs2::ad_vector_t& state, const ocs2::ad_vector_t& input,
                                       const ocs2::ad_vector_t& parameters) const override;

 private:
  StageInequalityConstraint(const StageInequalityConstraint& other) = default;

  OptiPessiModelParameters params_;
  const OptiPessiReferenceManager* referenceManagerPtr_;
  size_t numConstraints_ = 0;
};

}  // namespace opti_pessi
