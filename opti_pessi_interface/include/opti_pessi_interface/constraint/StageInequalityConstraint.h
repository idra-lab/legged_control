#pragma once

#include <string>

#include <ocs2_core/constraint/StateInputConstraintCppAd.h>

#include "opti_pessi_interface/OptiPessiModelParameters.h"
#include "opti_pessi_interface/OptiPessiReferenceManager.h"
#include "opti_pessi_interface/definitions.h"

namespace opti_pessi {

/**
 * All path inequalities of the Opti-Pessi OCP, imposed on BOTH branches over intervals 0..N-1:
 *
 *   - body-frame CoM velocity box and yaw-rate bound at knot i+1                        (3)
 *   - foot reachability (|p_body - hip| <= rHip) and left/right separation,
 *     for this phase's stance feet and the commanded landing feet                       (8)
 *   - separating-hyperplane collision avoidance, two planes per obstacle               (12 per obstacle)
 *
 * The input box bounds and friction cones live in InputBoundsConstraint, which is evaluated on
 * (x_i, u_i) directly.
 *
 * Every row here is written on the SUCCESSOR knot, recomputed inside the constraint as
 * x_{i+1} = lipMap(x_i, u_i) -- the dynamics are stepped explicitly rather than carried in extra
 * state slots. See the long note at the top of the .cpp.
 *
 * The two branches differ ONLY in the keep-out radius: the optimistic branch is separated from the
 * frozen disk B(o, r_obs), the pessimistic branch from the worst-case reachable disk
 * B(o, r_obs + v_obs * sum_{k<=i} dt_k^pessi), where the sum is the clock state plus this
 * interval's own pessimistic duration.
 *
 * There is no ||a|| = 1 row: the hyperplane normals are parameterized by angle (see definitions.h).
 */
class StageInequalityConstraint final : public ocs2::StateInputConstraintCppAd {
 public:
  StageInequalityConstraint(OptiPessiModelParameters params, const OptiPessiReferenceManager& referenceManager,
                            const std::string& libraryFolder, bool recompile);

  StageInequalityConstraint* clone() const override { return new StageInequalityConstraint(*this); }

  /** Active on every interval, interval 0 included -- see the .cpp. */
  bool isActive(scalar_t time) const override;

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
