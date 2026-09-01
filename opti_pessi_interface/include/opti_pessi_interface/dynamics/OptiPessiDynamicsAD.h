#pragma once

#include <string>

#include <ocs2_core/dynamics/SystemDynamicsBaseAD.h>

#include "opti_pessi_interface/OptiPessiModelParameters.h"
#include "opti_pessi_interface/definitions.h"

namespace opti_pessi {

/**
 * Augmented dynamics of the Opti-Pessi OCP: the optimistic and pessimistic branches advance
 * independently through the same exact discrete LIP map, and a scalar clock accumulates the
 * PESSIMISTIC branch's phase durations (that sum is what inflates the worst-case obstacle disk).
 *
 * See the time-convention note in definitions.h: systemFlowMap returns lipMap(x, u) - x so that
 * the EULER discretization with dt = 1.0 reproduces the discrete map exactly.
 */
class OptiPessiDynamicsAD final : public ocs2::SystemDynamicsBaseAD {
 public:
  OptiPessiDynamicsAD(OptiPessiModelParameters params, const std::string& libraryFolder, bool recompile);
  ~OptiPessiDynamicsAD() override = default;

  OptiPessiDynamicsAD* clone() const override { return new OptiPessiDynamicsAD(*this); }

 protected:
  ocs2::ad_vector_t systemFlowMap(ocs2::ad_scalar_t time, const ocs2::ad_vector_t& state, const ocs2::ad_vector_t& input,
                                  const ocs2::ad_vector_t& parameters) const override;

 private:
  OptiPessiDynamicsAD(const OptiPessiDynamicsAD& other) = default;

  OptiPessiModelParameters params_;
};

}  // namespace opti_pessi
