//
// Weighted whole-body control on explicit CoM and foot references, for planners without a centroidal
// (state, input) trajectory such as the Opti-Pessi LIP MPC.
//

#pragma once

#include "legged_wbc/WbcBase.h"

namespace legged {

/** Task-space references of one control tick, in odom. Feet are indexed like CentroidalModelInfo (contactNames3DoF). */
struct WbcReference {
  contact_flag_t contact{};
  feet_array_t<vector3_t> footPosition{};
  feet_array_t<vector3_t> footVelocity{};
  feet_array_t<vector3_t> footForce{};  // desired contact force, zero while swinging
  vector3_t comPosition = vector3_t::Zero();
  vector3_t comVelocity = vector3_t::Zero();
  vector3_t comAcceleration = vector3_t::Zero();
  scalar_t yaw = 0.0;
  scalar_t yawRate = 0.0;
  scalar_t yawAcceleration = 0.0;
};

/**
 * Same QP as WeightedWbc (EoM, torque limits, friction cone, stance feet fixed as constraints), with the
 * weighted tasks written on a WbcReference instead of a centroidal state/input:
 *   - swing feet: PD on the reference position/velocity, as formulateSwingLegTask();
 *   - contact forces: the reference forces, as formulateContactForceTask();
 *   - centroidal: m ddc = A_lin qdd + dA_lin v on the MEASURED model, with ddc the reference CoM acceleration
 *     plus PD on the CoM, so the swinging legs are part of the momentum and no joint reference is needed;
 *     PD on yaw towards the reference and on roll/pitch towards zero, on the ZYX Euler rows of qdd.
 */
class OptiPessiWbc : public WbcBase {
 public:
  using WbcBase::WbcBase;

  /** Returns x = [qdd, F, tau] like WeightedWbc::update(). */
  vector_t update(const WbcReference& reference, const vector_t& rbdStateMeasured);

  /** There is no centroidal reference to track: always throws, use update(reference, rbdStateMeasured). */
  vector_t update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured, size_t mode,
                  scalar_t period) override;

  void loadTasksSetting(const std::string& taskFile, bool verbose) override;

  /** Number of QPs qpOASES did not solve since construction (diagnostics). */
  size_t getNumQpFailures() const { return numQpFailures_; }

  /** Linear rows of the centroidal task at the last solution: CoM acceleration the QP achieves minus the requested one [m/s^2]. */
  const vector3_t& getLastCentroidalResidual() const { return lastCentroidalResidual_; }

  scalar_t getFrictionCoefficient() const { return frictionCoeff_; }

 protected:
  Task formulateConstraints();
  Task formulateSwingFootTask(const WbcReference& reference);
  Task formulateReferenceForceTask(const WbcReference& reference) const;
  Task formulateCentroidalTask(const WbcReference& reference);

 private:
  scalar_t weightSwingLeg_{}, weightCentroidal_{}, weightContactForce_{};
  scalar_t comKpXY_{}, comKdXY_{}, comKpZ_{}, comKdZ_{};
  scalar_t yawKp_{}, yawKd_{}, rollPitchKp_{}, rollPitchKd_{};
  size_t numQpFailures_ = 0;
  matrix_t centroidalLinearA_;  // linear rows of the last centroidal task, for the residual
  vector3_t centroidalLinearB_ = vector3_t::Zero();
  vector3_t lastCentroidalResidual_ = vector3_t::Zero();
};

}  // namespace legged
