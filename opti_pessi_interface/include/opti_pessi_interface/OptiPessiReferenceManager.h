#pragma once

#include <ocs2_oc/synchronized_module/ReferenceManager.h>

#include "opti_pessi_interface/OptiPessiModelParameters.h"
#include "opti_pessi_interface/definitions.h"

namespace opti_pessi {

/**
 * Carries the per-iteration, non-differentiated data of the Opti-Pessi OCP: the goal, the measured
 * obstacle centres, the trot parity at the start of the horizon, and two homotopy knobs used by
 * the closed loop when a solve fails.
 *
 * Cost and constraint terms hold a const reference to this object and read it inside their
 * getParameters(...) hooks, so a change here is picked up by the next solve without rebuilding any
 * CppAD library. This mirrors how legged_interface's constraints consume
 * SwitchedModelReferenceManager.
 */
class OptiPessiReferenceManager : public ocs2::ReferenceManager {
 public:
  explicit OptiPessiReferenceManager(const OptiPessiModelParameters& params)
      : ocs2::ReferenceManager(ocs2::TargetTrajectories(), ocs2::ModeSchedule()),
        goal_(params.goal),
        obstacles_(params.obstaclePositions),
        initBias_(vector_t::Zero(2)) {}

  ~OptiPessiReferenceManager() override = default;

  /** Goal CoM position in the world frame. */
  const vector_t& getGoal() const { return goal_; }
  void setGoal(const vector_t& goal) { goal_ = goal; }

  /** Measured obstacle centres, numObstacles x 2. Held fixed inside one OCP solve. */
  const matrix_t& getObstacles() const { return obstacles_; }
  void setObstacles(const matrix_t& obstacles) { obstacles_ = obstacles; }

  /**
   * Trot parity at knot 0 of the horizon. Knot i stands on gaitPair(gaitOffset + i), so the horizon
   * alternates (FR, RL) / (FL, RR) starting from the parity of the current simulation step.
   */
  int getGaitOffset() const { return gaitOffset_; }
  void setGaitOffset(int gaitOffset) { gaitOffset_ = gaitOffset; }

  /** World-frame offset added to the initializer's next footholds (retry homotopy). */
  const vector_t& getInitBias() const { return initBias_; }
  void setInitBias(const vector_t& initBias) { initBias_ = initBias; }

  /**
   * Multiplier on v_obstacle * elapsedTime in the pessimistic keep-out radius.
   * 1.0 reproduces the CasADi reference; smaller values relax the worst-case disk on retry.
   */
  scalar_t getPessiScale() const { return pessiScale_; }
  void setPessiScale(scalar_t pessiScale) { pessiScale_ = pessiScale; }

 private:
  vector_t goal_;
  matrix_t obstacles_;
  int gaitOffset_ = 0;
  vector_t initBias_;
  scalar_t pessiScale_ = 1.0;
};

}  // namespace opti_pessi
