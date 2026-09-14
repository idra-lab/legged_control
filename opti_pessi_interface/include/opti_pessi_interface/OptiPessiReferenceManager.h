#pragma once

#include <stdexcept>

#include <ocs2_oc/synchronized_module/ReferenceManager.h>

#include "opti_pessi_interface/OptiPessiModelParameters.h"
#include "opti_pessi_interface/definitions.h"

namespace opti_pessi {

/**
 * Carries the per-iteration, non-differentiated data of the Opti-Pessi OCP: the goal, the measured
 * obstacles (centre, radius and speed bound of each slot), the trot parity at the start of the
 * horizon, and two homotopy knobs used by the closed loop when a solve fails.
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
        obstacleRadii_(vector_t::Constant(params.numObstacles(), params.obstacleRadius)),
        obstacleMaxSpeeds_(vector_t::Constant(params.numObstacles(), params.obstacleMaxSpeed)),
        initBias_(vector_t::Zero(2)) {}

  ~OptiPessiReferenceManager() override = default;

  /** Goal CoM position in the world frame. */
  const vector_t& getGoal() const { return goal_; }
  void setGoal(const vector_t& goal) { goal_ = goal; }

  /** Measured obstacle centres, numObstacles x 2. Held fixed inside one OCP solve. */
  const matrix_t& getObstacles() const { return obstacles_; }
  void setObstacles(const matrix_t& obstacles) { obstacles_ = obstacles; }

  /** Keep-out radius r_obs and speed bound v_obs of each obstacle slot. */
  const vector_t& getObstacleRadii() const { return obstacleRadii_; }
  const vector_t& getObstacleMaxSpeeds() const { return obstacleMaxSpeeds_; }

  /** Centres, radii and speed bounds together. The number of slots is fixed by the OCP: sizes must match. */
  void setObstacles(const matrix_t& obstacles, const vector_t& radii, const vector_t& maxSpeeds) {
    if (obstacles.rows() != obstacles_.rows() || obstacles.cols() != 2 || radii.size() != obstacles.rows() ||
        maxSpeeds.size() != obstacles.rows()) {
      throw std::invalid_argument("[OptiPessiReferenceManager] setObstacles: expected " + std::to_string(obstacles_.rows()) +
                                  " obstacle slots.");
    }
    obstacles_ = obstacles;
    obstacleRadii_ = radii;
    obstacleMaxSpeeds_ = maxSpeeds;
  }

  /** Obstacle block of the obstacle constraints' parameters: ObstacleP::DIM entries per slot. */
  vector_t getObstacleParameters() const {
    const int numObstacles = static_cast<int>(obstacles_.rows());
    vector_t p(ObstacleP::DIM * numObstacles);
    for (int j = 0; j < numObstacles; ++j) {
      p(ObstacleP::DIM * j + ObstacleP::X) = obstacles_(j, 0);
      p(ObstacleP::DIM * j + ObstacleP::Y) = obstacles_(j, 1);
      p(ObstacleP::DIM * j + ObstacleP::RADIUS) = obstacleRadii_(j);
      p(ObstacleP::DIM * j + ObstacleP::MAX_SPEED) = obstacleMaxSpeeds_(j);
    }
    return p;
  }

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
  vector_t obstacleRadii_;
  vector_t obstacleMaxSpeeds_;
  int gaitOffset_ = 0;
  vector_t initBias_;
  scalar_t pessiScale_ = 1.0;
};

}  // namespace opti_pessi
