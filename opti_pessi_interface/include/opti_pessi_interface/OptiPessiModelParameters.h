#pragma once

#include <array>
#include <string>
#include <vector>

#include "opti_pessi_interface/definitions.h"

namespace opti_pessi {

enum class ObstacleMovement { Straight, Patrol, Circle, Antagonist };

ObstacleMovement obstacleMovementFromString(const std::string& name);
std::string toString(ObstacleMovement movement);

/**
 * Everything the Opti-Pessi OCP needs, loaded from config/task.info (robot + cost + limits +
 * horizon) and config/scenario_S*.info (goal + obstacles + obstacle plant + simulation).
 *
 * Field names mirror aliengo_conf.py of the Python reference so the two can be diffed by eye.
 */
struct OptiPessiModelParameters {
  // --- model (task.info: model, hipOffsets) ---
  scalar_t comHeight = 0.38;            // h    [m]
  scalar_t gravity = 9.81;              // g    [m/s^2]
  scalar_t mass = 24.24;                // m    [kg]
  scalar_t inertia = 1.048;             // I    [kg m^2] about the vertical axis
  scalar_t frictionCoefficient = 0.8;   // mu
  std::array<vector_t, 4> hipOffsets{}; // body-frame hip offsets, indexed by Foot

  // --- cost weights (task.info: cost) ---
  scalar_t wc = 1.0;        // CoM position error
  scalar_t wdc = 2.0;       // CoM velocity magnitude
  scalar_t wp = 0.5;        // foothold-to-hip distance
  scalar_t wa = 0.5;        // CoP centering (alpha - 1/2)
  scalar_t wdt = 1e-2;      // contact-phase duration
  scalar_t wtheta = 0.0;    // heading alignment (identically zero, kept for 1:1 fidelity)
  scalar_t wdtheta = 2.0;   // yaw rate

  // --- limits (task.info: limits) ---
  scalar_t alphaReduction = 0.1;  // CoP margin: alpha in [eps, 1-eps]
  scalar_t dtMin = 0.2;           // [s]
  scalar_t dtMax = 0.35;          // [s]
  scalar_t dtCost0 = 0.35;        // preferred phase duration [s]
  scalar_t footHipMax = 0.1;      // foot-to-hip reachability radius [m]
  scalar_t dcxMax = 1.5;          // body-frame longitudinal speed [m/s]
  scalar_t dcyMax = 0.45;         // body-frame lateral speed [m/s]
  scalar_t dthetaMax = 0.8;       // yaw rate [rad/s]

  // --- horizon (task.info: horizon) ---
  int N = 6;  // number of contact phases

  /**
   * Measured robot state the simulation starts from, R^10:
   * [cx, cy, theta, dcx, dcy, dtheta, p0x, p0y, p1x, p1y].
   * This is the whole per-branch state: RobotX::DIM == kMeasuredStateDim.
   */
  static constexpr int kMeasuredStateDim = RobotX::DIM;
  vector_t initialState = vector_t::Zero(kMeasuredStateDim);

  // --- scenario: goal and obstacles ---
  vector_t goal = vector_t::Zero(2);
  matrix_t obstaclePositions;      // numObstacles x 2, initial/measured centres
  scalar_t obstacleRadius = 0.2;   // r_obs [m]
  scalar_t obstacleMaxSpeed = 1.0; // v_obs [m/s], the bound assumed by the pessimistic branch

  // --- scenario: obstacle plant (ground truth, unknown to the OCP) ---
  std::vector<ObstacleMovement> obstacleMovement;
  vector_t obstacleSpeed;  // numObstacles, the TRUE speed
  matrix_t obstacleDir;    // numObstacles x 2

  // --- scenario: closed-loop simulation ---
  scalar_t simTime = 30.0;
  scalar_t goalTolerance = 0.1;
  std::string figName = "mpc_sim_";

  /** LIP natural frequency omega = sqrt(g/h). */
  scalar_t omega() const { return std::sqrt(gravity / comHeight); }

  int numObstacles() const { return static_cast<int>(obstaclePositions.rows()); }

  int stateDim() const { return AUG_STATE_DIM; }
  int inputDim() const { return augInputDim(numObstacles()); }
};

inline const vector_t& hipOf(const OptiPessiModelParameters& p, Foot f) {
  return p.hipOffsets[static_cast<size_t>(f)];
}

/**
 * Loads the model/cost/limits/horizon block from taskFile and the goal/obstacle/simulation block
 * from scenarioFile. Throws std::runtime_error if either file is missing or malformed.
 */
OptiPessiModelParameters loadOptiPessiModelParameters(const std::string& taskFile, const std::string& scenarioFile, bool verbose);

}  // namespace opti_pessi
