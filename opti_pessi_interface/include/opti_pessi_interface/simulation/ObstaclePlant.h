#pragma once

#include "opti_pessi_interface/OptiPessiModelParameters.h"
#include "opti_pessi_interface/definitions.h"

namespace opti_pessi {

/**
 * Ground-truth obstacle motion used by the closed-loop simulation. The OCP never sees this: it only
 * ever receives the current measured centres and the speed bound v_obs.
 */
struct ObstaclePlant {
  matrix_t positions;  // numObstacles x 2
  matrix_t direction;  // numObstacles x 2, current heading (patrol flips it)
  vector_t radius;     // circle mode: orbit radius about the goal
  vector_t phase;      // circle mode: current angle
  bool circleInitialized = false;
};

ObstaclePlant makeObstaclePlant(const OptiPessiModelParameters& params);

/**
 * Advances every obstacle by dt seconds under its own motion law.
 * @param robotCom  current CoM, used only by the antagonist law (which chases the robot).
 * @param step      simulation step index, used by the patrol law to latch its initial direction.
 */
void stepObstacles(ObstaclePlant& plant, const OptiPessiModelParameters& params, const vector_t& robotCom, scalar_t dt, int step);

/**
 * True if the convex hull of {4 hips, 2 stance feet} comes closer than dMin to the obstacle centre.
 * Brute-force support-function check over a fixed set of directions -- this is the collision
 * DETECTOR for the simulation log, not the constraint used by the solver.
 */
bool hipsAndFeetCollide(const vector_t& com, scalar_t theta, const vector_t& feet, const vector_t& obstacle,
                        const OptiPessiModelParameters& params, scalar_t dMin);

}  // namespace opti_pessi
