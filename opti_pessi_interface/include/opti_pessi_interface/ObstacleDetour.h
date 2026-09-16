#pragma once

#include "opti_pessi_interface/OptiPessiModelParameters.h"
#include "opti_pessi_interface/definitions.h"

namespace opti_pessi {

/**
 * Temporary goal that walks the robot AROUND an obstacle sitting between it and the goal.
 *
 * The OCP only tracks the goal (wc * |c - goal|^2) over a short horizon (N phases, well under a second), and the
 * pessimistic keep-out grows by v_obs over that horizon. With the obstacle on the straight line to the goal, going
 * around it means first increasing the distance to the goal, which no horizon this short ever pays for: the robot
 * walks up to the grown disk and stalls there (measured on goal (5,-3), obstacle (2,-1): stuck at (1.1,-0.8) in the
 * standalone simulation, capture-point stops, restarts from stance and eventually a fall in Gazebo).
 *
 * The detour disk of each obstacle is what the horizon may make it grow into, plus the robot:
 *
 *     R = r_obs + v_obs * N * dtMax + rHull + kDetourMargin
 *
 * While the segment robot -> goal crosses that disk (obstacle ahead, goal outside the disk), the goal handed to the
 * OCP is a point past the tangent from the robot to the disk, on the side chosen when the obstacle started blocking.
 * Inside the disk the walking direction keeps turning away from the obstacle with the depth, never past 135 deg, so the
 * robot circles out instead of reversing. The side is kept while the same obstacle blocks, so an obstacle dead on the
 * line cannot make it chatter; the detour is released with hysteresis once the line clears or the obstacle falls
 * behind.
 *
 * Not thread-safe: one instance per thread that pushes references (it remembers the chosen side).
 */
class ObstacleDetour {
 public:
  explicit ObstacleDetour(const OptiPessiModelParameters& params);

  /**
   * Goal to hand to the OCP. obstacles is numSlots x 2; a slot with radius and speed bound both zero is a free slot
   * and is ignored. Returns goal itself when nothing blocks.
   */
  vector_t detourGoal(const vector_t& com, const vector_t& goal, const matrix_t& obstacles, const vector_t& radii,
                      const vector_t& maxSpeeds);

  /** Whether the last detourGoal() call returned a detour rather than the goal. */
  bool active() const { return active_; }

  /** Forgets the chosen side (e.g. on a restart). */
  void reset() { active_ = false; }

 private:
  scalar_t horizonTime_ = 0.0;  // N * dtMax [s]
  scalar_t hullRadius_ = 0.0;   // disk about the CoM containing every hip [m]

  bool active_ = false;
  scalar_t side_ = 1.0;                         // +1: pass with the obstacle on the robot's right, -1: on its left
  vector_t blockingCentre_ = vector_t::Zero(2);  // obstacle the side was chosen for
};

}  // namespace opti_pessi
