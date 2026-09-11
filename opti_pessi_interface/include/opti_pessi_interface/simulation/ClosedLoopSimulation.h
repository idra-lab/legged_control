#pragma once

#include <vector>

#include <ocs2_ipm/IpmMpc.h>

#include "opti_pessi_interface/OptiPessiInterface.h"
#include "opti_pessi_interface/definitions.h"

namespace opti_pessi {

/** Closed-loop trajectories logged by runClosedLoopSimulation. */
struct ClosedLoopResult {
  matrix_t stateTrajectory;                       // 10 x (steps + 1)
  matrix_t inputTrajectory;                       // 8 x steps
  matrix_t stanceHip0;                            // 2 x steps, world position of the first stance hip
  matrix_t stanceHip1;                            // 2 x steps
  std::vector<matrix_t> obstacleTrajectories;     // numObstacles slices of 2 x (steps + 1)
  vector_t solveTimes;                            // steps, wall-clock seconds per MPC iteration
  vector_t appliedPessiScale;                     // steps, keep-out growth factor actually achieved
  bool collision = false;

  /** Steps whose accepted solve used a relaxed keep-out (pessiScale < 1): NOT fully robust. */
  int relaxedSteps = 0;
  /** Steps where every solve failed and the saturated fallback input was applied. */
  int fallbackSteps = 0;
};

/**
 * Receding-horizon loop, one solve per contact phase.
 *
 * Each iteration pushes the trot parity and the measured obstacle centres into the reference
 * manager, solves the Opti-Pessi OCP warm-started from the shifted previous solution, applies
 * u_0 = u_0^opti for its own duration dt_0, and advances the obstacle plant by that same duration.
 * The plant successor is the optimistic branch's first knot (perfect-model assumption, matching the
 * Python reference).
 *
 * CONTINUATION ON THE KEEP-OUT GROWTH
 *
 * The pessimistic keep-out disk inflates to r_obs + v_obs * T over the horizon, which for the
 * default settings (v_obs = 1 m/s, T ~ 2 s) exceeds the initial robot-obstacle distance. The NLP is
 * still feasible -- the pessimistic branch just has to flee -- but a standing-still initial guess
 * violates it badly, and the open-loop LIP is exponentially unstable so the guess cannot simply be
 * nudged into feasibility. IPOPT copes via its restoration phase; the HPIPM-based IPM does not, and
 * its linesearch collapses to a zero step.
 *
 * So on failure the solve is retried along a continuation in OptiPessiReferenceManager::pessiScale,
 * which multiplies the growth term: an easy relaxed problem is solved first and warm-starts the next
 * one, walking back up to the nominal pessiScale = 1. A step whose best successful solve had
 * pessiScale < 1 is NOT robust to the full v_obs bound; those are counted in relaxedSteps and
 * reported per step in appliedPessiScale, never silently accepted.
 *
 * If every attempt fails, a saturated fallback input is integrated instead (fallbackSteps).
 *
 * REAL-TIME ITERATION
 *
 * With realTimeIteration = true the retry ladder above is switched OFF: exactly one solve per
 * control step, always warm-started from the shifted previous solution, and the result is always
 * kept as the next warm start whether or not the plan is trustworthy. That is the RTI contract --
 * the iterate converges along the closed loop rather than within a step, so discarding it and
 * re-solving cold would defeat the scheme and blow the per-step budget the scheme exists to respect.
 *
 * What is NOT switched off is the sanity gate on the APPLIED input: an insane or wildly infeasible
 * first input still falls back to the saturated / capture-point step, and still counts in
 * fallbackSteps. RTI bounds the computation per step; it does not make a bad step safe to execute.
 * relaxedSteps stays 0 in this mode because the keep-out continuation never runs.
 *
 * @param mpc  ocs2::IpmMpc built for this interface's problem and bound to its reference manager.
 */
ClosedLoopResult runClosedLoopSimulation(OptiPessiInterface& interface, ocs2::IpmMpc& mpc, bool verbose,
                                         bool realTimeIteration = false);

}  // namespace opti_pessi
