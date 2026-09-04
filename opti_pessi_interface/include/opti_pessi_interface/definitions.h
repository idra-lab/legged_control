#pragma once

#include <array>
#include <cmath>

#include <ocs2_core/Types.h>

namespace opti_pessi {

using ocs2::matrix_t;
using ocs2::scalar_t;
using ocs2::vector_t;

/**
 * IMPORTANT -- TIME CONVENTION
 *
 * OCS2 solves a continuous-time optimal control problem, but the Opti-Pessi OCP is intrinsically
 * discrete: one knot is one contact phase, and the phase duration is itself a decision variable.
 *
 * The convention used throughout this package is therefore:
 *
 *   - "time" passed around by OCS2 is the KNOT INDEX, not seconds. The horizon runs from 0 to N.
 *   - the IPM discretization uses dt = 1.0 with the EULER integrator (see config/task.info),
 *     so the multiple-shooting defect is  x_{i+1} = x_i + 1.0 * flowMap(x_i, u_i).
 *   - OptiPessiDynamicsAD::systemFlowMap therefore returns  lipMap(x, u) - x, which makes the
 *     Euler step reproduce the exact discrete LIP map of the Python reference.
 *   - the REAL contact-phase duration in seconds is the decision variable u(RobotU::DT), and the
 *     elapsed physical time is accumulated in the clock state x(CLOCK_INDEX).
 *
 * Do not "fix" the dt=1.0 / EULER settings: they are load-bearing.
 */

enum class Foot { FL = 0, FR = 1, RL = 2, RR = 3 };

/**
 * Per-branch robot state layout (R^10) -- exactly the reference's state vector
 * (ocp_quadruped.py:223, `x = opti.variable(10, N+1)`), with no auxiliary slots.
 *
 * NO PREVIOUS-PHASE VARIABLES LIVE HERE.
 *
 * Two constraint families of the reference relate knot i to knot i+1: foot reachability
 * (ocp_quadruped.py:116-128) and the mid-step collision plane (ocp_quadruped.py:279-284). An earlier
 * transcription carried the previous phase's footholds and pose in the state so those rows could be
 * written knot-locally; that grew the state to R^17. They are now written the way the reference
 * writes them: the next state is recomputed inside the constraint as x_{i+1} = lipMap(x_i, u_i), i.e.
 * the dynamics are stepped explicitly, and the rows are imposed over intervals i = 0..N-1.
 *
 * The multiple-shooting defect still forces x_{i+1} = lipMap(x_i, u_i) at any solution, so the two
 * forms describe the same feasible set. This one costs more conditioning (cosh/sinh of the decision
 * variable dt and products with alpha reach every Jacobian row) and buys back a state that matches
 * the reference one-for-one, plus rows on interval 0 that constrain the state the applied input
 * actually lands in.
 */
struct RobotX {
  static constexpr int CX = 0;
  static constexpr int CY = 1;
  static constexpr int TH = 2;
  static constexpr int DCX = 3;
  static constexpr int DCY = 4;
  static constexpr int DTH = 5;
  static constexpr int P0X = 6;
  static constexpr int P0Y = 7;
  static constexpr int P1X = 8;
  static constexpr int P1Y = 9;
  static constexpr int DIM = 10;
};

/** Per-branch robot input layout (R^8). The first four entries are the NEXT footholds. */
struct RobotU {
  static constexpr int P0X = 0;
  static constexpr int P0Y = 1;
  static constexpr int P1X = 2;
  static constexpr int P1Y = 3;
  static constexpr int ALPHA = 4;
  static constexpr int DT = 5;
  static constexpr int BETA = 6;
  static constexpr int GAMMA = 7;
  static constexpr int DIM = 8;
};

/**
 * Separating-hyperplane decision variables per obstacle, carried in the INPUT vector so the solver
 * treats them as free variables with no extra machinery.
 *
 * Layout: [phiMid, bMid, phiLand, bLand] -- two independent planes per obstacle per knot, matching
 * the reference's a = variable(4*n_obs, N), b = variable(2*n_obs, N). The mid-step plane separates
 * the hips at the half-way pose; the landing plane separates the hips AND both stance feet at the
 * knot pose. Without the mid-step plane the robot is unconstrained between knots, i.e. for up to
 * dtMax = 0.35 s per phase.
 *
 * The unit normal is parameterized by ANGLE, a = (cos phi, sin phi), rather than carried as a free
 * 2-vector with a separate ||a|| = 1 equality. The two are the same set, but the angle form makes
 * the unit-norm condition hold identically, and that matters: the collision rows
 *
 *     -(a.hip + b) >= 0        and        a.o + b - dMin >= 0
 *
 * are positively homogeneous in (a, b), so with a free a the solver can satisfy a geometrically
 * IMPOSSIBLE separation simply by inflating ||a||, dumping the infeasibility into the norm equality
 * instead. That is not hypothetical -- the free-vector encoding returns ||a|| ~ 95 on the very first
 * solve of scenario S4 and the closed loop degrades from there. With the angle form the cheat does
 * not exist, and the norm equality (and its constraint block) disappears entirely.
 */
constexpr int kHyperplaneVarsPerObs = 4;

/**
 * NO SLACK VARIABLE LIVES HERE -- read this before adding one back.
 *
 * A per-obstacle slack s_j was once carried in the input, relaxing the pessimistic keep-out row to
 * a.o + b - dMin + s >= 0 with a cost penalty of 1e4*s + 1e5*s^2. It ended runs. The row is coupled
 * to the robot through -(a.y + b) >= 0, so shrinking s means growing b, and growing b pushes the
 * robot's own half-space away from the obstacle. Against wc = 1.0 on CoM position error, a 1e5
 * weight on s makes flying the CoM tens of metres CHEAPER than carrying 0.2 m of slack: the solver
 * duly launched the plan (cy = 0.01, 1.6, 3.9, 7.6, 14.1, 24.9, 44.4, 77.7 on consecutive steps,
 * dt pinned at dtMax throughout) and the cost ladder ran to 1e10.
 *
 * It is also redundant. IpmSolver is a primal-dual interior point method: it already carries a
 * slack and a dual per hard inequality, governed by a barrier parameter that is driven to zero on a
 * schedule, with fraction-to-boundary line search and complementarity control. A hand-rolled slack
 * inside the decision vector gets none of that -- just a fixed 1e5 quadratic that never decreases
 * and permanently wrecks the Hessian conditioning.
 *
 * If the keep-out genuinely has to be softened, the levers are the pessiScale continuation in
 * ClosedLoopSimulation.cpp and OptimalControlProblem::softConstraintPtr with a RelaxedBarrierPenalty
 * (bounded gradient by construction). Not this.
 */

/** Offsets within one obstacle's hyperplane block: two independent (phi, b) planes. */
struct Hyperplane {
  static constexpr int MID_PHI = 0;
  static constexpr int MID_B = 1;
  static constexpr int LAND_PHI = 2;
  static constexpr int LAND_B = 3;
};

/** Augmented state: optimistic branch || pessimistic branch || elapsed-time clock. */
constexpr int AUG_STATE_DIM = 2 * RobotX::DIM + 1;

/** Index of the elapsed-time clock inside the augmented state. */
constexpr int CLOCK_INDEX = 2 * RobotX::DIM;

/** Augmented input: u_opti || u_pessi || hyperplanes(opti) || hyperplanes(pessi). */
inline int augInputDim(int numObstacles) {
  return 2 * RobotU::DIM + 2 * kHyperplaneVarsPerObs * numObstacles;
}

/** Offset of the optimistic branch's hyperplane block inside the augmented input. */
inline int optiHyperplaneOffset() {
  return 2 * RobotU::DIM;
}

/** Offset of the pessimistic branch's hyperplane block inside the augmented input. */
inline int pessiHyperplaneOffset(int numObstacles) {
  return 2 * RobotU::DIM + kHyperplaneVarsPerObs * numObstacles;
}

inline bool isLeft(Foot f) {
  return f == Foot::FL || f == Foot::RL;
}

/** Diagonal trot: even knots stand on (FR, RL), odd knots on (FL, RR). */
inline std::array<Foot, 2> gaitPair(int knotParity) {
  if (knotParity % 2 == 0) {
    return {Foot::FR, Foot::RL};
  }
  return {Foot::FL, Foot::RR};
}

/** Map an OCS2 "time" (== knot index, see the note above) to a valid interval index in [0, N-1]. */
inline int intervalIndex(scalar_t time, int N) {
  const int i = static_cast<int>(std::lround(time));
  if (i < 0) {
    return 0;
  }
  if (i >= N) {
    return N - 1;
  }
  return i;
}

}  // namespace opti_pessi
