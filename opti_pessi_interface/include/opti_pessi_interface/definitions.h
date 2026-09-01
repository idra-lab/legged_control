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

/** Per-branch robot state layout (R^10). */
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
  /**
   * Footholds of the PREVIOUS contact phase, carried along so that the reference's reachability
   * condition "the feet you are standing on must still be reachable once the CoM has moved"
   * (ocp_quadruped.py:118-121, p_i measured against base i+1) can be written at the knot where both
   * quantities live, instead of being composed with the dynamics. They are a pure copy -- see
   * lipMap -- so they cost nothing in conditioning. Dropping this family lets the robot stride far
   * harder than the reference: max speed 1.53 m/s against the reference's 0.88 m/s.
   */
  static constexpr int PP0X = 10;
  static constexpr int PP0Y = 11;
  static constexpr int PP1X = 12;
  static constexpr int PP1Y = 13;
  static constexpr int DIM = 14;
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
 * Layout: [phiMid, phiLand, bMid, bLand].
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
constexpr int kHyperplaneVarsPerObs = 2;

/** Offsets within one obstacle's hyperplane block. */
struct Hyperplane {
  static constexpr int PHI = 0;
  static constexpr int B = 1;
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
