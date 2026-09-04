#pragma once

#include <array>

#include <Eigen/Core>
#include <ocs2_core/Types.h>

#include "opti_pessi_wbc_bridge/LegIndexing.h"
#include "opti_pessi_wbc_bridge/LegKinematics.h"
#include "opti_pessi_wbc_bridge/PlanBuffer.h"

namespace opti_pessi_bridge {

/** Physical constants shared by the LIP plan and the whole-body reference. */
struct SynthesisSettings {
  double comHeight{0.38};
  double mass{24.24};
  double inertia{1.048};
  double gravity{9.81};
  double swingHeight{0.1};
};

/**
 * The plan's LIP state at partial phase time tau.
 *
 * lipMap takes its duration solely from u(RobotU::DT), so substituting DT := tau evaluates the
 * exact closed-form LIP flow partway through the phase -- no separate integrator is needed. Only
 * the CoM/yaw fields of the result are meaningful here; the foot fields hold the NEXT footholds
 * and are handled by the swing-trajectory code instead.
 */
ocs2::vector_t lipStateAt(const PhasePlan& plan, double tau, const SynthesisSettings& settings);

/** Foot targets for all four legs, indexed by CONTACT index. */
struct FootReference {
  std::array<Eigen::Vector3d, kNumLegs> position{};
  std::array<Eigen::Vector3d, kNumLegs> velocity{};
  std::array<bool, kNumLegs> inContact{};
};

/**
 * Foot positions and velocities at partial phase time tau, in the world frame.
 *
 * The stance pair gaitPair(parity) is held at the plan's p0/p1. The swing pair gaitPair(parity+1)
 * travels from where those feet currently stand to the next footholds in u(P0X..P1Y), with a
 * cubic profile in xy and a raised-cosine arc in z that has zero velocity at lift-off and
 * touchdown.
 */
FootReference footReferenceAt(const PhasePlan& plan, double tau, int parity, const SynthesisSettings& settings);

/**
 * Per-leg contact forces at partial phase time tau, indexed by CONTACT index. Swing legs get zero.
 *
 * Horizontal components come from the plan's (beta, gamma) split of m*ddc, evaluated at the CoM
 * position at tau rather than at the knot, which is the true LIP law ddc = omega^2 (c - cop).
 * Vertical components follow the CoP definition z = p0 + alpha*(p1 - p0), so fz0 = (1-alpha)*m*g
 * and fz1 = alpha*m*g -- the split that puts the net vertical force exactly at the commanded CoP.
 */
std::array<Eigen::Vector3d, kNumLegs> contactForcesAt(const PhasePlan& plan, double tau, int parity,
                                                      const SynthesisSettings& settings);

/** The reference legged_wbc consumes, in centroidal form. */
struct CentroidalReference {
  ocs2::vector_t state;   // [normalized momentum(6), base pose(6), joint angles(12)]
  ocs2::vector_t input;   // [contact forces(12), joint velocities(12)]
  size_t mode{0};
  bool allFeetReachable{true};
};

/**
 * The whole conversion, for one control tick: LIP phase plan at partial time tau -> the centroidal
 * state, input and contact mode legged_wbc's update() expects.
 */
CentroidalReference synthesize(const LegGeometry& geom, const PhasePlan& plan, double tau, int parity,
                               const SynthesisSettings& settings);

}  // namespace opti_pessi_bridge
