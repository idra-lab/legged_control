#pragma once

#include <array>
#include <cstddef>

#include <Eigen/Core>

#include "opti_pessi_wbc_bridge/LegIndexing.h"

namespace opti_pessi_bridge {

/**
 * Geometry of the standard Unitree 3-dof leg: abduction (HAA) about x, then hip pitch (HFE) and
 * knee (KFE) both about y. Indexed by JOINT BLOCK ({LF, LH, RF, RH}), because the joint vector is
 * the only consumer.
 */
struct LegGeometry {
  double thighOffset{0.0868};  // HAA-to-HFE offset along +-y
  double thighLength{0.25};
  double calfLength{0.25};
  std::array<Eigen::Vector3d, kNumLegs> hipPositionInBase{};  // HAA joint origin in the base frame
  std::array<double, kNumLegs> abductionSign{};               // +1 for left legs, -1 for right
};

/** Aliengo constants from legged_unitree_description/urdf/aliengo/const.xacro. */
LegGeometry aliengoLegGeometry();

/** Foot position in the base frame, for one leg. q = (HAA, HFE, KFE). */
Eigen::Vector3d forwardKinematics(const LegGeometry& geom, size_t jointBlock, const Eigen::Vector3d& q);

/**
 * Joint angles reproducing footInBase, for one leg. Sets *reachable to false and returns the
 * closest reachable configuration when the target lies outside the leg's LINK REACH (the
 * abduction-offset cylinder and the thigh/calf link-length annulus). Joint limits (HAA/HFE/KFE
 * range) are NOT checked here -- a result can be flagged reachable while still lying outside a
 * joint's physical limit.
 *
 * PRECONDITION -- defined only for foot targets below the hip. forwardKinematics is NOT
 * injective over the full joint box: the HAA equation cos(q1)*py + sin(q1)*pz = d has two
 * solutions, one with the foot below the hip (planarZ < 0) and one with the knee folded up so the
 * foot sits above it (planarZ > 0); both reach the identical foot position, so no function of
 * footInBase alone can recover which one was intended. This implementation always returns the
 * below-hip branch. That is not a limitation in practice: the robot rides at comHeight = 0.38 m
 * with feet ranging over z in [0, 0.1], so the foot sits ~0.3 m below the hip at all times, far
 * from the planarZ = 0 boundary where the ambiguity lives. For a target above the hip, the
 * returned joints still place the foot exactly on the target (forward kinematics of the result
 * matches footInBase), but they will generally NOT match whatever joint configuration originally
 * produced that target -- do not "fix" this by reintroducing the other branch; there is no
 * foot-position-only way to choose correctly between them. See testLegKinematics.cpp's IK DOMAIN
 * block for the derivation and the tests that pin this contract down.
 */
Eigen::Vector3d inverseKinematics(const LegGeometry& geom, size_t jointBlock, const Eigen::Vector3d& footInBase,
                                  bool* reachable);

/** d(footInBase) / dq for one leg, 3x3. */
Eigen::Matrix3d legJacobian(const LegGeometry& geom, size_t jointBlock, const Eigen::Vector3d& q);

/** Joint velocities reproducing footVelocityInBase at configuration q. */
Eigen::Vector3d jointVelocities(const LegGeometry& geom, size_t jointBlock, const Eigen::Vector3d& q,
                                const Eigen::Vector3d& footVelocityInBase);

}  // namespace opti_pessi_bridge
