#include "opti_pessi_wbc_bridge/LegKinematics.h"

#include <algorithm>
#include <cmath>

#include <Eigen/SVD>

namespace opti_pessi_bridge {

namespace {
double clampToUnit(double v) {
  return std::max(-1.0, std::min(1.0, v));
}
}  // namespace

LegGeometry aliengoLegGeometry() {
  LegGeometry geom;
  geom.thighOffset = 0.0868;
  geom.thighLength = 0.25;
  geom.calfLength = 0.25;

  // const.xacro: leg_offset_x = 0.2407, leg_offset_y = 0.051 -- these are the HAA joint origins,
  // confirmed against the generated URDF (LF_HAA origin xyz="0.2407 0.051 0", parent base).
  // NOTE: hip_offset (0.083) in const.xacro is a DEAD property -- it is defined there and used
  // nowhere in the aliengo URDF. The HAA-to-HFE lateral shift is thigh_offset = 0.0868, carried by
  // thighOffset above (URDF LF_HFE origin xyz="0 0.0868 0"). Do not fold 0.083 into kY "to fix"
  // the mismatch: that would displace every hip by 8.3 cm.
  constexpr double kX = 0.2407;
  constexpr double kY = 0.051;

  // Joint-block order: LF, LH, RF, RH.
  geom.hipPositionInBase[0] = Eigen::Vector3d(kX, kY, 0.0);    // LF
  geom.hipPositionInBase[1] = Eigen::Vector3d(-kX, kY, 0.0);   // LH
  geom.hipPositionInBase[2] = Eigen::Vector3d(kX, -kY, 0.0);   // RF
  geom.hipPositionInBase[3] = Eigen::Vector3d(-kX, -kY, 0.0);  // RH

  geom.abductionSign[0] = 1.0;   // LF, left
  geom.abductionSign[1] = 1.0;   // LH, left
  geom.abductionSign[2] = -1.0;  // RF, right
  geom.abductionSign[3] = -1.0;  // RH, right
  return geom;
}

Eigen::Vector3d forwardKinematics(const LegGeometry& geom, size_t jointBlock, const Eigen::Vector3d& q) {
  const double d = geom.abductionSign[jointBlock] * geom.thighOffset;
  const double l2 = geom.thighLength;
  const double l3 = geom.calfLength;

  const double c1 = std::cos(q.x());
  const double s1 = std::sin(q.x());

  // Planar chain in the x-z plane of the HAA-rotated frame. Both HFE and KFE rotate about +y, and
  // the links hang along -z at zero angle.
  const double planarX = -l2 * std::sin(q.y()) - l3 * std::sin(q.y() + q.z());
  const double planarZ = -l2 * std::cos(q.y()) - l3 * std::cos(q.y() + q.z());

  // Rotate (0, d, planarZ) about x by q1, keeping planarX along x.
  Eigen::Vector3d footInHip;
  footInHip.x() = planarX;
  footInHip.y() = c1 * d - s1 * planarZ;
  footInHip.z() = s1 * d + c1 * planarZ;

  return geom.hipPositionInBase[jointBlock] + footInHip;
}

Eigen::Vector3d inverseKinematics(const LegGeometry& geom, size_t jointBlock, const Eigen::Vector3d& footInBase,
                                  bool* reachable) {
  const double d = geom.abductionSign[jointBlock] * geom.thighOffset;
  const double l2 = geom.thighLength;
  const double l3 = geom.calfLength;

  const Eigen::Vector3d p = footInBase - geom.hipPositionInBase[jointBlock];

  bool ok = true;

  // --- HAA ---------------------------------------------------------------------------------
  // The y component in the HAA-rotated frame must equal d:
  //     cos(q1) * py + sin(q1) * pz ... solved as R * cos(q1 - phi) = d, phi = atan2(pz, py).
  // Rearranged for the branch that keeps the knee below the trunk.
  const double R = std::hypot(p.y(), p.z());
  double q1 = 0.0;
  if (R < std::abs(d)) {
    ok = false;  // foot is inside the cylinder the abduction offset sweeps: unreachable
    q1 = std::atan2(p.z(), p.y());
  } else {
    const double phi = std::atan2(p.z(), p.y());
    // planarZ must be negative (foot below the hip), which selects this branch.
    q1 = phi + std::acos(clampToUnit(d / R));
    // Normalize into (-pi, pi].
    while (q1 > M_PI) q1 -= 2.0 * M_PI;
    while (q1 <= -M_PI) q1 += 2.0 * M_PI;
  }

  const double c1 = std::cos(q1);
  const double s1 = std::sin(q1);

  // --- planar two-link ---------------------------------------------------------------------
  const double planarX = p.x();
  const double planarZ = -s1 * p.y() + c1 * p.z();

  const double r2 = planarX * planarX + planarZ * planarZ;
  const double r = std::sqrt(r2);
  const double reach = l2 + l3;
  const double minReach = std::abs(l2 - l3);
  if (r > reach || r < minReach) {
    ok = false;
  }

  const double cosKnee = clampToUnit((r2 - l2 * l2 - l3 * l3) / (2.0 * l2 * l3));
  const double q3 = -std::acos(cosKnee);  // knee bends backward: KFE limits are negative

  // atan2(-planarX, -planarZ) is the direction of the hip-to-foot vector measured from the -z
  // axis, which is the zero of HFE.
  const double q2 = std::atan2(-planarX, -planarZ) - std::atan2(l3 * std::sin(q3), l2 + l3 * std::cos(q3));

  if (reachable != nullptr) {
    *reachable = ok;
  }
  return Eigen::Vector3d(q1, q2, q3);
}

Eigen::Matrix3d legJacobian(const LegGeometry& geom, size_t jointBlock, const Eigen::Vector3d& q) {
  const double d = geom.abductionSign[jointBlock] * geom.thighOffset;
  const double l2 = geom.thighLength;
  const double l3 = geom.calfLength;

  const double c1 = std::cos(q.x());
  const double s1 = std::sin(q.x());
  const double s2 = std::sin(q.y());
  const double c2 = std::cos(q.y());
  const double s23 = std::sin(q.y() + q.z());
  const double c23 = std::cos(q.y() + q.z());

  // Same planar chain as forwardKinematics, differentiated w.r.t. q2 and q3.
  const double planarZ = -l2 * c2 - l3 * c23;
  const double dPlanarX_dq2 = -l2 * c2 - l3 * c23;
  const double dPlanarX_dq3 = -l3 * c23;
  const double dPlanarZ_dq2 = l2 * s2 + l3 * s23;
  const double dPlanarZ_dq3 = l3 * s23;

  Eigen::Matrix3d J;
  // x = planarX, independent of q1.
  J(0, 0) = 0.0;
  J(0, 1) = dPlanarX_dq2;
  J(0, 2) = dPlanarX_dq3;
  // y = c1 * d - s1 * planarZ
  J(1, 0) = -s1 * d - c1 * planarZ;
  J(1, 1) = -s1 * dPlanarZ_dq2;
  J(1, 2) = -s1 * dPlanarZ_dq3;
  // z = s1 * d + c1 * planarZ
  J(2, 0) = c1 * d - s1 * planarZ;
  J(2, 1) = c1 * dPlanarZ_dq2;
  J(2, 2) = c1 * dPlanarZ_dq3;
  return J;
}

Eigen::Vector3d jointVelocities(const LegGeometry& geom, size_t jointBlock, const Eigen::Vector3d& q,
                                const Eigen::Vector3d& footVelocityInBase) {
  const Eigen::Matrix3d J = legJacobian(geom, jointBlock, q);

  // SINGULAR-VALUE-ADAPTIVE damping. A single fixed Tikhonov lambda cannot be both exact away from
  // singularities and meaningfully bounded at them -- those pull in opposite directions, and any
  // lambda small enough to clear an exactness tolerance provides essentially no protection.
  //
  // Instead the damping is ZERO while the leg is well conditioned (so the solve is exact, to
  // floating-point noise) and ramps in only as the smallest singular value collapses -- which for
  // this leg happens as the knee straightens and r approaches l2 + l3 = 0.5 m. That is a real
  // operating condition: the trot's workspace reaches r ~ 0.5.
  //
  // At the singularity the solution norm is bounded by roughly ||v|| / (2 * kMaxDamping), so
  // kMaxDamping is chosen to cap the commanded joint rate near the URDF's own limits
  // (const.xacro: hip_velocity_max 20, calf_velocity_max 15.89 rad/s).
  constexpr double kSigmaThreshold = 0.02;  // [m/rad] below this the leg is treated as near-singular
  constexpr double kMaxDamping = 0.025;     // caps ||qdot|| / ||v|| at ~20 rad/s per m/s

  const Eigen::JacobiSVD<Eigen::Matrix3d> svd(J, Eigen::ComputeFullU | Eigen::ComputeFullV);
  const Eigen::Vector3d sigma = svd.singularValues();
  const double sigmaMin = sigma(2);  // JacobiSVD returns singular values in descending order

  double lambda = 0.0;
  if (sigmaMin < kSigmaThreshold) {
    const double ratio = sigmaMin / kSigmaThreshold;
    lambda = kMaxDamping * std::sqrt(1.0 - ratio * ratio);
  }

  Eigen::Vector3d qdot = Eigen::Vector3d::Zero();
  const Eigen::Vector3d projected = svd.matrixU().transpose() * footVelocityInBase;
  for (int i = 0; i < 3; ++i) {
    const double s = sigma(i);
    const double denom = s * s + lambda * lambda;
    if (denom > 0.0) {
      qdot += svd.matrixV().col(i) * (s / denom) * projected(i);
    }
  }
  return qdot;
}

}  // namespace opti_pessi_bridge
