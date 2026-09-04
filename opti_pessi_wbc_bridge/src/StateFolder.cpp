#include "opti_pessi_wbc_bridge/StateFolder.h"

#include <cmath>

namespace opti_pessi_bridge {

namespace {
constexpr int kBasePoseOffset = 6;
constexpr int kJointAngleOffset = 12;
constexpr int kYawIndex = kBasePoseOffset + 3;

/** Rotation of a planar vector by yaw. */
Eigen::Vector2d rotateYaw(double yaw, const Eigen::Vector2d& v) {
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return Eigen::Vector2d(c * v.x() - s * v.y(), s * v.x() + c * v.y());
}
}  // namespace

ocs2::vector_t foldToLipState(const LegGeometry& geom, const ocs2::vector_t& centroidalState,
                              const ocs2::vector_t& centroidalInput, int parity, const FoldedHistory& history) {
  // centroidalInput is unused today: the CoM velocity is taken from the normalized momentum
  // shortcut below rather than the joint-velocity contribution. Kept in the signature for a
  // possible future refinement (see StateFolder.h).
  (void)centroidalInput;

  ocs2::vector_t x = ocs2::vector_t::Zero(opti_pessi::RobotX::DIM);

  const double baseX = centroidalState(kBasePoseOffset + 0);
  const double baseY = centroidalState(kBasePoseOffset + 1);
  const double yaw = centroidalState(kYawIndex);

  // Normalized centroidal momentum is h / m, so its linear part IS the CoM velocity.
  //
  // APPROXIMATION: the CoM is taken to be the base origin. That matches how comHeight is defined in
  // the planner's config, and the offset is small for this robot, but it is an approximation, not an
  // identity. If the Gazebo bring-up shows a systematic tracking offset, THIS IS THE FIRST PLACE TO
  // LOOK -- swap in Pinocchio's actual CoM (getCoMPosition on the measured configuration) here and
  // in the matching assembly step of the reference synthesizer.
  x(opti_pessi::RobotX::CX) = baseX;
  x(opti_pessi::RobotX::CY) = baseY;
  x(opti_pessi::RobotX::TH) = yaw;
  x(opti_pessi::RobotX::DCX) = centroidalState(0);
  x(opti_pessi::RobotX::DCY) = centroidalState(1);
  x(opti_pessi::RobotX::DTH) = centroidalState(5);

  // Stance feet, in gaitPair order, in the world frame.
  const auto pair = opti_pessi::gaitPair(parity);
  for (int k = 0; k < 2; ++k) {
    const size_t block = jointBlockOf(pair[k]);
    const Eigen::Vector3d q(centroidalState(kJointAngleOffset + 3 * block + 0),
                            centroidalState(kJointAngleOffset + 3 * block + 1),
                            centroidalState(kJointAngleOffset + 3 * block + 2));
    const Eigen::Vector3d footInBase = forwardKinematics(geom, block, q);
    const Eigen::Vector2d footWorld =
        Eigen::Vector2d(baseX, baseY) + rotateYaw(yaw, footInBase.head<2>());

    if (k == 0) {
      x(opti_pessi::RobotX::P0X) = footWorld.x();
      x(opti_pessi::RobotX::P0Y) = footWorld.y();
    } else {
      x(opti_pessi::RobotX::P1X) = footWorld.x();
      x(opti_pessi::RobotX::P1Y) = footWorld.y();
    }
  }

  if (history.valid) {
    x(opti_pessi::RobotX::PP0X) = history.previousFoot0.x();
    x(opti_pessi::RobotX::PP0Y) = history.previousFoot0.y();
    x(opti_pessi::RobotX::PP1X) = history.previousFoot1.x();
    x(opti_pessi::RobotX::PP1Y) = history.previousFoot1.y();
    x(opti_pessi::RobotX::PCX) = history.previousCom.x();
    x(opti_pessi::RobotX::PCY) = history.previousCom.y();
    x(opti_pessi::RobotX::PTH) = history.previousYaw;
  } else {
    // No history yet: seed from the current values so the reachability and mid-step rows start
    // satisfied instead of being handed zeros.
    x(opti_pessi::RobotX::PP0X) = x(opti_pessi::RobotX::P0X);
    x(opti_pessi::RobotX::PP0Y) = x(opti_pessi::RobotX::P0Y);
    x(opti_pessi::RobotX::PP1X) = x(opti_pessi::RobotX::P1X);
    x(opti_pessi::RobotX::PP1Y) = x(opti_pessi::RobotX::P1Y);
    x(opti_pessi::RobotX::PCX) = x(opti_pessi::RobotX::CX);
    x(opti_pessi::RobotX::PCY) = x(opti_pessi::RobotX::CY);
    x(opti_pessi::RobotX::PTH) = x(opti_pessi::RobotX::TH);
  }

  return x;
}

}  // namespace opti_pessi_bridge
