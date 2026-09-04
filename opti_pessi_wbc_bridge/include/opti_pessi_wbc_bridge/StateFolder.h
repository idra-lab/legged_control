#pragma once

#include <Eigen/Core>

#include <ocs2_core/Types.h>

#include "opti_pessi_wbc_bridge/LegIndexing.h"
#include "opti_pessi_wbc_bridge/LegKinematics.h"

namespace opti_pessi_bridge {

/** Previous-phase quantities the OCP carries as pure bookkeeping (RobotX PP0/PP1/PC/PTH). */
struct FoldedHistory {
  Eigen::Vector2d previousFoot0{Eigen::Vector2d::Zero()};
  Eigen::Vector2d previousFoot1{Eigen::Vector2d::Zero()};
  Eigen::Vector2d previousCom{Eigen::Vector2d::Zero()};
  double previousYaw{0.0};
  bool valid{false};
};

/**
 * Folds a measured centroidal whole-body state down to the 17-dof LIP state the Opti-Pessi OCP
 * takes as its initial condition.
 *
 * centroidalState : [normalized momentum(6), base pose(6), joint angles(12)]
 * centroidalInput : [contact forces(12), joint velocities(12)]
 *
 * The two stance feet are the pair opti_pessi::gaitPair(parity) designates, in that order, and are
 * reported in the world frame.
 */
ocs2::vector_t foldToLipState(const LegGeometry& geom, const ocs2::vector_t& centroidalState,
                              const ocs2::vector_t& centroidalInput, int parity, const FoldedHistory& history);

}  // namespace opti_pessi_bridge
