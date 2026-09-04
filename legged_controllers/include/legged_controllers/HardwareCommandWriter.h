//
// Shared hardware command write-out for legged_controllers.
//
// Extracted from LeggedController::update() so that OptiPessiController does not duplicate the
// block: per-joint torque/position/velocity/kp/kd write-out, gated on the same safety check that
// LeggedController has always run first. Both controllers call writeHardwareCommand() with their
// own joint-handle vector, model info, safety checker and measured observation.
//

#pragma once

#include <cstddef>
#include <vector>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_core/Types.h>
#include <ocs2_mpc/SystemObservation.h>

#include "legged_controllers/SafetyChecker.h"

namespace legged {

/**
 * Writes one whole-body-control solution out to the joint hardware command interfaces: per-joint
 * desired position, desired velocity, kp = 0, kd = 3, feedforward torque -- the exact block
 * LeggedController::update() has always performed after calling WbcBase::update(), including its
 * safety-check gate.
 *
 * Runs the safety check against `measuredObservation` (the ESTIMATED/measured state, not the
 * reference) before writing anything. On failure, no hardware command is written and false is
 * returned so the caller can abort its update() cycle (LeggedController returns
 * controller_interface::return_type::ERROR in that case; callers should do the same).
 *
 * Templated on the joint-handle type so both LeggedController's and OptiPessiController's
 * HybridJointHandle (two structurally-identical, independently-defined structs -- see the RULING in
 * task-12-brief.md, which forbids modifying LeggedController.h to share the type) satisfy it via
 * duck typing: any type exposing setCommand(pos, vel, kp, kd, effort) works.
 */
template <typename JointHandleT>
bool writeHardwareCommand(std::vector<JointHandleT>& hybridJointHandles, const ocs2::CentroidalModelInfo& info,
                          SafetyChecker& safetyChecker, const ocs2::SystemObservation& measuredObservation,
                          const ocs2::vector_t& desiredState, const ocs2::vector_t& desiredInput, const ocs2::vector_t& wbcSolution,
                          const rclcpp::Logger& logger, const char* safetyCheckFailedMessage = "Safety check failed!") {
  using namespace ocs2;

  const vector_t torque = wbcSolution.tail(info.actuatedDofNum);
  const vector_t posDes = centroidal_model::getJointAngles(desiredState, info);
  const vector_t velDes = centroidal_model::getJointVelocities(desiredInput, info);

  if (!safetyChecker.check(measuredObservation, desiredState, desiredInput)) {
    RCLCPP_ERROR(logger, "%s", safetyCheckFailedMessage);
    return false;
  }

  for (size_t j = 0; j < info.actuatedDofNum; ++j) {
    hybridJointHandles[j].setCommand(posDes(j), velDes(j), 0, 3, torque(j));
  }
  return true;
}

}  // namespace legged
