#pragma once

#include <array>
#include <cstddef>

#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>

#include "opti_pessi_interface/definitions.h"

namespace opti_pessi_bridge {

constexpr size_t kNumLegs = 4;

/**
 * TWO ORDERINGS COEXIST IN THIS STACK. Conflating them is the single most likely way to get a
 * robot that looks almost right and falls over.
 *
 *   contact order : {LF, RF, LH, RH}  -- contactNames3DoF, contact-force blocks of the centroidal
 *                                        input, contact_flag_t, ModeNumber bits.
 *   joint order   : {LF, LH, RF, RH}  -- Pinocchio q, defaultJointState (reference.info),
 *                                        joint-velocity blocks of the centroidal input.
 *
 * opti_pessi::Foot{FL, FR, RL, RR} is the same sequence as the contact order under different
 * names, so contactIndexOf is the identity. It is NOT the identity into joint blocks.
 */
inline size_t contactIndexOf(opti_pessi::Foot foot) {
  switch (foot) {
    case opti_pessi::Foot::FL: return 0;  // LF
    case opti_pessi::Foot::FR: return 1;  // RF
    case opti_pessi::Foot::RL: return 2;  // LH
    case opti_pessi::Foot::RR: return 3;  // RH
  }
  return 0;
}

inline size_t jointBlockOf(opti_pessi::Foot foot) {
  switch (foot) {
    case opti_pessi::Foot::FL: return 0;  // LF
    case opti_pessi::Foot::RL: return 1;  // LH
    case opti_pessi::Foot::FR: return 2;  // RF
    case opti_pessi::Foot::RR: return 3;  // RH
  }
  return 0;
}

/** Diagonal trot: even phases stand on (FR, RL) = RF_LH = 6, odd on (FL, RR) = LF_RH = 9. */
inline size_t modeNumberForParity(int parity) {
  const auto pair = opti_pessi::gaitPair(parity);
  size_t mode = 0;
  // ModeNumber is a bitmask over {LF, RF, LH, RH} with LF the most significant of the four bits.
  for (const auto foot : pair) {
    mode |= (1u << (3 - contactIndexOf(foot)));
  }
  return mode;
}

}  // namespace opti_pessi_bridge
