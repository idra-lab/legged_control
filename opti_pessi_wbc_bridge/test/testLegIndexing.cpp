#include <gtest/gtest.h>

#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>

#include "opti_pessi_wbc_bridge/LegIndexing.h"

using namespace opti_pessi_bridge;
using opti_pessi::Foot;

// Contact order is {LF, RF, LH, RH}; opti_pessi's Foot enum is {FL, FR, RL, RR}.
// Same order, different names -- so the contact map is the identity.
TEST(LegIndexing, contactIndexIsIdentity) {
  EXPECT_EQ(contactIndexOf(Foot::FL), 0u);
  EXPECT_EQ(contactIndexOf(Foot::FR), 1u);
  EXPECT_EQ(contactIndexOf(Foot::RL), 2u);
  EXPECT_EQ(contactIndexOf(Foot::RR), 3u);
}

// Joint order is {LF, LH, RF, RH}, which crosses over relative to contact order.
TEST(LegIndexing, jointBlockCrossesOver) {
  EXPECT_EQ(jointBlockOf(Foot::FL), 0u);
  EXPECT_EQ(jointBlockOf(Foot::RL), 1u);
  EXPECT_EQ(jointBlockOf(Foot::FR), 2u);
  EXPECT_EQ(jointBlockOf(Foot::RR), 3u);
}

// Diagonal trot: even knots stand on (FR, RL), odd knots on (FL, RR).
TEST(LegIndexing, modeNumberMatchesGaitPair) {
  EXPECT_EQ(modeNumberForParity(0), static_cast<size_t>(ocs2::legged_robot::ModeNumber::RF_LH));
  EXPECT_EQ(modeNumberForParity(1), static_cast<size_t>(ocs2::legged_robot::ModeNumber::LF_RH));
  EXPECT_EQ(modeNumberForParity(0), 6u);
  EXPECT_EQ(modeNumberForParity(1), 9u);
}

// The mode number must agree with the stance flags derived from gaitPair, on the contact ordering.
TEST(LegIndexing, modeNumberAgreesWithStanceFlags) {
  for (int parity = 0; parity < 2; ++parity) {
    const auto stance = ocs2::legged_robot::modeNumber2StanceLeg(modeNumberForParity(parity));
    const auto pair = opti_pessi::gaitPair(parity);

    std::array<bool, 4> expected{false, false, false, false};
    expected[contactIndexOf(pair[0])] = true;
    expected[contactIndexOf(pair[1])] = true;

    for (size_t i = 0; i < 4; ++i) {
      EXPECT_EQ(stance[i], expected[i]) << "parity " << parity << " contact " << i;
    }
  }
}
