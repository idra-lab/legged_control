#include <gtest/gtest.h>

#include "opti_pessi_wbc_bridge/PhaseClock.h"

using namespace opti_pessi_bridge;

TEST(PhaseClock, startsAtPhaseZeroWithEvenParity) {
  PhaseClock clock;
  clock.reset(0.25);

  EXPECT_EQ(clock.phaseIndex(), 0);
  EXPECT_EQ(clock.parity(), 0);
  EXPECT_EQ(clock.modeNumber(), 6u);
  EXPECT_NEAR(clock.tau(), 0.0, 1e-12);
  EXPECT_NEAR(clock.progress(), 0.0, 1e-12);
}

TEST(PhaseClock, advanceWithinPhaseDoesNotCrossBoundary) {
  PhaseClock clock;
  clock.reset(0.25);

  EXPECT_FALSE(clock.advance(0.1));
  EXPECT_NEAR(clock.tau(), 0.1, 1e-12);
  EXPECT_EQ(clock.phaseIndex(), 0);
  EXPECT_NEAR(clock.progress(), 0.4, 1e-12);
}

TEST(PhaseClock, crossingBoundaryFlipsParityAndCarriesRemainder) {
  PhaseClock clock;
  clock.reset(0.25);

  EXPECT_FALSE(clock.advance(0.2));
  EXPECT_TRUE(clock.advance(0.1));  // 0.30 total, crosses at 0.25

  EXPECT_EQ(clock.phaseIndex(), 1);
  EXPECT_EQ(clock.parity(), 1);
  EXPECT_EQ(clock.modeNumber(), 9u);
  EXPECT_NEAR(clock.tau(), 0.05, 1e-12) << "the overshoot must carry into the new phase";
}

TEST(PhaseClock, parityAlternatesAcrossManyPhases) {
  PhaseClock clock;
  clock.reset(0.2);

  for (int phase = 1; phase <= 6; ++phase) {
    ASSERT_TRUE(clock.advance(0.2));
    EXPECT_EQ(clock.phaseIndex(), phase);
    EXPECT_EQ(clock.parity(), phase % 2);
    EXPECT_EQ(clock.modeNumber(), phase % 2 == 0 ? 6u : 9u);
  }
}

// The next phase's duration comes from the newly latched plan, so it must be settable at the
// boundary without disturbing the carried remainder.
TEST(PhaseClock, phaseDurationCanChangeAtBoundary) {
  PhaseClock clock;
  clock.reset(0.25);
  ASSERT_TRUE(clock.advance(0.30));
  ASSERT_NEAR(clock.tau(), 0.05, 1e-12);

  clock.setPhaseDuration(0.20);
  EXPECT_NEAR(clock.phaseDuration(), 0.20, 1e-12);
  EXPECT_NEAR(clock.tau(), 0.05, 1e-12) << "changing duration must not reset elapsed time";
  EXPECT_NEAR(clock.progress(), 0.25, 1e-12);
}

// progress() is what drives the swing arc, so it must never leave [0, 1].
TEST(PhaseClock, progressIsClampedToUnitInterval) {
  PhaseClock clock;
  clock.reset(0.25);
  clock.advance(0.24);
  EXPECT_LE(clock.progress(), 1.0);
  EXPECT_GE(clock.progress(), 0.0);
}

// The clamp must actually be exercised, not merely present: drive the clock into a state where
// tau_ exceeds the current phase duration and confirm progress() saturates instead of exceeding 1.
// Without this, deleting the clamp from progress() would still pass the test above.
TEST(PhaseClock, progressSaturatesWhenElapsedExceedsShortenedPhase) {
  PhaseClock clock;
  clock.reset(0.25);
  ASSERT_FALSE(clock.advance(0.10));

  // Shrink the current phase below the already-elapsed time. No boundary has been observed yet.
  clock.setPhaseDuration(0.05);

  EXPECT_GT(clock.tau(), clock.phaseDuration()) << "fixture must actually overrun the phase";
  EXPECT_DOUBLE_EQ(clock.progress(), 1.0) << "progress must saturate, never exceed 1";
}

// An advance spanning several phases must land on the correct phase index, not skip silently.
// parity() drives which stance pair the controller is told to stand on, so a dropped phase would
// mean the wrong two feet with no diagnostic.
TEST(PhaseClock, advanceSpanningMultiplePhasesLandsOnTheCorrectIndex) {
  PhaseClock clock;
  clock.reset(0.20);

  // 0.70 s at 0.20 s per phase: three full boundaries, 0.10 s into phase 3.
  EXPECT_TRUE(clock.advance(0.70));

  EXPECT_EQ(clock.phaseIndex(), 3);
  EXPECT_EQ(clock.parity(), 1);
  EXPECT_EQ(clock.modeNumber(), 9u);
  EXPECT_NEAR(clock.tau(), 0.10, 1e-12);
  EXPECT_LT(clock.tau(), clock.phaseDuration()) << "elapsed time must be reduced into the new phase";
}

// After setPhaseDuration leaves the clock overrunning, the next advance must roll over correctly
// rather than staying stuck in an inconsistent state.
TEST(PhaseClock, advanceSelfCorrectsAfterPhaseDurationShrinksBelowElapsed) {
  PhaseClock clock;
  clock.reset(0.25);
  ASSERT_FALSE(clock.advance(0.10));
  clock.setPhaseDuration(0.04);  // tau_ = 0.10 now spans two and a half phases

  EXPECT_TRUE(clock.advance(0.0));

  EXPECT_EQ(clock.phaseIndex(), 2);
  EXPECT_NEAR(clock.tau(), 0.02, 1e-12);
  EXPECT_LT(clock.tau(), clock.phaseDuration());
}
