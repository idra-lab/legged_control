#include <gtest/gtest.h>

#include <atomic>
#include <thread>

#include "opti_pessi_wbc_bridge/PlanBuffer.h"

using namespace opti_pessi_bridge;

namespace {
PhasePlan makePlan(int index, double dtValue) {
  PhasePlan plan;
  plan.robotState = ocs2::vector_t::Zero(opti_pessi::RobotX::DIM);
  plan.robotInput = ocs2::vector_t::Zero(opti_pessi::RobotU::DIM);
  plan.robotInput(opti_pessi::RobotU::DT) = dtValue;
  plan.phaseIndex = index;
  plan.valid = true;
  return plan;
}
}  // namespace

TEST(PlanBuffer, startsEmpty) {
  PlanBuffer buffer;
  EXPECT_FALSE(buffer.hasPending());
  EXPECT_FALSE(buffer.current().valid);
}

TEST(PlanBuffer, publishedPlanBecomesPending) {
  PlanBuffer buffer;
  buffer.publish(makePlan(3, 0.22));

  ASSERT_TRUE(buffer.hasPending());
  const auto taken = buffer.takePending();

  EXPECT_EQ(taken.phaseIndex, 3);
  EXPECT_NEAR(taken.robotInput(opti_pessi::RobotU::DT), 0.22, 1e-12);
  EXPECT_FALSE(buffer.hasPending()) << "taking must clear the pending slot";
}

TEST(PlanBuffer, newerPublishReplacesUnconsumedPlan) {
  PlanBuffer buffer;
  buffer.publish(makePlan(1, 0.20));
  buffer.publish(makePlan(2, 0.25));

  const auto taken = buffer.takePending();
  EXPECT_EQ(taken.phaseIndex, 2) << "the newest plan wins; stale plans are dropped";
}

TEST(PlanBuffer, currentIsIndependentOfPending) {
  PlanBuffer buffer;
  buffer.setCurrent(makePlan(5, 0.21));
  buffer.publish(makePlan(6, 0.24));

  EXPECT_EQ(buffer.current().phaseIndex, 5) << "publishing must not disturb the executing plan";
  EXPECT_TRUE(buffer.hasPending());
}

// The solver thread writes while the control thread reads; this must not tear or race.
TEST(PlanBuffer, concurrentPublishAndTakeIsSafe) {
  PlanBuffer buffer;
  std::atomic<bool> stop{false};

  std::thread writer([&] {
    for (int i = 0; i < 20000 && !stop; ++i) {
      buffer.publish(makePlan(i, 0.20 + 1e-6 * i));
    }
  });

  int taken = 0;
  for (int i = 0; i < 20000; ++i) {
    if (buffer.hasPending()) {
      const auto plan = buffer.takePending();
      ASSERT_EQ(plan.robotState.size(), opti_pessi::RobotX::DIM);
      ASSERT_EQ(plan.robotInput.size(), opti_pessi::RobotU::DIM);
      ASSERT_TRUE(plan.valid);

      // THE assertion that makes this a synchronization test rather than a crash test.
      // makePlan() ties dt to the index as 0.20 + 1e-6 * index, so a plan whose fields came from
      // two different publish() calls -- the way a torn, unlocked write would most plausibly
      // manifest -- fails here. Checking only sizes and `valid` would let that through.
      ASSERT_NEAR(plan.robotInput(opti_pessi::RobotU::DT), 0.20 + 1e-6 * plan.phaseIndex, 1e-12)
          << "phaseIndex " << plan.phaseIndex << " does not match its own dt: fields are torn";
      ++taken;
    }
  }
  stop = true;
  writer.join();

  EXPECT_GT(taken, 0) << "the reader should have observed at least one plan";
}
