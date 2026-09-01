#include <cmath>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "opti_pessi_interface/LipKinematics.h"
#include "opti_pessi_interface/definitions.h"
#include "opti_pessi_interface/initialization/OptiPessiInitializer.h"

using namespace opti_pessi;

namespace {

OptiPessiModelParameters testParameters() {
  OptiPessiModelParameters p;
  p.comHeight = 0.38;
  p.gravity = 9.81;
  p.mass = 24.24;
  p.inertia = 1.048;
  p.hipOffsets[static_cast<size_t>(Foot::FL)] = (vector_t(2) << 0.2407, 0.134).finished();
  p.hipOffsets[static_cast<size_t>(Foot::FR)] = (vector_t(2) << 0.2407, -0.134).finished();
  p.hipOffsets[static_cast<size_t>(Foot::RL)] = (vector_t(2) << -0.2407, 0.134).finished();
  p.hipOffsets[static_cast<size_t>(Foot::RR)] = (vector_t(2) << -0.2407, -0.134).finished();
  p.obstaclePositions = (matrix_t(1, 2) << 0.3, 0.8).finished();
  return p;
}

}  // namespace

TEST(LipStep, MatchesExactDiscretization) {
  const scalar_t h = 0.38;
  const scalar_t g = 9.81;
  const scalar_t w = std::sqrt(g / h);
  const scalar_t mass = 24.24;
  const scalar_t inertia = 1.048;
  const scalar_t dt = 0.25;

  vector_t x = vector_t::Zero(RobotX::DIM);
  x.head(10) << 0.1, -0.2, 0.3, 0.4, -0.05, 0.1, 0.0, 0.05, 0.2, -0.05;
  x.segment(RobotX::PP0X, 4) = x.segment(RobotX::P0X, 4);
  vector_t u(RobotU::DIM);
  u << 0.15, 0.02, 0.22, -0.03, 0.4, dt, 0.3, 0.7;

  const vector_t xn = lipMapScalar(x, u, w, mass, inertia);

  const scalar_t ch = std::cosh(w * dt);
  const scalar_t sh = std::sinh(w * dt);
  const scalar_t alpha = u(RobotU::ALPHA);
  const Eigen::Vector2d p0(x(RobotX::P0X), x(RobotX::P0Y));
  const Eigen::Vector2d p1(x(RobotX::P1X), x(RobotX::P1Y));
  const Eigen::Vector2d c(x(RobotX::CX), x(RobotX::CY));
  const Eigen::Vector2d dc(x(RobotX::DCX), x(RobotX::DCY));
  const Eigen::Vector2d cop = p0 + alpha * (p1 - p0);
  const Eigen::Vector2d ddc = w * w * (c - cop);
  const scalar_t f0x = u(RobotU::BETA) * mass * ddc(0);
  const scalar_t f0y = u(RobotU::GAMMA) * mass * ddc(1);
  const scalar_t f1x = (1.0 - u(RobotU::BETA)) * mass * ddc(0);
  const scalar_t f1y = (1.0 - u(RobotU::GAMMA)) * mass * ddc(1);
  const scalar_t tau = (p0(0) - c(0)) * f0y - (p0(1) - c(1)) * f0x + (p1(0) - c(0)) * f1y - (p1(1) - c(1)) * f1x;
  const Eigen::Vector2d cNext = ch * c + (sh / w) * dc + (1.0 - ch) * cop;
  const Eigen::Vector2d dcNext = (w * sh) * c + ch * dc - (w * sh) * cop;

  EXPECT_NEAR(xn(RobotX::CX), cNext(0), 1e-12);
  EXPECT_NEAR(xn(RobotX::CY), cNext(1), 1e-12);
  EXPECT_NEAR(xn(RobotX::TH), x(RobotX::TH) + dt * x(RobotX::DTH), 1e-12);
  EXPECT_NEAR(xn(RobotX::DCX), dcNext(0), 1e-12);
  EXPECT_NEAR(xn(RobotX::DCY), dcNext(1), 1e-12);
  EXPECT_NEAR(xn(RobotX::DTH), x(RobotX::DTH) + dt * tau / inertia, 1e-12);
  EXPECT_NEAR(xn(RobotX::P0X), u(RobotU::P0X), 1e-12);
  EXPECT_NEAR(xn(RobotX::P0Y), u(RobotU::P0Y), 1e-12);
  EXPECT_NEAR(xn(RobotX::P1X), u(RobotU::P1X), 1e-12);
  EXPECT_NEAR(xn(RobotX::P1Y), u(RobotU::P1Y), 1e-12);
  // The previous-foothold slots are a pure copy of this phase's stance feet.
  EXPECT_NEAR(xn(RobotX::PP0X), x(RobotX::P0X), 1e-12);
  EXPECT_NEAR(xn(RobotX::PP1Y), x(RobotX::P1Y), 1e-12);
}

TEST(LipStep, PureTranslationHasClosedForm) {
  const scalar_t w = std::sqrt(9.81 / 0.38);
  vector_t x = vector_t::Zero(RobotX::DIM);
  x(RobotX::DCX) = 1.0;
  vector_t u = vector_t::Zero(RobotU::DIM);
  u(RobotU::ALPHA) = 0.5;
  u(RobotU::DT) = 0.2;
  u(RobotU::BETA) = 0.5;
  u(RobotU::GAMMA) = 0.5;

  const vector_t xn = lipMapScalar(x, u, w, 24.24, 1.048);
  EXPECT_NEAR(xn(RobotX::CX), std::sinh(w * 0.2) / w, 1e-12);
  EXPECT_NEAR(xn(RobotX::DCX), std::cosh(w * 0.2), 1e-12);
  EXPECT_NEAR(xn(RobotX::CY), 0.0, 1e-12);
}

// The augmented step is what OCS2's EULER integrator with dt = 1.0 must reproduce, because
// systemFlowMap returns lipMap(x, u) - x. This pins that contract on the scalar side.
TEST(LipStep, AugmentedStepAdvancesBothBranchesAndClock) {
  const OptiPessiModelParameters params = testParameters();

  vector_t xOpti = vector_t::Zero(RobotX::DIM);
  xOpti.head(10) << 0.1, -0.2, 0.3, 0.4, -0.05, 0.1, 0.0, 0.05, 0.2, -0.05;
  vector_t xPessi = vector_t::Zero(RobotX::DIM);
  xPessi.head(10) << -0.1, 0.2, -0.3, 0.2, 0.05, -0.1, 0.05, 0.0, -0.05, 0.2;

  vector_t x = vector_t::Zero(AUG_STATE_DIM);
  x.segment(0, RobotX::DIM) = xOpti;
  x.segment(RobotX::DIM, RobotX::DIM) = xPessi;
  x(CLOCK_INDEX) = 0.7;

  vector_t u = vector_t::Zero(augInputDim(params.numObstacles()));
  u.segment(0, RobotU::DIM) << 0.15, 0.02, 0.22, -0.03, 0.4, 0.25, 0.3, 0.7;
  u.segment(RobotU::DIM, RobotU::DIM) << 0.10, 0.05, 0.18, 0.01, 0.6, 0.30, 0.4, 0.5;

  const vector_t xNext = augmentedLipStep(params, x, u);

  const vector_t expectedOpti = lipMapScalar(xOpti, u.head(RobotU::DIM), params.omega(), params.mass, params.inertia);
  const vector_t expectedPessi =
      lipMapScalar(xPessi, u.segment(RobotU::DIM, RobotU::DIM), params.omega(), params.mass, params.inertia);

  EXPECT_LT((xNext.segment(0, RobotX::DIM) - expectedOpti).norm(), 1e-12);
  EXPECT_LT((xNext.segment(RobotX::DIM, RobotX::DIM) - expectedPessi).norm(), 1e-12);
  // The clock integrates the PESSIMISTIC dt, which is what inflates the worst-case obstacle disk.
  EXPECT_NEAR(xNext(CLOCK_INDEX), 0.7 + 0.30, 1e-12);
}

TEST(LipStep, PackInitialStateDuplicatesBothBranches) {
  vector_t robotState = vector_t::Zero(RobotX::DIM);
  robotState.head(10) << 0.1, -0.2, 0.3, 0.4, -0.05, 0.1, 0.0, 0.05, 0.2, -0.05;

  const vector_t x = packInitialState(robotState);
  ASSERT_EQ(x.size(), AUG_STATE_DIM);
  EXPECT_LT((x.segment(0, RobotX::DIM) - robotState).norm(), 1e-15);
  EXPECT_LT((x.segment(RobotX::DIM, RobotX::DIM) - robotState).norm(), 1e-15);
  EXPECT_NEAR(x(CLOCK_INDEX), 0.0, 1e-15);
  EXPECT_LT((extractRobotState(x) - robotState).norm(), 1e-15);
}

TEST(LipStep, GaitAlternatesDiagonalPairs) {
  EXPECT_EQ(gaitPair(0)[0], Foot::FR);
  EXPECT_EQ(gaitPair(0)[1], Foot::RL);
  EXPECT_EQ(gaitPair(1)[0], Foot::FL);
  EXPECT_EQ(gaitPair(1)[1], Foot::RR);
  EXPECT_EQ(gaitPair(2)[0], Foot::FR);
  EXPECT_TRUE(isLeft(Foot::FL));
  EXPECT_TRUE(isLeft(Foot::RL));
  EXPECT_FALSE(isLeft(Foot::FR));
  EXPECT_FALSE(isLeft(Foot::RR));
}
