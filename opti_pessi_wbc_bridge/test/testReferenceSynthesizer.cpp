#include <algorithm>
#include <cmath>

#include <gtest/gtest.h>

#include "opti_pessi_interface/LipKinematics.h"
#include "opti_pessi_wbc_bridge/LegIndexing.h"
#include "opti_pessi_wbc_bridge/ReferenceSynthesizer.h"

using namespace opti_pessi_bridge;

namespace {
SynthesisSettings testSettings() {
  SynthesisSettings settings;
  settings.comHeight = 0.38;
  settings.mass = 24.24;
  settings.inertia = 1.048;
  settings.gravity = 9.81;
  settings.swingHeight = 0.1;
  return settings;
}

double omegaOf(const SynthesisSettings& s) {
  return std::sqrt(s.gravity / s.comHeight);
}

PhasePlan testPlan(double dtValue) {
  PhasePlan plan;
  plan.robotState = ocs2::vector_t::Zero(opti_pessi::RobotX::DIM);
  plan.robotState(opti_pessi::RobotX::CX) = 0.10;
  plan.robotState(opti_pessi::RobotX::CY) = 0.02;
  plan.robotState(opti_pessi::RobotX::TH) = 0.05;
  plan.robotState(opti_pessi::RobotX::DCX) = 0.40;
  plan.robotState(opti_pessi::RobotX::DCY) = -0.05;
  plan.robotState(opti_pessi::RobotX::DTH) = 0.10;
  plan.robotState(opti_pessi::RobotX::P0X) = 0.24;
  plan.robotState(opti_pessi::RobotX::P0Y) = -0.13;
  plan.robotState(opti_pessi::RobotX::P1X) = -0.24;
  plan.robotState(opti_pessi::RobotX::P1Y) = 0.13;

  // Previous-phase footholds: where the SWINGING pair stood last phase, i.e. where their arc
  // starts. These are deliberately distinct from both the current stance feet (P0/P1 above) and
  // the landing targets (RobotU::P0/P1 below), so that a take-off sourced from the wrong one is
  // detectable rather than coincidentally equal. A 0.24 m step to the landing target keeps the
  // per-sample displacement in swingPathIsContinuous comfortably under its 0.02 bound.
  plan.robotState(opti_pessi::RobotX::PP0X) = 0.20;
  plan.robotState(opti_pessi::RobotX::PP0Y) = 0.13;
  plan.robotState(opti_pessi::RobotX::PP1X) = -0.28;
  plan.robotState(opti_pessi::RobotX::PP1Y) = -0.13;

  plan.robotInput = ocs2::vector_t::Zero(opti_pessi::RobotU::DIM);
  plan.robotInput(opti_pessi::RobotU::P0X) = 0.44;
  plan.robotInput(opti_pessi::RobotU::P0Y) = 0.13;
  plan.robotInput(opti_pessi::RobotU::P1X) = -0.04;
  plan.robotInput(opti_pessi::RobotU::P1Y) = -0.13;
  plan.robotInput(opti_pessi::RobotU::ALPHA) = 0.5;
  plan.robotInput(opti_pessi::RobotU::DT) = dtValue;
  plan.robotInput(opti_pessi::RobotU::BETA) = 0.5;
  plan.robotInput(opti_pessi::RobotU::GAMMA) = 0.5;

  plan.phaseIndex = 0;
  plan.valid = true;
  return plan;
}
}  // namespace

// At tau = 0 the synthesized state must be the phase's own start state.
TEST(ReferenceSynthesizer, tauZeroReproducesTheKnotState) {
  const auto settings = testSettings();
  const auto plan = testPlan(0.25);

  const auto x = lipStateAt(plan, 0.0, settings);

  EXPECT_NEAR(x(opti_pessi::RobotX::CX), plan.robotState(opti_pessi::RobotX::CX), 1e-12);
  EXPECT_NEAR(x(opti_pessi::RobotX::CY), plan.robotState(opti_pessi::RobotX::CY), 1e-12);
  EXPECT_NEAR(x(opti_pessi::RobotX::TH), plan.robotState(opti_pessi::RobotX::TH), 1e-12);
  EXPECT_NEAR(x(opti_pessi::RobotX::DCX), plan.robotState(opti_pessi::RobotX::DCX), 1e-12);
  EXPECT_NEAR(x(opti_pessi::RobotX::DCY), plan.robotState(opti_pessi::RobotX::DCY), 1e-12);
}

// At tau = dt it must agree exactly with the plan's own discrete step.
TEST(ReferenceSynthesizer, tauEqualsDtReproducesLipMap) {
  const auto settings = testSettings();
  const double dtValue = 0.25;
  const auto plan = testPlan(dtValue);

  const auto x = lipStateAt(plan, dtValue, settings);
  const auto expected = opti_pessi::lipMapScalar(plan.robotState, plan.robotInput, omegaOf(settings),
                                                 settings.mass, settings.inertia);

  EXPECT_NEAR(x(opti_pessi::RobotX::CX), expected(opti_pessi::RobotX::CX), 1e-12);
  EXPECT_NEAR(x(opti_pessi::RobotX::CY), expected(opti_pessi::RobotX::CY), 1e-12);
  EXPECT_NEAR(x(opti_pessi::RobotX::TH), expected(opti_pessi::RobotX::TH), 1e-12);
  EXPECT_NEAR(x(opti_pessi::RobotX::DCX), expected(opti_pessi::RobotX::DCX), 1e-12);
  EXPECT_NEAR(x(opti_pessi::RobotX::DCY), expected(opti_pessi::RobotX::DCY), 1e-12);
  EXPECT_NEAR(x(opti_pessi::RobotX::DTH), expected(opti_pessi::RobotX::DTH), 1e-12);
}

// The CoM path must be continuous and monotone in tau, with no jumps.
TEST(ReferenceSynthesizer, comPathIsContinuousInTau) {
  const auto settings = testSettings();
  const double dtValue = 0.25;
  const auto plan = testPlan(dtValue);

  const int steps = 250;
  Eigen::Vector2d previous(plan.robotState(opti_pessi::RobotX::CX), plan.robotState(opti_pessi::RobotX::CY));
  for (int i = 1; i <= steps; ++i) {
    const double tau = dtValue * static_cast<double>(i) / steps;
    const auto x = lipStateAt(plan, tau, settings);
    const Eigen::Vector2d c(x(opti_pessi::RobotX::CX), x(opti_pessi::RobotX::CY));

    EXPECT_LT((c - previous).norm(), 0.02) << "step " << i << " jumped";
    previous = c;
  }
}

// Velocity must be the time derivative of position, to finite-difference accuracy.
TEST(ReferenceSynthesizer, velocityIsTheDerivativeOfPosition) {
  const auto settings = testSettings();
  const auto plan = testPlan(0.25);
  const double tau = 0.12;
  const double h = 1e-6;

  const auto xPlus = lipStateAt(plan, tau + h, settings);
  const auto xMinus = lipStateAt(plan, tau - h, settings);
  const auto x = lipStateAt(plan, tau, settings);

  const double numericalDcx =
      (xPlus(opti_pessi::RobotX::CX) - xMinus(opti_pessi::RobotX::CX)) / (2.0 * h);
  const double numericalDcy =
      (xPlus(opti_pessi::RobotX::CY) - xMinus(opti_pessi::RobotX::CY)) / (2.0 * h);

  EXPECT_NEAR(x(opti_pessi::RobotX::DCX), numericalDcx, 1e-6);
  EXPECT_NEAR(x(opti_pessi::RobotX::DCY), numericalDcy, 1e-6);
}

// Stance feet stay exactly where the plan says, for the whole phase.
TEST(FootReference, stanceFeetAreHeldFixed) {
  const auto settings = testSettings();
  const double dtValue = 0.25;
  const auto plan = testPlan(dtValue);
  const int parity = 0;
  const auto stancePair = opti_pessi::gaitPair(parity);

  for (double tau = 0.0; tau <= dtValue; tau += dtValue / 20.0) {
    const auto feet = footReferenceAt(plan, tau, parity, settings);

    for (int k = 0; k < 2; ++k) {
      const size_t contact = contactIndexOf(stancePair[k]);
      EXPECT_TRUE(feet.inContact[contact]);
      const double expectedX = (k == 0) ? plan.robotState(opti_pessi::RobotX::P0X)
                                        : plan.robotState(opti_pessi::RobotX::P1X);
      const double expectedY = (k == 0) ? plan.robotState(opti_pessi::RobotX::P0Y)
                                        : plan.robotState(opti_pessi::RobotX::P1Y);
      EXPECT_NEAR(feet.position[contact].x(), expectedX, 1e-12);
      EXPECT_NEAR(feet.position[contact].y(), expectedY, 1e-12);
      EXPECT_NEAR(feet.position[contact].z(), 0.0, 1e-12);
      EXPECT_NEAR(feet.velocity[contact].norm(), 0.0, 1e-12);
    }
  }
}

// Swing feet must land exactly on the next footholds the plan commands.
TEST(FootReference, swingFeetLandOnTheCommandedFootholds) {
  const auto settings = testSettings();
  const double dtValue = 0.25;
  const auto plan = testPlan(dtValue);
  const int parity = 0;
  const auto swingPair = opti_pessi::gaitPair(parity + 1);

  const auto feet = footReferenceAt(plan, dtValue, parity, settings);

  const size_t c0 = contactIndexOf(swingPair[0]);
  const size_t c1 = contactIndexOf(swingPair[1]);

  EXPECT_FALSE(feet.inContact[c0]);
  EXPECT_FALSE(feet.inContact[c1]);
  EXPECT_NEAR(feet.position[c0].x(), plan.robotInput(opti_pessi::RobotU::P0X), 1e-9);
  EXPECT_NEAR(feet.position[c0].y(), plan.robotInput(opti_pessi::RobotU::P0Y), 1e-9);
  EXPECT_NEAR(feet.position[c1].x(), plan.robotInput(opti_pessi::RobotU::P1X), 1e-9);
  EXPECT_NEAR(feet.position[c1].y(), plan.robotInput(opti_pessi::RobotU::P1Y), 1e-9);
  EXPECT_NEAR(feet.position[c0].z(), 0.0, 1e-9) << "must touch down at ground height";
}

// Swing feet must BEGIN their arc at the previous phase's footholds, not at the current stance
// feet. This is the highest-risk substitution in this task -- sourcing take-off from P0/P1 instead
// of PP0/PP1 would make every foot teleport at each phase boundary.
//
// Note why the landing test cannot cover this: it evaluates at tau = dt, where smoothStep(1) = 1
// collapses the position to `land` regardless of where the arc started. Only tau = 0 discriminates.
TEST(FootReference, swingFeetTakeOffFromThePreviousFootholds) {
  const auto settings = testSettings();
  const double dtValue = 0.25;
  const auto plan = testPlan(dtValue);
  const int parity = 0;
  const auto swingPair = opti_pessi::gaitPair(parity + 1);

  const auto feet = footReferenceAt(plan, 0.0, parity, settings);

  const size_t c0 = contactIndexOf(swingPair[0]);
  const size_t c1 = contactIndexOf(swingPair[1]);

  EXPECT_NEAR(feet.position[c0].x(), plan.robotState(opti_pessi::RobotX::PP0X), 1e-12);
  EXPECT_NEAR(feet.position[c0].y(), plan.robotState(opti_pessi::RobotX::PP0Y), 1e-12);
  EXPECT_NEAR(feet.position[c1].x(), plan.robotState(opti_pessi::RobotX::PP1X), 1e-12);
  EXPECT_NEAR(feet.position[c1].y(), plan.robotState(opti_pessi::RobotX::PP1Y), 1e-12);

  // And explicitly NOT the current stance feet, so the test's purpose survives fixture edits.
  EXPECT_GT(std::abs(feet.position[c0].x() - plan.robotState(opti_pessi::RobotX::P0X)), 1e-6)
      << "swing foot took off from the current stance foot, not the previous foothold";
}

// The reported swing velocity must be the true time derivative of the reported position, including
// the 1/dt chain-rule factor. Without this, dropping the "/ dtValue" would make every swing
// velocity wrong by a factor of dt (~4x here) while still vanishing at both endpoints -- so every
// other test in this file would still pass.
TEST(FootReference, swingVelocityIsTheTimeDerivativeOfSwingPosition) {
  const auto settings = testSettings();
  const double dtValue = 0.25;
  const auto plan = testPlan(dtValue);
  const int parity = 0;
  const size_t contact = contactIndexOf(opti_pessi::gaitPair(parity + 1)[0]);

  // An interior sample where both the xy and z derivatives are non-zero.
  const double tau = 0.25 * dtValue;
  const double h = 1e-7;

  const auto plus = footReferenceAt(plan, tau + h, parity, settings);
  const auto minus = footReferenceAt(plan, tau - h, parity, settings);
  const auto at = footReferenceAt(plan, tau, parity, settings);

  const Eigen::Vector3d numerical = (plus.position[contact] - minus.position[contact]) / (2.0 * h);

  EXPECT_NEAR(at.velocity[contact].x(), numerical.x(), 1e-5);
  EXPECT_NEAR(at.velocity[contact].y(), numerical.y(), 1e-5);
  EXPECT_NEAR(at.velocity[contact].z(), numerical.z(), 1e-5);

  // Guard against a vacuous pass: the derivative must actually be non-zero at this sample.
  EXPECT_GT(numerical.norm(), 1e-3) << "sample point is not exercising a moving foot";
}

// Touchdown and lift-off must be soft: zero vertical velocity at both ends.
TEST(FootReference, swingVelocityVanishesAtBothEnds) {
  const auto settings = testSettings();
  const double dtValue = 0.25;
  const auto plan = testPlan(dtValue);
  const int parity = 0;
  const size_t contact = contactIndexOf(opti_pessi::gaitPair(parity + 1)[0]);

  const auto atStart = footReferenceAt(plan, 0.0, parity, settings);
  const auto atEnd = footReferenceAt(plan, dtValue, parity, settings);

  EXPECT_NEAR(atStart.velocity[contact].z(), 0.0, 1e-9);
  EXPECT_NEAR(atEnd.velocity[contact].z(), 0.0, 1e-9);
}

// The swing foot must actually clear the ground, peaking near swingHeight at mid-phase.
TEST(FootReference, swingFootClearsTheGround) {
  const auto settings = testSettings();
  const double dtValue = 0.25;
  const auto plan = testPlan(dtValue);
  const int parity = 0;
  const size_t contact = contactIndexOf(opti_pessi::gaitPair(parity + 1)[0]);

  const auto midPhase = footReferenceAt(plan, dtValue / 2.0, parity, settings);
  EXPECT_NEAR(midPhase.position[contact].z(), settings.swingHeight, 1e-9);

  for (double tau = 0.0; tau <= dtValue; tau += dtValue / 50.0) {
    const auto feet = footReferenceAt(plan, tau, parity, settings);
    EXPECT_GE(feet.position[contact].z(), -1e-12) << "swing foot must never go below ground";
  }
}

// Exactly two feet in contact at all times, matching the mode number.
TEST(FootReference, exactlyTwoFeetAreInContact) {
  const auto settings = testSettings();
  const auto plan = testPlan(0.25);

  for (int parity = 0; parity < 2; ++parity) {
    const auto feet = footReferenceAt(plan, 0.1, parity, settings);
    const int count = std::count(feet.inContact.begin(), feet.inContact.end(), true);
    EXPECT_EQ(count, 2) << "parity " << parity;
  }
}

// The swing path must be continuous in tau.
TEST(FootReference, swingPathIsContinuous) {
  const auto settings = testSettings();
  const double dtValue = 0.25;
  const auto plan = testPlan(dtValue);
  const int parity = 0;
  const size_t contact = contactIndexOf(opti_pessi::gaitPair(parity + 1)[0]);

  Eigen::Vector3d previous = footReferenceAt(plan, 0.0, parity, settings).position[contact];
  for (int i = 1; i <= 200; ++i) {
    const double tau = dtValue * static_cast<double>(i) / 200.0;
    const Eigen::Vector3d p = footReferenceAt(plan, tau, parity, settings).position[contact];
    EXPECT_LT((p - previous).norm(), 0.02) << "step " << i;
    previous = p;
  }
}

// Total vertical force must carry the robot's weight exactly: the LIP has no vertical dynamics.
TEST(ContactForces, verticalForcesSumToWeight) {
  const auto settings = testSettings();
  const auto plan = testPlan(0.25);

  for (double tau = 0.0; tau <= 0.25; tau += 0.05) {
    const auto forces = contactForcesAt(plan, tau, 0, settings);
    double totalZ = 0.0;
    for (const auto& f : forces) {
      totalZ += f.z();
    }
    EXPECT_NEAR(totalZ, settings.mass * settings.gravity, 1e-9) << "tau " << tau;
  }
}

// Horizontal forces must equal m * ddc, with ddc from the LIP law at the CURRENT CoM.
TEST(ContactForces, horizontalForcesMatchLipAcceleration) {
  const auto settings = testSettings();
  const auto plan = testPlan(0.25);
  const double tau = 0.1;
  const double omega = omegaOf(settings);

  const auto x = lipStateAt(plan, tau, settings);
  const Eigen::Vector2d c(x(opti_pessi::RobotX::CX), x(opti_pessi::RobotX::CY));
  const Eigen::Vector2d p0(plan.robotState(opti_pessi::RobotX::P0X), plan.robotState(opti_pessi::RobotX::P0Y));
  const Eigen::Vector2d p1(plan.robotState(opti_pessi::RobotX::P1X), plan.robotState(opti_pessi::RobotX::P1Y));
  const double alpha = plan.robotInput(opti_pessi::RobotU::ALPHA);
  const Eigen::Vector2d cop = p0 + alpha * (p1 - p0);
  const Eigen::Vector2d expected = settings.mass * omega * omega * (c - cop);

  const auto forces = contactForcesAt(plan, tau, 0, settings);
  Eigen::Vector2d total = Eigen::Vector2d::Zero();
  for (const auto& f : forces) {
    total += f.head<2>();
  }

  EXPECT_NEAR(total.x(), expected.x(), 1e-9);
  EXPECT_NEAR(total.y(), expected.y(), 1e-9);
}

// Swing legs carry no force at all -- legged_wbc's ZeroForce expectations depend on this.
TEST(ContactForces, swingLegsCarryZeroForce) {
  const auto settings = testSettings();
  const auto plan = testPlan(0.25);

  for (int parity = 0; parity < 2; ++parity) {
    const auto forces = contactForcesAt(plan, 0.1, parity, settings);
    const auto swingPair = opti_pessi::gaitPair(parity + 1);
    for (const auto foot : swingPair) {
      EXPECT_NEAR(forces[contactIndexOf(foot)].norm(), 0.0, 1e-12) << "parity " << parity;
    }
  }
}

// The centre of pressure implied by the synthesized vertical forces must be the plan's own CoP.
//
// Deliberately overrides ALPHA away from the shared fixture's 0.5: at alpha = 0.5 the correct split
// fz0 = (1-alpha)*w and an INVERTED fz0 = alpha*w produce the identical value (0.5*w either way), so
// the symmetric fixture value cannot distinguish a correct split from a swapped one. An asymmetric
// alpha makes fz0 != fz1, so a swapped assignment yields a visibly wrong CoP.
TEST(ContactForces, verticalSplitReproducesTheCommandedCop) {
  const auto settings = testSettings();
  auto plan = testPlan(0.25);
  plan.robotInput(opti_pessi::RobotU::ALPHA) = 0.2;
  const int parity = 0;
  const auto stancePair = opti_pessi::gaitPair(parity);

  const double alpha = plan.robotInput(opti_pessi::RobotU::ALPHA);
  const Eigen::Vector2d p0(plan.robotState(opti_pessi::RobotX::P0X), plan.robotState(opti_pessi::RobotX::P0Y));
  const Eigen::Vector2d p1(plan.robotState(opti_pessi::RobotX::P1X), plan.robotState(opti_pessi::RobotX::P1Y));
  const Eigen::Vector2d expectedCop = p0 + alpha * (p1 - p0);

  const auto forces = contactForcesAt(plan, 0.1, parity, settings);
  const double fz0 = forces[contactIndexOf(stancePair[0])].z();
  const double fz1 = forces[contactIndexOf(stancePair[1])].z();
  ASSERT_GT(fz0 + fz1, 1e-9);

  const Eigen::Vector2d cop = (fz0 * p0 + fz1 * p1) / (fz0 + fz1);
  EXPECT_NEAR(cop.x(), expectedCop.x(), 1e-9);
  EXPECT_NEAR(cop.y(), expectedCop.y(), 1e-9);
}

// Both feet must stay loaded: alphaReduction exists precisely to prevent a zero-load foot.
TEST(ContactForces, bothStanceFeetStayLoaded) {
  const auto settings = testSettings();
  const auto plan = testPlan(0.25);
  const auto stancePair = opti_pessi::gaitPair(0);

  const auto forces = contactForcesAt(plan, 0.1, 0, settings);
  EXPECT_GT(forces[contactIndexOf(stancePair[0])].z(), 0.0);
  EXPECT_GT(forces[contactIndexOf(stancePair[1])].z(), 0.0);
}

TEST(Synthesize, producesCorrectlySizedVectors) {
  const auto geom = aliengoLegGeometry();
  const auto settings = testSettings();
  const auto plan = testPlan(0.25);

  const auto reference = synthesize(geom, plan, 0.1, 0, settings);

  EXPECT_EQ(reference.state.size(), 24);
  EXPECT_EQ(reference.input.size(), 24);
}

TEST(Synthesize, modeMatchesParity) {
  const auto geom = aliengoLegGeometry();
  const auto settings = testSettings();
  const auto plan = testPlan(0.25);

  EXPECT_EQ(synthesize(geom, plan, 0.1, 0, settings).mode, 6u);
  EXPECT_EQ(synthesize(geom, plan, 0.1, 1, settings).mode, 9u);
}

TEST(Synthesize, basePoseUsesLipPoseAtFixedHeightAndLevelAttitude) {
  const auto geom = aliengoLegGeometry();
  const auto settings = testSettings();
  const auto plan = testPlan(0.25);
  const double tau = 0.1;

  const auto reference = synthesize(geom, plan, tau, 0, settings);
  const auto lip = lipStateAt(plan, tau, settings);

  EXPECT_NEAR(reference.state(6), lip(opti_pessi::RobotX::CX), 1e-12);
  EXPECT_NEAR(reference.state(7), lip(opti_pessi::RobotX::CY), 1e-12);
  EXPECT_NEAR(reference.state(8), settings.comHeight, 1e-12);
  EXPECT_NEAR(reference.state(9), lip(opti_pessi::RobotX::TH), 1e-12) << "yaw";
  EXPECT_NEAR(reference.state(10), 0.0, 1e-12) << "pitch must be level";
  EXPECT_NEAR(reference.state(11), 0.0, 1e-12) << "roll must be level";
}

TEST(Synthesize, normalizedMomentumCarriesComVelocity) {
  const auto geom = aliengoLegGeometry();
  const auto settings = testSettings();
  const auto plan = testPlan(0.25);
  const double tau = 0.1;

  const auto reference = synthesize(geom, plan, tau, 0, settings);
  const auto lip = lipStateAt(plan, tau, settings);

  EXPECT_NEAR(reference.state(0), lip(opti_pessi::RobotX::DCX), 1e-12);
  EXPECT_NEAR(reference.state(1), lip(opti_pessi::RobotX::DCY), 1e-12);
  EXPECT_NEAR(reference.state(2), 0.0, 1e-12) << "no vertical CoM motion in the LIP";
  EXPECT_NEAR(reference.state(5), settings.inertia * lip(opti_pessi::RobotX::DTH) / settings.mass, 1e-12);
}

// THE central consistency check: FK of the synthesized joint angles must reproduce the foot targets.
// This is the only path by which foot targets reach legged_wbc.
TEST(Synthesize, forwardKinematicsOfJointAnglesReproducesFootTargets) {
  const auto geom = aliengoLegGeometry();
  const auto settings = testSettings();
  const auto plan = testPlan(0.25);
  const double tau = 0.1;
  const int parity = 0;

  const auto reference = synthesize(geom, plan, tau, parity, settings);
  ASSERT_TRUE(reference.allFeetReachable);

  const auto feet = footReferenceAt(plan, tau, parity, settings);
  const auto lip = lipStateAt(plan, tau, settings);
  const double yaw = lip(opti_pessi::RobotX::TH);
  const Eigen::Vector2d base(lip(opti_pessi::RobotX::CX), lip(opti_pessi::RobotX::CY));

  for (int foot = 0; foot < 4; ++foot) {
    const opti_pessi::Foot f = static_cast<opti_pessi::Foot>(foot);
    const size_t block = jointBlockOf(f);
    const size_t contact = contactIndexOf(f);

    const Eigen::Vector3d q(reference.state(12 + 3 * block + 0),
                            reference.state(12 + 3 * block + 1),
                            reference.state(12 + 3 * block + 2));
    const Eigen::Vector3d footInBase = forwardKinematics(geom, block, q);

    // Rotate back into the world frame and add the base translation.
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    const Eigen::Vector3d footWorld(base.x() + c * footInBase.x() - s * footInBase.y(),
                                    base.y() + s * footInBase.x() + c * footInBase.y(),
                                    settings.comHeight + footInBase.z());

    EXPECT_NEAR(footWorld.x(), feet.position[contact].x(), 1e-8) << "contact " << contact;
    EXPECT_NEAR(footWorld.y(), feet.position[contact].y(), 1e-8) << "contact " << contact;
    EXPECT_NEAR(footWorld.z(), feet.position[contact].z(), 1e-8) << "contact " << contact;
  }
}

// Contact forces must land in the input's force blocks, in contact order.
TEST(Synthesize, inputCarriesForcesThenJointVelocities) {
  const auto geom = aliengoLegGeometry();
  const auto settings = testSettings();
  const auto plan = testPlan(0.25);
  const double tau = 0.1;

  const auto reference = synthesize(geom, plan, tau, 0, settings);
  const auto forces = contactForcesAt(plan, tau, 0, settings);

  for (size_t contact = 0; contact < 4; ++contact) {
    EXPECT_NEAR(reference.input(3 * contact + 0), forces[contact].x(), 1e-12);
    EXPECT_NEAR(reference.input(3 * contact + 1), forces[contact].y(), 1e-12);
    EXPECT_NEAR(reference.input(3 * contact + 2), forces[contact].z(), 1e-12);
  }
}

// Joint velocities must be consistent with a finite difference of the joint angles.
TEST(Synthesize, jointVelocitiesMatchFiniteDifferenceOfAngles) {
  const auto geom = aliengoLegGeometry();
  const auto settings = testSettings();
  const auto plan = testPlan(0.25);
  const double tau = 0.1;
  const double h = 1e-6;

  const auto reference = synthesize(geom, plan, tau, 0, settings);
  const auto plus = synthesize(geom, plan, tau + h, 0, settings);
  const auto minus = synthesize(geom, plan, tau - h, 0, settings);

  for (int j = 0; j < 12; ++j) {
    const double numerical = (plus.state(12 + j) - minus.state(12 + j)) / (2.0 * h);
    EXPECT_NEAR(reference.input(12 + j), numerical, 1e-4) << "joint " << j;
  }
}

// The reference must be continuous across a phase boundary, otherwise the WBC sees a step.
TEST(Synthesize, referenceIsContinuousAcrossThePhaseBoundary) {
  const auto geom = aliengoLegGeometry();
  const auto settings = testSettings();
  const double dtValue = 0.25;
  const auto plan = testPlan(dtValue);

  // End of phase 0.
  const auto atEnd = synthesize(geom, plan, dtValue, 0, settings);

  // Start of phase 1: the successor state becomes the new plan's start, parity flips, and the feet
  // that were swinging are now standing.
  PhasePlan next = plan;
  next.robotState = opti_pessi::lipMapScalar(plan.robotState, plan.robotInput,
                                             std::sqrt(settings.gravity / settings.comHeight),
                                             settings.mass, settings.inertia);
  next.phaseIndex = 1;
  const auto atStart = synthesize(geom, next, 0.0, 1, settings);

  EXPECT_NEAR(atEnd.state(6), atStart.state(6), 1e-9) << "base x";
  EXPECT_NEAR(atEnd.state(7), atStart.state(7), 1e-9) << "base y";
  EXPECT_NEAR(atEnd.state(9), atStart.state(9), 1e-9) << "yaw";
  for (int j = 0; j < 12; ++j) {
    EXPECT_NEAR(atEnd.state(12 + j), atStart.state(12 + j), 1e-6) << "joint " << j;
  }
}
