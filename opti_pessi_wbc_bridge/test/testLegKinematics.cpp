#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "opti_pessi_wbc_bridge/LegKinematics.h"

using namespace opti_pessi_bridge;

namespace {
constexpr double kHaaMin = -70.0 * M_PI / 180.0;
constexpr double kHaaMax = 70.0 * M_PI / 180.0;
constexpr double kHfeMin = -1.0;
constexpr double kHfeMax = 1.5;
constexpr double kKfeMin = -159.0 * M_PI / 180.0;
constexpr double kKfeMax = -37.0 * M_PI / 180.0;

// --- IK DOMAIN -------------------------------------------------------------------------------
// forwardKinematics is NOT injective over the full joint box: the HAA equation
//     cos(q1)*py + sin(q1)*pz = d
// has two solutions, selected by whether the foot lies below or above the hip in the leg plane.
// Configurations with the knee folded up (e.g. q2 = -1.0, q3 = -2.775 gives planarZ = +0.066) put
// the foot ABOVE the hip, and there a second valid configuration reaches the identical foot
// position -- so no function of foot position alone can recover which one was sampled.
//
// inverseKinematics is therefore DEFINED ONLY for foot targets below the hip. That is the sole
// regime this robot operates in: the base rides at comHeight = 0.38 m and feet range over
// z in [0, 0.1], so the foot sits ~0.3 m below the hip at all times, far from the boundary.
//
// The domain test below is a function of the SAMPLED joints only, needing no IK:
//     planarZ = -l2*cos(q2) - l3*cos(q2 + q3)   must be < -kDomainMargin
constexpr double kDomainMargin = 0.05;  // 5 cm clear of the degenerate planarZ = 0 boundary

bool isInIkDomain(const LegGeometry& geom, const Eigen::Vector3d& q) {
  const double planarZ = -geom.thighLength * std::cos(q.y()) - geom.calfLength * std::cos(q.y() + q.z());
  return planarZ < -kDomainMargin;
}
}  // namespace

// The nominal standing configuration from reference.info must produce a foot below and outboard of
// its hip, at a sane standing height.
TEST(LegKinematics, nominalStanceIsBelowTheHip) {
  const auto geom = aliengoLegGeometry();
  const Eigen::Vector3d qNominal(-0.10, 0.62, -1.24);  // LF from defaultJointState

  const Eigen::Vector3d foot = forwardKinematics(geom, 0, qNominal);

  EXPECT_LT(foot.z(), -0.2) << "foot should be well below the base origin";
  EXPECT_GT(foot.z(), -0.6) << "foot should not be absurdly far below the base";
  EXPECT_GT(foot.x(), 0.0) << "LF foot should be forward of the base origin";
  EXPECT_GT(foot.y(), 0.0) << "LF foot should be to the left of the base origin";
}

// The real gate: within its declared domain, IK inverts FK exactly, for every leg.
TEST(LegKinematics, inverseInvertsForwardInDeclaredDomain) {
  const auto geom = aliengoLegGeometry();
  std::mt19937 rng(42);
  std::uniform_real_distribution<double> haa(kHaaMin, kHaaMax);
  std::uniform_real_distribution<double> hfe(kHfeMin, kHfeMax);
  std::uniform_real_distribution<double> kfe(kKfeMin, kKfeMax);

  for (size_t block = 0; block < kNumLegs; ++block) {
    int checked = 0;
    for (int trial = 0; trial < 2000 && checked < 500; ++trial) {
      const Eigen::Vector3d q(haa(rng), hfe(rng), kfe(rng));
      if (!isInIkDomain(geom, q)) {
        continue;  // outside the IK's contract; covered by the out-of-domain test below
      }
      ++checked;

      const Eigen::Vector3d foot = forwardKinematics(geom, block, q);

      bool reachable = false;
      const Eigen::Vector3d qBack = inverseKinematics(geom, block, foot, &reachable);

      ASSERT_TRUE(reachable) << "block " << block << " q " << q.transpose();
      EXPECT_NEAR(qBack.x(), q.x(), 1e-9) << "block " << block << " trial " << trial;
      EXPECT_NEAR(qBack.y(), q.y(), 1e-9) << "block " << block << " trial " << trial;
      EXPECT_NEAR(qBack.z(), q.z(), 1e-9) << "block " << block << " trial " << trial;
    }
    ASSERT_GE(checked, 500) << "block " << block << ": too few in-domain samples to be a real test";
  }
}

// THE load-bearing property for the bridge: whatever joints IK returns, they must put the foot
// exactly where it was asked to. This must hold across the actual walking workspace, and it holds
// regardless of the branch ambiguity above.
TEST(LegKinematics, forwardKinematicsOfIkReachesTheTargetAcrossTheWalkingWorkspace) {
  const auto geom = aliengoLegGeometry();
  std::mt19937 rng(7);
  // Foot targets in the base frame over the region the trot actually visits: within +-0.25 m of
  // the hip laterally and longitudinally, and 0.28-0.45 m below the base.
  //
  // The lower bound spans BOTH comHeight conventions in this repo: opti_pessi_interface's
  // task.info uses 0.38, while legged_controllers/config/aliengo/reference.info uses 0.4, whose
  // defaultJointState puts the LF foot at z = -0.4135. A -0.40 floor would leave the controller's
  // own nominal standing pose untested.
  std::uniform_real_distribution<double> dx(-0.25, 0.25);
  std::uniform_real_distribution<double> dy(-0.25, 0.25);
  std::uniform_real_distribution<double> dz(-0.45, -0.28);

  for (size_t block = 0; block < kNumLegs; ++block) {
    int checked = 0;
    for (int trial = 0; trial < 2000 && checked < 300; ++trial) {
      const Eigen::Vector3d target = geom.hipPositionInBase[block] + Eigen::Vector3d(dx(rng), dy(rng), dz(rng));

      bool reachable = false;
      const Eigen::Vector3d q = inverseKinematics(geom, block, target, &reachable);
      if (!reachable) {
        continue;  // outside the leg's reach; the unreachable test covers reporting
      }
      ++checked;

      const Eigen::Vector3d foot = forwardKinematics(geom, block, q);
      EXPECT_NEAR((foot - target).norm(), 0.0, 1e-9) << "block " << block << " target " << target.transpose();
    }
    ASSERT_GE(checked, 300) << "block " << block << ": too few reachable targets to be a real test";
  }
}

// Out-of-domain targets (foot above the hip) must still land the foot on the target -- IK returns
// the below-hip branch, which is a different configuration than any that produced the target from
// above. Position correctness is guaranteed; joint recovery explicitly is not.
TEST(LegKinematics, aboveHipTargetsStillReachThePositionViaTheBelowHipBranch) {
  const auto geom = aliengoLegGeometry();
  const Eigen::Vector3d qFoldedUp(0.2, -1.0, -2.775);
  ASSERT_FALSE(isInIkDomain(geom, qFoldedUp)) << "this fixture must be outside the declared domain";

  const Eigen::Vector3d target = forwardKinematics(geom, 0, qFoldedUp);

  bool reachable = false;
  const Eigen::Vector3d q = inverseKinematics(geom, 0, target, &reachable);

  ASSERT_TRUE(reachable);
  const Eigen::Vector3d foot = forwardKinematics(geom, 0, q);
  EXPECT_NEAR((foot - target).norm(), 0.0, 1e-9) << "position must still be reached exactly";
}

// Round-tripping the other way: IK then FK reproduces the requested foot position.
TEST(LegKinematics, forwardInvertsInverse) {
  const auto geom = aliengoLegGeometry();
  const Eigen::Vector3d target(0.25, 0.15, -0.35);

  bool reachable = false;
  const Eigen::Vector3d q = inverseKinematics(geom, 0, target, &reachable);
  ASSERT_TRUE(reachable);

  const Eigen::Vector3d foot = forwardKinematics(geom, 0, q);
  EXPECT_NEAR((foot - target).norm(), 0.0, 1e-9);
}

// Unreachable targets must be reported, not silently returned as NaN.
TEST(LegKinematics, unreachableTargetIsFlagged) {
  const auto geom = aliengoLegGeometry();
  const Eigen::Vector3d tooFar(0.25, 0.15, -5.0);

  bool reachable = true;
  const Eigen::Vector3d q = inverseKinematics(geom, 0, tooFar, &reachable);

  EXPECT_FALSE(reachable);
  EXPECT_TRUE(q.allFinite()) << "clamped result must still be finite";
}

// The analytic Jacobian must match a central finite difference of forwardKinematics.
TEST(LegKinematics, jacobianMatchesFiniteDifference) {
  const auto geom = aliengoLegGeometry();
  const Eigen::Vector3d q(-0.10, 0.62, -1.24);
  const double h = 1e-6;

  for (size_t block = 0; block < kNumLegs; ++block) {
    const Eigen::Matrix3d J = legJacobian(geom, block, q);

    for (int j = 0; j < 3; ++j) {
      Eigen::Vector3d qPlus = q;
      Eigen::Vector3d qMinus = q;
      qPlus[j] += h;
      qMinus[j] -= h;

      const Eigen::Vector3d numerical =
          (forwardKinematics(geom, block, qPlus) - forwardKinematics(geom, block, qMinus)) / (2.0 * h);

      for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(J(i, j), numerical(i), 1e-6) << "block " << block << " entry (" << i << "," << j << ")";
      }
    }
  }
}

// Joint velocities must reproduce the requested foot velocity through the Jacobian, for every leg.
TEST(LegKinematics, jointVelocitiesReproduceFootVelocity) {
  const auto geom = aliengoLegGeometry();
  const Eigen::Vector3d q(-0.10, 0.62, -1.24);
  const Eigen::Vector3d footVel(0.3, -0.1, 0.2);

  for (size_t block = 0; block < kNumLegs; ++block) {
    const Eigen::Vector3d qdot = jointVelocities(geom, block, q, footVel);
    const Eigen::Vector3d reproduced = legJacobian(geom, block, q) * qdot;
    EXPECT_NEAR((reproduced - footVel).norm(), 0.0, 1e-9) << "block " << block;
  }
}

// Away from singularities the damping must be exactly zero, so the solve is exact rather than
// merely close. This is what lets the 1e-9 tolerance above be honest.
TEST(LegKinematics, wellConditionedSolveIsExactNotMerelyDamped) {
  const auto geom = aliengoLegGeometry();
  const Eigen::Vector3d q(-0.10, 0.62, -1.24);
  const Eigen::Vector3d footVel(0.3, -0.1, 0.2);

  const Eigen::Vector3d qdot = jointVelocities(geom, 0, q, footVel);
  const Eigen::Vector3d reproduced = legJacobian(geom, 0, q) * qdot;

  // Floating-point noise, not a damping bias: several orders tighter than the 1e-9 contract.
  EXPECT_NEAR((reproduced - footVel).norm(), 0.0, 1e-12);
}

// THE point of the damping: as the knee straightens the Jacobian loses rank, and the commanded
// joint rate must stay bounded instead of exploding. Without adaptive damping this test fails by
// many orders of magnitude.
TEST(LegKinematics, nearSingularConfigurationYieldsBoundedJointRates) {
  const auto geom = aliengoLegGeometry();

  // Knee almost straight: r -> l2 + l3 = 0.5 m, where the planar Jacobian degenerates.
  const Eigen::Vector3d qNearSingular(0.0, 0.0, -1e-4);
  const Eigen::Matrix3d J = legJacobian(geom, 0, qNearSingular);
  const Eigen::JacobiSVD<Eigen::Matrix3d> svd(J);
  ASSERT_LT(svd.singularValues()(2), 0.02) << "fixture must actually be near-singular";

  // A unit foot velocity, including a component along the degenerate direction.
  const Eigen::Vector3d footVel(0.0, 0.0, 1.0);
  const Eigen::Vector3d qdot = jointVelocities(geom, 0, qNearSingular, footVel);

  ASSERT_TRUE(qdot.allFinite()) << "must not produce NaN/Inf at a singularity";
  // Bounded near the URDF's own joint-rate limits (hip 20, calf 15.89 rad/s), not unbounded.
  EXPECT_LT(qdot.norm(), 25.0) << "qdot = " << qdot.transpose();
}

// The bound must hold across the degenerate direction generally, not just one lucky input.
TEST(LegKinematics, jointRatesStayBoundedOverSingularDirections) {
  const auto geom = aliengoLegGeometry();
  const Eigen::Vector3d qNearSingular(0.0, 0.0, -1e-4);

  std::mt19937 rng(11);
  std::uniform_real_distribution<double> dir(-1.0, 1.0);
  for (int trial = 0; trial < 200; ++trial) {
    Eigen::Vector3d footVel(dir(rng), dir(rng), dir(rng));
    if (footVel.norm() < 1e-6) {
      continue;
    }
    footVel.normalize();

    const Eigen::Vector3d qdot = jointVelocities(geom, 0, qNearSingular, footVel);
    ASSERT_TRUE(qdot.allFinite()) << "trial " << trial;
    EXPECT_LT(qdot.norm(), 25.0) << "trial " << trial << " footVel " << footVel.transpose();
  }
}
