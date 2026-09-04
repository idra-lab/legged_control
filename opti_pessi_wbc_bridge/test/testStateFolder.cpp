#include <gtest/gtest.h>

#include <cmath>

#include "opti_pessi_wbc_bridge/StateFolder.h"

using namespace opti_pessi_bridge;

namespace {
constexpr int kStateDim = 24;  // 6 momentum + 6 base pose + 12 joints
constexpr int kInputDim = 24;  // 12 contact forces + 12 joint velocities
constexpr int kYawIndex = 9;

ocs2::vector_t nominalCentroidalState(double x, double y, double yaw) {
  ocs2::vector_t state = ocs2::vector_t::Zero(kStateDim);
  state(6) = x;
  state(7) = y;
  state(8) = 0.38;      // base height
  state(kYawIndex) = yaw;
  // defaultJointState, joint order LF, LH, RF, RH.
  const double q[12] = {-0.10, 0.62, -1.24, -0.10, 0.62, -1.24, 0.10, 0.62, -1.24, 0.10, 0.62, -1.24};
  for (int i = 0; i < 12; ++i) {
    state(12 + i) = q[i];
  }
  return state;
}
}  // namespace

TEST(StateFolder, producesA17DimensionalRobotState) {
  const auto geom = aliengoLegGeometry();
  const auto state = nominalCentroidalState(1.0, 2.0, 0.3);
  const auto input = ocs2::vector_t::Zero(kInputDim);

  const auto lip = foldToLipState(geom, state, input, 0, FoldedHistory{});

  EXPECT_EQ(lip.size(), opti_pessi::RobotX::DIM);
}

TEST(StateFolder, copiesPlanarPoseAndVelocity) {
  const auto geom = aliengoLegGeometry();
  auto state = nominalCentroidalState(1.0, 2.0, 0.3);
  state(0) = 0.4;  // normalized linear momentum x == CoM velocity x
  state(1) = -0.2;
  state(5) = 0.15;  // normalized angular momentum z
  const auto input = ocs2::vector_t::Zero(kInputDim);

  const auto lip = foldToLipState(geom, state, input, 0, FoldedHistory{});

  EXPECT_NEAR(lip(opti_pessi::RobotX::CX), 1.0, 1e-12);
  EXPECT_NEAR(lip(opti_pessi::RobotX::CY), 2.0, 1e-12);
  EXPECT_NEAR(lip(opti_pessi::RobotX::TH), 0.3, 1e-12);
  EXPECT_NEAR(lip(opti_pessi::RobotX::DCX), 0.4, 1e-12);
  EXPECT_NEAR(lip(opti_pessi::RobotX::DCY), -0.2, 1e-12);
}

// The stance feet recorded are the two the current parity designates, in gaitPair order.
TEST(StateFolder, stanceFeetFollowGaitParity) {
  const auto geom = aliengoLegGeometry();
  const auto state = nominalCentroidalState(0.0, 0.0, 0.0);
  const auto input = ocs2::vector_t::Zero(kInputDim);

  for (int parity = 0; parity < 2; ++parity) {
    const auto lip = foldToLipState(geom, state, input, parity, FoldedHistory{});
    const auto pair = opti_pessi::gaitPair(parity);

    // With zero base pose and nominal joints, each foot sits under its own hip.
    for (int k = 0; k < 2; ++k) {
      const Eigen::Vector3d expected =
          forwardKinematics(geom, jointBlockOf(pair[k]),
                            Eigen::Vector3d(state(12 + 3 * jointBlockOf(pair[k])),
                                            state(13 + 3 * jointBlockOf(pair[k])),
                                            state(14 + 3 * jointBlockOf(pair[k]))));
      const int xIdx = (k == 0) ? opti_pessi::RobotX::P0X : opti_pessi::RobotX::P1X;
      const int yIdx = (k == 0) ? opti_pessi::RobotX::P0Y : opti_pessi::RobotX::P1Y;
      EXPECT_NEAR(lip(xIdx), expected.x(), 1e-9) << "parity " << parity << " foot " << k;
      EXPECT_NEAR(lip(yIdx), expected.y(), 1e-9) << "parity " << parity << " foot " << k;
    }
  }
}

// Foot positions must be expressed in the WORLD frame, so a base translation moves them.
TEST(StateFolder, footPositionsAreInTheWorldFrame) {
  const auto geom = aliengoLegGeometry();
  const auto atOrigin = nominalCentroidalState(0.0, 0.0, 0.0);
  const auto shifted = nominalCentroidalState(1.5, -0.5, 0.0);
  const auto input = ocs2::vector_t::Zero(kInputDim);

  const auto lipA = foldToLipState(geom, atOrigin, input, 0, FoldedHistory{});
  const auto lipB = foldToLipState(geom, shifted, input, 0, FoldedHistory{});

  EXPECT_NEAR(lipB(opti_pessi::RobotX::P0X) - lipA(opti_pessi::RobotX::P0X), 1.5, 1e-9);
  EXPECT_NEAR(lipB(opti_pessi::RobotX::P0Y) - lipA(opti_pessi::RobotX::P0Y), -0.5, 1e-9);
}

TEST(StateFolder, worldTransformAppliesRotationBeforeTranslation) {
  const auto geom = aliengoLegGeometry();
  const double yaw = M_PI / 2.0;
  const double baseX = 1.5;
  const double baseY = -0.5;
  const auto state = nominalCentroidalState(baseX, baseY, yaw);
  const auto input = ocs2::vector_t::Zero(kInputDim);

  const auto lip = foldToLipState(geom, state, input, 0, FoldedHistory{});

  // Independently recompute the expected world position of the first stance foot.
  const auto pair = opti_pessi::gaitPair(0);
  const size_t block = jointBlockOf(pair[0]);
  const Eigen::Vector3d q(state(12 + 3 * block + 0), state(12 + 3 * block + 1),
                          state(12 + 3 * block + 2));
  const Eigen::Vector3d footInBase = forwardKinematics(geom, block, q);

  const double expectedX = baseX - footInBase.y();  // R(+pi/2): x' = -y
  const double expectedY = baseY + footInBase.x();  //           y' =  x

  EXPECT_NEAR(lip(opti_pessi::RobotX::P0X), expectedX, 1e-9);
  EXPECT_NEAR(lip(opti_pessi::RobotX::P0Y), expectedY, 1e-9);

  // Explicitly assert the buggy order is NOT what we got, so the test's purpose survives edits.
  const double buggyX = -(footInBase.y() + baseY);
  const double buggyY = footInBase.x() + baseX;
  EXPECT_FALSE(std::abs(lip(opti_pessi::RobotX::P0X) - buggyX) < 1e-9 &&
               std::abs(lip(opti_pessi::RobotX::P0Y) - buggyY) < 1e-9)
      << "foot was transformed with the translation applied before the rotation";
}

// A nonzero yaw must rotate the foot offsets about the base.
TEST(StateFolder, yawRotatesFootPositions) {
  const auto geom = aliengoLegGeometry();
  const auto unrotated = nominalCentroidalState(0.0, 0.0, 0.0);
  const auto rotated = nominalCentroidalState(0.0, 0.0, M_PI / 2.0);
  const auto input = ocs2::vector_t::Zero(kInputDim);

  const auto lipA = foldToLipState(geom, unrotated, input, 0, FoldedHistory{});
  const auto lipB = foldToLipState(geom, rotated, input, 0, FoldedHistory{});

  // Rotating by +90 degrees maps (x, y) to (-y, x).
  EXPECT_NEAR(lipB(opti_pessi::RobotX::P0X), -lipA(opti_pessi::RobotX::P0Y), 1e-9);
  EXPECT_NEAR(lipB(opti_pessi::RobotX::P0Y), lipA(opti_pessi::RobotX::P0X), 1e-9);
}

// History fields are carried verbatim; they are pure bookkeeping for the OCP's reachability rows.
TEST(StateFolder, historyFieldsAreCarried) {
  const auto geom = aliengoLegGeometry();
  const auto state = nominalCentroidalState(0.0, 0.0, 0.0);
  const auto input = ocs2::vector_t::Zero(kInputDim);

  FoldedHistory history;
  history.previousFoot0 = Eigen::Vector2d(0.1, 0.2);
  history.previousFoot1 = Eigen::Vector2d(0.3, 0.4);
  history.previousCom = Eigen::Vector2d(0.5, 0.6);
  history.previousYaw = 0.7;
  history.valid = true;

  const auto lip = foldToLipState(geom, state, input, 0, history);

  EXPECT_NEAR(lip(opti_pessi::RobotX::PP0X), 0.1, 1e-12);
  EXPECT_NEAR(lip(opti_pessi::RobotX::PP0Y), 0.2, 1e-12);
  EXPECT_NEAR(lip(opti_pessi::RobotX::PP1X), 0.3, 1e-12);
  EXPECT_NEAR(lip(opti_pessi::RobotX::PP1Y), 0.4, 1e-12);
  EXPECT_NEAR(lip(opti_pessi::RobotX::PCX), 0.5, 1e-12);
  EXPECT_NEAR(lip(opti_pessi::RobotX::PCY), 0.6, 1e-12);
  EXPECT_NEAR(lip(opti_pessi::RobotX::PTH), 0.7, 1e-12);
}

// With no history yet, the current values seed the previous slots so the OCP's reachability rows
// are trivially satisfied on the first solve rather than fed garbage.
TEST(StateFolder, invalidHistorySeedsFromCurrentValues) {
  const auto geom = aliengoLegGeometry();
  const auto state = nominalCentroidalState(1.0, 2.0, 0.0);
  const auto input = ocs2::vector_t::Zero(kInputDim);

  const auto lip = foldToLipState(geom, state, input, 0, FoldedHistory{});

  EXPECT_NEAR(lip(opti_pessi::RobotX::PP0X), lip(opti_pessi::RobotX::P0X), 1e-12);
  EXPECT_NEAR(lip(opti_pessi::RobotX::PP1Y), lip(opti_pessi::RobotX::P1Y), 1e-12);
  EXPECT_NEAR(lip(opti_pessi::RobotX::PCX), lip(opti_pessi::RobotX::CX), 1e-12);
  EXPECT_NEAR(lip(opti_pessi::RobotX::PTH), lip(opti_pessi::RobotX::TH), 1e-12);
}
