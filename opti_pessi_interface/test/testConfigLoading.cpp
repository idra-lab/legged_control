#include <string>

#include <gtest/gtest.h>

#include "opti_pessi_interface/OptiPessiModelParameters.h"

using namespace opti_pessi;

namespace {

// OPTI_PESSI_CONFIG_DIR is injected by CMake and points at the in-source config/ directory.
std::string configPath(const std::string& name) {
  return std::string(OPTI_PESSI_CONFIG_DIR) + "/" + name;
}

OptiPessiModelParameters load(const std::string& scenario) {
  return loadOptiPessiModelParameters(configPath("task.info"), configPath(scenario), false);
}

}  // namespace

// Every literal below is transcribed from opti-pessi-mpc/aliengo_conf.py. If task.info drifts from
// the Python reference, this test is where it shows up.
TEST(ConfigLoading, TaskFileMatchesPythonReference) {
  const auto p = load("scenario_S1.info");

  EXPECT_DOUBLE_EQ(p.comHeight, 0.38);
  EXPECT_DOUBLE_EQ(p.gravity, 9.81);
  EXPECT_DOUBLE_EQ(p.mass, 24.24);
  EXPECT_DOUBLE_EQ(p.inertia, 1.048);
  EXPECT_DOUBLE_EQ(p.frictionCoefficient, 0.8);

  EXPECT_DOUBLE_EQ(p.wc, 1.0);
  EXPECT_DOUBLE_EQ(p.wdc, 2.0);
  EXPECT_DOUBLE_EQ(p.wp, 0.5);
  EXPECT_DOUBLE_EQ(p.wa, 0.5);
  EXPECT_DOUBLE_EQ(p.wdt, 1e-2);
  EXPECT_DOUBLE_EQ(p.wtheta, 0.0);
  EXPECT_DOUBLE_EQ(p.wdtheta, 2.0);

  EXPECT_DOUBLE_EQ(p.alphaReduction, 0.1);
  EXPECT_DOUBLE_EQ(p.dtMin, 0.2);
  EXPECT_DOUBLE_EQ(p.dtMax, 0.35);
  EXPECT_DOUBLE_EQ(p.dtCost0, 0.35);
  EXPECT_DOUBLE_EQ(p.footHipMax, 0.1);
  EXPECT_DOUBLE_EQ(p.dcxMax, 1.5);
  EXPECT_DOUBLE_EQ(p.dcyMax, 0.45);
  EXPECT_DOUBLE_EQ(p.dthetaMax, 0.8);

  EXPECT_EQ(p.N, 6);
  EXPECT_NEAR(p.omega(), std::sqrt(9.81 / 0.38), 1e-12);

  EXPECT_DOUBLE_EQ(p.initialState(RobotX::CX), -0.8);
  EXPECT_DOUBLE_EQ(p.initialState(RobotX::CY), -0.5);
  EXPECT_DOUBLE_EQ(p.initialState.tail(8).norm(), 0.0);

  // Hip offsets in Foot order (FL, FR, RL, RR).
  EXPECT_DOUBLE_EQ(hipOf(p, Foot::FL)(0), 0.2407);
  EXPECT_DOUBLE_EQ(hipOf(p, Foot::FL)(1), 0.134);
  EXPECT_DOUBLE_EQ(hipOf(p, Foot::FR)(0), 0.2407);
  EXPECT_DOUBLE_EQ(hipOf(p, Foot::FR)(1), -0.134);
  EXPECT_DOUBLE_EQ(hipOf(p, Foot::RL)(0), -0.2407);
  EXPECT_DOUBLE_EQ(hipOf(p, Foot::RL)(1), 0.134);
  EXPECT_DOUBLE_EQ(hipOf(p, Foot::RR)(0), -0.2407);
  EXPECT_DOUBLE_EQ(hipOf(p, Foot::RR)(1), -0.134);

  // Augmented dimensions: 2 branches of 14 states (10 physical + 4 previous footholds) + clock,
  // and 2 branches of 8 inputs + 2 hyperplane variables (phi, b) per obstacle per branch.
  EXPECT_EQ(p.stateDim(), 2 * 14 + 1);
  EXPECT_EQ(p.inputDim(), 2 * 8 + 2 * 2 * 1);
}

TEST(ConfigLoading, ScenarioS1IsStaticObstacle) {
  const auto p = load("scenario_S1.info");
  ASSERT_EQ(p.numObstacles(), 1);
  EXPECT_DOUBLE_EQ(p.goal(0), 1.0);
  EXPECT_DOUBLE_EQ(p.goal(1), 0.2);
  EXPECT_DOUBLE_EQ(p.obstaclePositions(0, 0), 0.1);
  EXPECT_DOUBLE_EQ(p.obstaclePositions(0, 1), 1.0);
  EXPECT_DOUBLE_EQ(p.obstacleRadius, 0.2);
  EXPECT_DOUBLE_EQ(p.obstacleMaxSpeed, 1.0);
  EXPECT_EQ(p.obstacleMovement.at(0), ObstacleMovement::Straight);
  EXPECT_DOUBLE_EQ(p.obstacleSpeed(0), 0.0);
  EXPECT_DOUBLE_EQ(p.obstacleDir(0, 0), 0.0);
  EXPECT_DOUBLE_EQ(p.obstacleDir(0, 1), -1.0);
  EXPECT_DOUBLE_EQ(p.simTime, 20.0);
  EXPECT_DOUBLE_EQ(p.goalTolerance, 0.1);
  EXPECT_EQ(p.figName, "mpc_sim_0_");
}

TEST(ConfigLoading, ScenarioS2IsPatrol) {
  const auto p = load("scenario_S2.info");
  EXPECT_EQ(p.obstacleMovement.at(0), ObstacleMovement::Patrol);
  EXPECT_DOUBLE_EQ(p.obstacleSpeed(0), 0.5);
  EXPECT_DOUBLE_EQ(p.simTime, 20.0);
  EXPECT_EQ(p.figName, "mpc_sim_5_");
}

TEST(ConfigLoading, ScenarioS3IsCircle) {
  const auto p = load("scenario_S3.info");
  EXPECT_EQ(p.obstacleMovement.at(0), ObstacleMovement::Circle);
  EXPECT_DOUBLE_EQ(p.obstacleSpeed(0), 0.45);
  EXPECT_EQ(p.figName, "mpc_sim_4_");
}

TEST(ConfigLoading, ScenarioS4IsAntagonist) {
  const auto p = load("scenario_S4.info");
  EXPECT_EQ(p.obstacleMovement.at(0), ObstacleMovement::Antagonist);
  EXPECT_DOUBLE_EQ(p.obstacleSpeed(0), 0.2);
  EXPECT_DOUBLE_EQ(p.obstaclePositions(0, 0), 0.3);
  EXPECT_DOUBLE_EQ(p.obstaclePositions(0, 1), 0.8);
  EXPECT_DOUBLE_EQ(p.simTime, 30.0);
  EXPECT_EQ(p.figName, "mpc_sim_2_");
}

TEST(ConfigLoading, RejectsUnknownMovement) {
  EXPECT_THROW(obstacleMovementFromString("teleport"), std::runtime_error);
  EXPECT_EQ(obstacleMovementFromString("ANTAGONIST"), ObstacleMovement::Antagonist);
}

TEST(ConfigLoading, RejectsMissingFiles) {
  EXPECT_THROW(loadOptiPessiModelParameters(configPath("does_not_exist.info"), configPath("scenario_S1.info"), false),
               std::runtime_error);
  EXPECT_THROW(loadOptiPessiModelParameters(configPath("task.info"), configPath("does_not_exist.info"), false),
               std::runtime_error);
}
