#include "opti_pessi_interface/OptiPessiModelParameters.h"

#include <algorithm>
#include <iostream>
#include <stdexcept>

#include <boost/filesystem/operations.hpp>
#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_core/misc/LoadData.h>

namespace opti_pessi {

namespace {

void assertFileExists(const std::string& file, const std::string& what) {
  if (!boost::filesystem::exists(file)) {
    throw std::runtime_error("[OptiPessiModelParameters] The " + what + " \"" + file + "\" does not exist.");
  }
}

boost::property_tree::ptree readInfo(const std::string& file) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(file, pt);
  return pt;
}

void printHeader(const std::string& title, bool verbose) {
  if (verbose) {
    std::cerr << "\n #### " << title << ":";
    std::cerr << "\n #### =============================================================================\n";
  }
}

}  // namespace

ObstacleMovement obstacleMovementFromString(const std::string& name) {
  std::string lower = name;
  std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char c) { return std::tolower(c); });
  if (lower == "straight") {
    return ObstacleMovement::Straight;
  }
  if (lower == "patrol") {
    return ObstacleMovement::Patrol;
  }
  if (lower == "circle") {
    return ObstacleMovement::Circle;
  }
  if (lower == "antagonist") {
    return ObstacleMovement::Antagonist;
  }
  throw std::runtime_error("[OptiPessiModelParameters] Unknown obstacle movement \"" + name +
                           "\". Expected one of: straight, patrol, circle, antagonist.");
}

std::string toString(ObstacleMovement movement) {
  switch (movement) {
    case ObstacleMovement::Straight:
      return "straight";
    case ObstacleMovement::Patrol:
      return "patrol";
    case ObstacleMovement::Circle:
      return "circle";
    case ObstacleMovement::Antagonist:
      return "antagonist";
  }
  return "unknown";
}

OptiPessiModelParameters loadOptiPessiModelParameters(const std::string& taskFile, const std::string& scenarioFile, bool verbose) {
  assertFileExists(taskFile, "task file");
  assertFileExists(scenarioFile, "scenario file");

  OptiPessiModelParameters p;

  // ---------------------------------------------------------------------------------------------
  // task file: model, cost, limits, horizon, initial state
  // ---------------------------------------------------------------------------------------------
  const auto task = readInfo(taskFile);

  printHeader("Opti-Pessi Model", verbose);
  ocs2::loadData::loadPtreeValue(task, p.comHeight, "model.comHeight", verbose);
  ocs2::loadData::loadPtreeValue(task, p.gravity, "model.gravity", verbose);
  ocs2::loadData::loadPtreeValue(task, p.mass, "model.mass", verbose);
  ocs2::loadData::loadPtreeValue(task, p.inertia, "model.inertia", verbose);
  ocs2::loadData::loadPtreeValue(task, p.frictionCoefficient, "model.frictionCoefficient", verbose);

  matrix_t hips(4, 2);
  ocs2::loadData::loadEigenMatrix(taskFile, "hipOffsets", hips);
  for (int f = 0; f < 4; ++f) {
    p.hipOffsets[static_cast<size_t>(f)] = hips.row(f).transpose();
  }
  if (verbose) {
    std::cerr << " #### 'hipOffsets' (FL, FR, RL, RR):\n" << hips << "\n";
  }

  printHeader("Opti-Pessi Cost Weights", verbose);
  ocs2::loadData::loadPtreeValue(task, p.wc, "cost.wc", verbose);
  ocs2::loadData::loadPtreeValue(task, p.wdc, "cost.wdc", verbose);
  ocs2::loadData::loadPtreeValue(task, p.wp, "cost.wp", verbose);
  ocs2::loadData::loadPtreeValue(task, p.wa, "cost.wa", verbose);
  ocs2::loadData::loadPtreeValue(task, p.wdt, "cost.wdt", verbose);
  ocs2::loadData::loadPtreeValue(task, p.wtheta, "cost.wtheta", verbose);
  ocs2::loadData::loadPtreeValue(task, p.wdtheta, "cost.wdtheta", verbose);

  printHeader("Opti-Pessi Limits", verbose);
  ocs2::loadData::loadPtreeValue(task, p.alphaReduction, "limits.alphaReduction", verbose);
  ocs2::loadData::loadPtreeValue(task, p.dtMin, "limits.dtMin", verbose);
  ocs2::loadData::loadPtreeValue(task, p.dtMax, "limits.dtMax", verbose);
  ocs2::loadData::loadPtreeValue(task, p.dtCost0, "limits.dtCost0", verbose);
  ocs2::loadData::loadPtreeValue(task, p.footHipMax, "limits.footHipMax", verbose);
  ocs2::loadData::loadPtreeValue(task, p.dcxMax, "limits.dcxMax", verbose);
  ocs2::loadData::loadPtreeValue(task, p.dcyMax, "limits.dcyMax", verbose);
  ocs2::loadData::loadPtreeValue(task, p.dthetaMax, "limits.dthetaMax", verbose);

  ocs2::loadData::loadPtreeValue(task, p.N, "horizon.N", verbose);
  if (p.N < 2) {
    throw std::runtime_error("[OptiPessiModelParameters] horizon.N must be at least 2.");
  }

  p.initialState.setZero(OptiPessiModelParameters::kMeasuredStateDim);
  ocs2::loadData::loadEigenMatrix(taskFile, "initialState", p.initialState);
  if (verbose) {
    std::cerr << " #### 'initialState': " << p.initialState.transpose() << "\n";
  }

  // ---------------------------------------------------------------------------------------------
  // scenario file: goal, obstacles, obstacle plant, simulation
  // ---------------------------------------------------------------------------------------------
  const auto scenario = readInfo(scenarioFile);

  printHeader("Opti-Pessi Scenario", verbose);
  p.goal.setZero(2);
  ocs2::loadData::loadEigenMatrix(scenarioFile, "goal", p.goal);
  if (verbose) {
    std::cerr << " #### 'goal': " << p.goal.transpose() << "\n";
  }

  int numObstacles = 0;
  ocs2::loadData::loadPtreeValue(scenario, numObstacles, "obstacles.numObstacles", verbose);
  if (numObstacles < 1) {
    throw std::runtime_error("[OptiPessiModelParameters] obstacles.numObstacles must be at least 1.");
  }
  ocs2::loadData::loadPtreeValue(scenario, p.obstacleRadius, "obstacles.radius", verbose);
  ocs2::loadData::loadPtreeValue(scenario, p.obstacleMaxSpeed, "obstacles.maxSpeed", verbose);

  p.obstaclePositions.setZero(numObstacles, 2);
  ocs2::loadData::loadEigenMatrix(scenarioFile, "obstacles.positions", p.obstaclePositions);
  if (verbose) {
    std::cerr << " #### 'obstacles.positions':\n" << p.obstaclePositions << "\n";
  }

  std::vector<std::string> movementNames;
  ocs2::loadData::loadStdVector(scenarioFile, "plant.movement", movementNames, verbose);
  if (static_cast<int>(movementNames.size()) != numObstacles) {
    throw std::runtime_error("[OptiPessiModelParameters] plant.movement has " + std::to_string(movementNames.size()) +
                             " entries but numObstacles is " + std::to_string(numObstacles) + ".");
  }
  p.obstacleMovement.clear();
  p.obstacleMovement.reserve(movementNames.size());
  for (const auto& name : movementNames) {
    p.obstacleMovement.push_back(obstacleMovementFromString(name));
  }

  std::vector<scalar_t> speeds;
  ocs2::loadData::loadStdVector(scenarioFile, "plant.speed", speeds, verbose);
  if (static_cast<int>(speeds.size()) != numObstacles) {
    throw std::runtime_error("[OptiPessiModelParameters] plant.speed has " + std::to_string(speeds.size()) +
                             " entries but numObstacles is " + std::to_string(numObstacles) + ".");
  }
  p.obstacleSpeed = Eigen::Map<const vector_t>(speeds.data(), numObstacles);

  p.obstacleDir.setZero(numObstacles, 2);
  ocs2::loadData::loadEigenMatrix(scenarioFile, "plant.direction", p.obstacleDir);
  if (verbose) {
    std::cerr << " #### 'plant.direction':\n" << p.obstacleDir << "\n";
  }

  ocs2::loadData::loadPtreeValue(scenario, p.simTime, "simulation.simTime", verbose);
  ocs2::loadData::loadPtreeValue(scenario, p.goalTolerance, "simulation.goalTolerance", verbose);
  ocs2::loadData::loadPtreeValue(scenario, p.figName, "simulation.figName", verbose);

  if (verbose) {
    std::cerr << " #### =============================================================================" << std::endl;
  }

  return p;
}

}  // namespace opti_pessi
