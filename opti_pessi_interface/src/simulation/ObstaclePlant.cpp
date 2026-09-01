#include "opti_pessi_interface/simulation/ObstaclePlant.h"

#include <algorithm>
#include <array>
#include <cmath>

#include "opti_pessi_interface/LipKinematics.h"

namespace opti_pessi {

ObstaclePlant makeObstaclePlant(const OptiPessiModelParameters& params) {
  ObstaclePlant plant;
  plant.positions = params.obstaclePositions;
  plant.direction = params.obstacleDir;
  plant.radius = vector_t::Zero(params.numObstacles());
  plant.phase = vector_t::Zero(params.numObstacles());
  return plant;
}

void stepObstacles(ObstaclePlant& plant, const OptiPessiModelParameters& params, const vector_t& robotCom, scalar_t dt, int step) {
  for (int j = 0; j < params.numObstacles(); ++j) {
    const scalar_t speed = params.obstacleSpeed(j);
    switch (params.obstacleMovement[static_cast<size_t>(j)]) {
      case ObstacleMovement::Straight: {
        plant.positions(j, 0) += plant.direction(j, 0) * speed * dt;
        plant.positions(j, 1) += plant.direction(j, 1) * speed * dt;
        break;
      }
      case ObstacleMovement::Patrol: {
        // Sweeps a 2 m corridor in y about the starting position, reversing at each end.
        if (step == 0) {
          plant.direction.row(j) = params.obstacleDir.row(j);
        } else if (plant.positions(j, 1) < params.obstaclePositions(j, 1) - 2.0 ||
                   plant.positions(j, 1) > params.obstaclePositions(j, 1)) {
          plant.direction.row(j) *= -1.0;
        }
        plant.positions(j, 0) += plant.direction(j, 0) * speed * dt;
        plant.positions(j, 1) += plant.direction(j, 1) * speed * dt;
        break;
      }
      case ObstacleMovement::Circle: {
        if (!plant.circleInitialized) {
          const scalar_t dx = plant.positions(j, 0) - params.goal(0);
          const scalar_t dy = plant.positions(j, 1) - params.goal(1);
          plant.radius(j) = std::sqrt(dx * dx + dy * dy);
          plant.phase(j) = std::atan2(dy, dx);
          plant.circleInitialized = true;
        }
        plant.phase(j) += (speed / plant.radius(j)) * dt;
        plant.positions(j, 0) = params.goal(0) + plant.radius(j) * std::cos(plant.phase(j));
        plant.positions(j, 1) = params.goal(1) + plant.radius(j) * std::sin(plant.phase(j));
        break;
      }
      case ObstacleMovement::Antagonist: {
        // Heads straight for the robot.
        vector_t v(2);
        v << robotCom(0) - plant.positions(j, 0), robotCom(1) - plant.positions(j, 1);
        const scalar_t norm = v.norm();
        if (norm > 1e-12) {
          v /= norm;
        }
        plant.positions(j, 0) += v(0) * speed * dt;
        plant.positions(j, 1) += v(1) * speed * dt;
        break;
      }
    }
  }
}

bool hipsAndFeetCollide(const vector_t& com, scalar_t theta, const vector_t& feet, const vector_t& obstacle,
                        const OptiPessiModelParameters& params, scalar_t dMin) {
  std::array<vector_t, 6> points{};
  int numPoints = 4;
  for (int f = 0; f < 4; ++f) {
    points[static_cast<size_t>(f)] = com + applyR(theta, hipOf(params, static_cast<Foot>(f)));
  }
  if (feet.size() >= 4) {
    points[4] = feet.head(2);
    points[5] = feet.tail(2);
    numPoints = 6;
  }

  // Max over directions of (obstacle projection - hull support) is the hull-to-centre distance.
  scalar_t best = -1e9;
  constexpr int numDirections = 64;
  for (int k = 0; k < numDirections; ++k) {
    const scalar_t angle = 2.0 * M_PI * static_cast<scalar_t>(k) / numDirections;
    const vector_t a = (vector_t(2) << std::cos(angle), std::sin(angle)).finished();
    scalar_t support = -1e9;
    for (int i = 0; i < numPoints; ++i) {
      support = std::max(support, a.dot(points[static_cast<size_t>(i)]));
    }
    best = std::max(best, a.dot(obstacle) - support);
  }
  return best < dMin;
}

}  // namespace opti_pessi
