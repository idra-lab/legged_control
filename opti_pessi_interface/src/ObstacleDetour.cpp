#include "opti_pessi_interface/ObstacleDetour.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "opti_pessi_interface/LipKinematics.h"

namespace opti_pessi {

namespace {

using vector2_t = Eigen::Matrix<scalar_t, 2, 1>;

// [m] past the grown keep-out and the hull. Covers the corner the robot cuts in Gazebo: it walks ahead rather than
// sideways (dcyMax = 0.25) and came 0.2 m inside a 0.15 m margin.
constexpr scalar_t kDetourMargin = 0.3;
constexpr scalar_t kMaxDetourAngle = 0.75 * M_PI;  // [rad] from the direction to the obstacle, deepest inside the disk
constexpr scalar_t kReleaseHysteresis = 0.1;  // [m] the line must clear the disk by this much to release the detour
constexpr scalar_t kLookahead = 1.0;          // [m] how far past the tangent point the detour goal sits
constexpr scalar_t kSameObstacle = 0.5;       // [m] a blocking centre this close to the last one keeps the side

scalar_t cross2(const vector2_t& a, const vector2_t& b) {
  return a.x() * b.y() - a.y() * b.x();
}

/** Distance from p to the segment [a, b]. */
scalar_t segmentDistance(const vector2_t& a, const vector2_t& b, const vector2_t& p) {
  const vector2_t ab = b - a;
  const scalar_t lengthSq = ab.squaredNorm();
  const scalar_t t = lengthSq > 1e-12 ? std::min(std::max((p - a).dot(ab) / lengthSq, scalar_t(0)), scalar_t(1)) : scalar_t(0);
  return (a + t * ab - p).norm();
}

}  // namespace

ObstacleDetour::ObstacleDetour(const OptiPessiModelParameters& params) : horizonTime_(params.N * params.dtMax) {
  for (const auto& hip : params.hipOffsets) {
    hullRadius_ = std::max(hullRadius_, hip.norm());
  }
}

vector_t ObstacleDetour::detourGoal(const vector_t& com, const vector_t& goal, const matrix_t& obstacles, const vector_t& radii,
                                    const vector_t& maxSpeeds) {
  const vector2_t c = com.head<2>();
  const vector2_t g = goal.head<2>();
  const vector2_t path = g - c;

  // The blocking obstacle closest along the path. While a detour is active its own obstacle keeps blocking until the
  // line clears it by the hysteresis.
  int blocking = -1;
  scalar_t blockingAlong = std::numeric_limits<scalar_t>::max();
  scalar_t blockingRadius = 0.0;
  for (int j = 0; j < obstacles.rows(); ++j) {
    if (radii(j) <= 0.0 && maxSpeeds(j) <= 0.0) {
      continue;  // free slot
    }
    const vector2_t o = obstacles.row(j).transpose();
    const scalar_t detourRadius = radii(j) + maxSpeeds(j) * horizonTime_ + hullRadius_ + kDetourMargin;
    const scalar_t along = (o - c).dot(path);
    if (along <= 0.0 || (g - o).norm() < detourRadius) {
      continue;  // behind the robot, or the goal itself is inside the disk: nothing to walk around
    }
    const bool sameAsActive = active_ && (o - blockingCentre_).norm() < kSameObstacle;
    const scalar_t threshold = sameAsActive ? detourRadius + kReleaseHysteresis : detourRadius;
    if (segmentDistance(c, g, o) < threshold && along < blockingAlong) {
      blocking = j;
      blockingAlong = along;
      blockingRadius = detourRadius;
    }
  }

  if (blocking < 0) {
    active_ = false;
    return goal;
  }

  const vector2_t o = obstacles.row(blocking).transpose();
  if (!active_ || (o - blockingCentre_).norm() >= kSameObstacle) {
    // Pass on the side away from the obstacle: obstacle left of the path (cross > 0) -> go right (side -1).
    side_ = cross2(path, o - c) > 0.0 ? -1.0 : 1.0;
  }
  active_ = true;
  blockingCentre_ = o;

  // The cost pulls with wc * |c - goal|^2, so the detour sits as far away as the real goal: a detour goal closer than the
  // goal would slow the robot down exactly where it has to cover the most ground.
  const scalar_t reach = std::max(path.norm(), kLookahead);
  const vector2_t toObstacle = o - c;
  const scalar_t d = toObstacle.norm();
  const vector2_t u = toObstacle / std::max(d, scalar_t(1e-9));
  // Angle between the direction to the obstacle and the direction to walk. Outside the disk: the tangent, asin(R/d),
  // which reaches 90 deg on the boundary. Inside: keeps turning away with the depth, up to kMaxDetourAngle, so the
  // robot circles out of the disk. It never points backwards: a detour goal behind the robot reversed a 0.6 m/s walk in
  // Gazebo, and the back-and-forth next to the keep-out made the solves infeasible until the fallback stops tipped it.
  scalar_t alpha;
  if (d > blockingRadius) {
    alpha = std::asin(blockingRadius / d);
  } else {
    const scalar_t depth = std::min((blockingRadius - d) / kDetourMargin, scalar_t(1));
    alpha = scalar_t(M_PI_2) + depth * (kMaxDetourAngle - scalar_t(M_PI_2));
  }
  const vector2_t direction = applyR(side_ * alpha, u);
  const scalar_t tangentLength = d > blockingRadius ? std::sqrt(d * d - blockingRadius * blockingRadius) : scalar_t(0);
  vector_t result = goal;
  result.head(2) = c + direction * std::max(reach, tangentLength + kLookahead);
  return result;
}

}  // namespace opti_pessi
