//
// Refactored for ROS 2
//

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <ocs2_self_collision_visualization/GeometryInterfaceVisualization.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <utility>

namespace legged {
using namespace ocs2;

class LeggedSelfCollisionVisualization : public GeometryInterfaceVisualization {
 public:
  LeggedSelfCollisionVisualization(PinocchioInterface pinocchioInterface, PinocchioGeometryInterface geometryInterface,
                                   const CentroidalModelPinocchioMapping& mapping, scalar_t maxUpdateFrequency = 50.0)
      : GeometryInterfaceVisualization(std::move(pinocchioInterface), std::move(geometryInterface), "odom"),
        mappingPtr_(mapping.clone()),
        lastTime_(std::numeric_limits<scalar_t>::lowest()),
        minPublishTimeDifference_(1.0 / maxUpdateFrequency) {}

  void update(const SystemObservation& observation) {
    if (observation.time - lastTime_ > minPublishTimeDifference_) {
      lastTime_ = observation.time;
      publishDistances(mappingPtr_->getPinocchioJointPosition(observation.state));
    }
  }

 private:
  std::unique_ptr<CentroidalModelPinocchioMapping> mappingPtr_;
  scalar_t lastTime_;
  scalar_t minPublishTimeDifference_;
};

}  // namespace legged
