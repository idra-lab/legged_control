#include "legged_controllers/FallRecovery.h"

#include <algorithm>
#include <utility>

namespace legged {

namespace {
// Time comparisons: durations are sums of equal ticks, so a segment of n ticks ends on its n-th tick.
constexpr double kTimeEpsilon = 1e-9;
}  // namespace

FallRecovery::FallRecovery(FallRecoverySettings settings) : settings_(std::move(settings)) {}

double FallRecovery::tilt(double roll, double pitch) {
  return std::acos(std::clamp(std::cos(pitch) * std::cos(roll), -1.0, 1.0));
}

bool FallRecovery::fallen(double roll, double pitch, double baseAboveFeet, bool heightArmed, const FallRecoverySettings& settings) {
  return tilt(roll, pitch) > settings.tiltFallen || (heightArmed && baseAboveFeet < settings.minBaseHeight);
}

FallRecovery::JointCommands FallRecovery::damping(const Vector12& jointPositions, const FallRecoverySettings& settings) {
  JointCommands commands;
  commands.position = jointPositions;
  commands.kd.setConstant(settings.dampingKd);
  return commands;
}

void FallRecovery::start() {
  ++consecutiveFalls_;
  rollAttempts_ = 0;
  rollDirection_ = 0.0;
  lastBackRollDirection_ = 0.0;
  dampingElapsed_ = 0.0;
  restElapsed_ = 0.0;
  segments_.clear();
  stage_ = consecutiveFalls_ > settings_.maxConsecutiveFalls ? Stage::GaveUp : Stage::Damping;
}

FallRecovery::Stage FallRecovery::update(double dt, const Measurement& measurement, JointCommands& commands) {
  switch (stage_) {
    case Stage::Damping: {
      dampingElapsed_ += dt;
      const bool atRest = measurement.angularVelocity.norm() < settings_.restAngularVelocity &&
                          measurement.jointVelocities.cwiseAbs().maxCoeff() < settings_.restJointVelocity;
      restElapsed_ = atRest ? restElapsed_ + dt : 0.0;
      if (restElapsed_ < settings_.restDuration - kTimeEpsilon && dampingElapsed_ < settings_.restTimeout - kTimeEpsilon) {
        commands = damping(measurement.jointPositions, settings_);
        break;
      }
      if (tilt(measurement.roll, measurement.pitch) < settings_.tiltUpright) {
        startSegments(Stage::Folding, {{allLegs(settings_.tuck), settings_.foldDuration}}, measurement.jointPositions, commands);
      } else if (rollAttempts_ >= settings_.maxRollAttempts) {
        stage_ = Stage::GaveUp;
        commands = damping(measurement.jointPositions, settings_);
      } else {
        ++rollAttempts_;
        rollDirection_ = chooseRollDirection(measurement);
        const Vector12 rollPose = rollOverPose(rollDirection_);
        startSegments(Stage::Rolling,
                      {{allLegs(settings_.tuck), settings_.tuckDuration},
                       {rollPose, settings_.sweepDuration},
                       {rollPose, settings_.holdDuration},
                       {allLegs(settings_.tuck), settings_.tuckDuration}},
                      measurement.jointPositions, commands);
      }
      break;
    }

    case Stage::Rolling:
    case Stage::Folding: {
      segmentElapsed_ += dt;
      if (segmentElapsed_ > segments_[segmentIndex_].duration + kTimeEpsilon) {
        // The previous tick ended this segment on its target: the next one starts from the joints measured now.
        ++segmentIndex_;
        segmentElapsed_ = 0.0;
        segmentStart_ = measurement.jointPositions;
      }
      if (segmentIndex_ < segments_.size()) {
        writeSegmentCommand(commands);
        break;
      }
      if (stage_ == Stage::Folding && tilt(measurement.roll, measurement.pitch) < settings_.tiltUpright) {
        // Folded on the belly: hold the tuck pose until the caller takes over.
        stage_ = Stage::Done;
        segmentIndex_ = segments_.size() - 1;
        segmentElapsed_ = segments_.back().duration;
        writeSegmentCommand(commands);
      } else {
        // Rolled, or tipped over while folding: damping again, then check where the robot lies now.
        stage_ = Stage::Damping;
        dampingElapsed_ = 0.0;
        restElapsed_ = 0.0;
        commands = damping(measurement.jointPositions, settings_);
      }
      break;
    }

    case Stage::Done:
      if (segments_.empty()) {
        commands = damping(measurement.jointPositions, settings_);
      } else {
        writeSegmentCommand(commands);
      }
      break;

    case Stage::GaveUp:
      commands = damping(measurement.jointPositions, settings_);
      break;
  }
  return stage_;
}

void FallRecovery::standUpCompleted() {
  consecutiveFalls_ = 0;
}

void FallRecovery::reset() {
  stage_ = Stage::Done;
  consecutiveFalls_ = 0;
  rollAttempts_ = 0;
  rollDirection_ = 0.0;
  lastBackRollDirection_ = 0.0;
  dampingElapsed_ = 0.0;
  restElapsed_ = 0.0;
  segments_.clear();
  segmentIndex_ = 0;
  segmentElapsed_ = 0.0;
}

bool FallRecovery::setSettings(const FallRecoverySettings& settings) {
  if (active()) {
    return false;
  }
  settings_ = settings;
  return true;
}

void FallRecovery::startSegments(Stage stage, std::vector<Segment> segments, const Vector12& jointPositions, JointCommands& commands) {
  stage_ = stage;
  segments_ = std::move(segments);
  segmentIndex_ = 0;
  segmentElapsed_ = 0.0;
  segmentStart_ = jointPositions;
  writeSegmentCommand(commands);
}

void FallRecovery::writeSegmentCommand(JointCommands& commands) const {
  // Cubic from the segment start to its target, zero velocity at both ends.
  const Segment& segment = segments_[segmentIndex_];
  const double s = std::min(segmentElapsed_ / segment.duration, 1.0);
  const Vector12 delta = segment.target - segmentStart_;
  commands.position = segmentStart_ + (3.0 - 2.0 * s) * s * s * delta;
  commands.velocity = 6.0 * s * (1.0 - s) / segment.duration * delta;
  commands.kp.setConstant(settings_.kp);
  commands.kd.setConstant(settings_.kd);
  commands.feedforward.setZero();
}

FallRecovery::Vector12 FallRecovery::rollOverPose(double direction) const {
  // Direction -1 is the mirror image of direction +1: the sides swap and HAA changes sign.
  const auto mirrored = [](const Eigen::Vector3d& leg) { return Eigen::Vector3d(-leg(0), leg(1), leg(2)); };
  const Eigen::Vector3d left = direction > 0.0 ? settings_.rollLeft : mirrored(settings_.rollRight);
  const Eigen::Vector3d right = direction > 0.0 ? settings_.rollRight : mirrored(settings_.rollLeft);
  Vector12 joints;
  joints << left, left, right, right;  // LF, LH, RF, RH
  return joints;
}

FallRecovery::Vector12 FallRecovery::allLegs(const Eigen::Vector3d& leg) {
  Vector12 joints;
  joints << leg, leg, leg, leg;
  return joints;
}

double FallRecovery::chooseRollDirection(const Measurement& measurement) {
  // Gravity along body y (ZYX Euler angles): which side the robot lies on.
  const double gravityY = -std::sin(measurement.roll) * std::cos(measurement.pitch);
  if (std::abs(gravityY) >= settings_.rollDirectionDeadband) {
    const double alongGravity = gravityY > 0.0 ? 1.0 : -1.0;
    return settings_.sideRollAlongGravity ? alongGravity : -alongGravity;
  }
  lastBackRollDirection_ = lastBackRollDirection_ > 0.0 ? -1.0 : 1.0;
  return lastBackRollDirection_;
}

}  // namespace legged
