//
// Fall recovery of OptiPessiController: fall test, damping mode and the joint-space sequence that gets a fallen robot
// back onto its belly with the legs tucked, from where OptiPessiController::standUp() stands it up.
//
// Pure Eigen, no ROS: see test/testFallRecovery.cpp.
//

#pragma once

#include <cmath>
#include <cstddef>
#include <vector>

#include <Eigen/Core>

namespace legged {

struct FallRecoverySettings {
  // Fall test
  double tiltFallen = 60.0 * M_PI / 180.0;   // [rad] angle between the body z axis and the vertical
  double tiltUpright = 30.0 * M_PI / 180.0;  // [rad] below it the robot is on its belly, no roll-over needed
  double minBaseHeight = 0.20;               // [m] base above the lowest foot, once the robot has stood up

  // Damping mode: kp = 0, feedforward = 0, velocity = 0
  double dampingKd = 2.0;

  // At rest: both below their threshold for restDuration. After restTimeout of damping the robot counts as at rest.
  double restAngularVelocity = 0.5;  // [rad/s] base, norm
  double restJointVelocity = 1.0;    // [rad/s] fastest joint
  double restDuration = 0.5;         // [s]
  double restTimeout = 3.0;          // [s]

  // Joint PD while rolling and folding
  double kp = 80.0;
  double kd = 2.0;

  // Keyframes of one leg (HAA, HFE, KFE).
  Eigen::Vector3d tuck{0.0, 1.2, -2.7};  // belly-down lying pose standUp() starts from, all four legs
  // Roll-over pose of the left legs (LF, LH) and of the right legs (RF, RH) for roll direction +1. Direction -1 uses the
  // mirror image: the left legs take rollRight and the right legs rollLeft, with HAA negated.
  // Tuned on the Aliengo in Gazebo: the left legs swing over the back (HFE past pi, foot line along the body z axis)
  // and paddle inward under the body (HAA at its limit), the right legs fold along the body out of the way. On its
  // back the robot rolls onto its belly; on a side (against gravity) it rolls onto its back first.
  Eigen::Vector3d rollLeft{1.2, 3.89, -1.5};
  Eigen::Vector3d rollRight{0.0, 1.6, -2.7};
  // On a side, roll direction = sign of gravity along body y if true, its opposite if false.
  bool sideRollAlongGravity = false;
  // |gravity along body y| below this: the robot lies on its back and the roll direction alternates from +1.
  double rollDirectionDeadband = 0.5;

  // Segment durations [s]
  double tuckDuration = 0.6;
  double sweepDuration = 0.25;  // fast enough to carry the robot past its tipping point
  double holdDuration = 0.3;
  double foldDuration = 1.0;

  size_t maxRollAttempts = 4;      // per recovery
  size_t maxConsecutiveFalls = 3;  // recoveries without a completed stand-up in between
};

/**
 * Stage machine of one recovery:
 *   Damping: damping mode until the robot is at rest. On its belly (tilt < tiltUpright) -> Folding, otherwise -> Rolling.
 *   Rolling: tuck, move to the roll-over pose, hold, tuck again (joint PD), then Damping to check again.
 *   Folding: joint PD to the tuck pose -> Done if still on the belly, the caller then stands the robot up; tipped over
 *            meanwhile -> Damping to check again.
 *   GaveUp:  damping mode for good, after maxRollAttempts or past maxConsecutiveFalls. Only reset() leaves it.
 *
 * Joint order is the joint handle order LF, LH, RF, RH, each (HAA, HFE, KFE). A positive HAA angle moves the foot
 * toward body +y on every leg. The roll direction picks the roll-over pose (FallRecoverySettings::rollLeft/rollRight):
 * on a side it follows gravity along body y (see sideRollAlongGravity), on the back it alternates from +1.
 */
class FallRecovery {
 public:
  enum class Stage { Damping, Rolling, Folding, Done, GaveUp };

  using Vector12 = Eigen::Matrix<double, 12, 1>;

  struct Measurement {
    double roll = 0.0;   // ZYX Euler angles of the base
    double pitch = 0.0;
    Eigen::Vector3d angularVelocity = Eigen::Vector3d::Zero();  // base
    Vector12 jointPositions = Vector12::Zero();
    Vector12 jointVelocities = Vector12::Zero();
  };

  /** Per joint, written as setCommand(position, velocity, kp, kd, feedforward). */
  struct JointCommands {
    Vector12 position = Vector12::Zero();
    Vector12 velocity = Vector12::Zero();
    Vector12 kp = Vector12::Zero();
    Vector12 kd = Vector12::Zero();
    Vector12 feedforward = Vector12::Zero();
  };

  explicit FallRecovery(FallRecoverySettings settings);

  /** Angle between the body z axis and the vertical: cos(tilt) = cos(pitch) cos(roll). */
  static double tilt(double roll, double pitch);

  /** Tilted past tiltFallen, or (heightArmed only) the base less than minBaseHeight above the lowest foot. */
  static bool fallen(double roll, double pitch, double baseAboveFeet, bool heightArmed, const FallRecoverySettings& settings);

  /** Damping mode on every joint: position = measured, velocity = 0, kp = 0, kd = dampingKd, feedforward = 0. */
  static JointCommands damping(const Vector12& jointPositions, const FallRecoverySettings& settings);

  /** A fall: enters Damping, or GaveUp past maxConsecutiveFalls. */
  void start();

  /** Advances the stage machine by dt and writes this tick's commands. Returns the stage after the tick. */
  Stage update(double dt, const Measurement& measurement, JointCommands& commands);

  /** The stand-up after a recovery completed: the count of consecutive falls restarts. */
  void standUpCompleted();

  /** No recovery, counts cleared. */
  void reset();

  /** Replaces the settings, only while no recovery runs. False, and nothing changes, during a recovery. */
  bool setSettings(const FallRecoverySettings& settings);

  /** A recovery is running: any stage but Done. */
  bool active() const { return stage_ != Stage::Done; }
  Stage stage() const { return stage_; }
  size_t rollAttempts() const { return rollAttempts_; }
  /** Direction of the last roll-over (+1 or -1), 0 before the first of this recovery. */
  double rollDirection() const { return rollDirection_; }
  const FallRecoverySettings& settings() const { return settings_; }

 private:
  /** Joint PD move of all joints to `target` in `duration`. */
  struct Segment {
    Vector12 target;
    double duration;
  };

  /** Starts the segments from the measured joints and writes their first command. */
  void startSegments(Stage stage, std::vector<Segment> segments, const Vector12& jointPositions, JointCommands& commands);
  void writeSegmentCommand(JointCommands& commands) const;
  /** Roll-over pose of all four legs for roll direction `direction` (+1 or -1). */
  Vector12 rollOverPose(double direction) const;
  /** All four legs at `leg`. */
  static Vector12 allLegs(const Eigen::Vector3d& leg);
  /** +1 or -1: from gravity along body y on a side (sideRollAlongGravity), alternating on the back. */
  double chooseRollDirection(const Measurement& measurement);

  FallRecoverySettings settings_;
  Stage stage_ = Stage::Done;
  size_t consecutiveFalls_ = 0;
  size_t rollAttempts_ = 0;
  double rollDirection_ = 0.0;          // direction of the last roll, 0 before the first
  double lastBackRollDirection_ = 0.0;  // direction of the last roll from the back, 0 before the first

  double dampingElapsed_ = 0.0;  // [s] in the current Damping stage
  double restElapsed_ = 0.0;     // [s] at rest without interruption

  std::vector<Segment> segments_;
  size_t segmentIndex_ = 0;
  double segmentElapsed_ = 0.0;
  Vector12 segmentStart_ = Vector12::Zero();
};

}  // namespace legged
