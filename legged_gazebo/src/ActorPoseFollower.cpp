// Moves a Gazebo actor to the positions received on a topic, as a person walking there: it faces the way it moves and
// its walk animation advances with the distance covered. The actor of worlds/scenario_room.world has it, and gets the
// positions of the obstacle of legged_controllers opti_pessi_scenario.py --person.
//
//   <plugin filename="legged_actor_pose_follower" name="legged::ActorPoseFollower">
//     <topic>/scenario_person/pose</topic>        gz.msgs.Pose in the world frame; x and y are read
//     <animation_x_vel>4.184</animation_x_vel>    [s/m] animation time per metre: the length of the walk cycle of
//   </plugin>                                     the mesh over the distance its root bone covers in it
//
// As gz-sim FollowActor: the system owns components::TrajectoryPose (x, y, yaw; the height stays in the <pose> of the
// actor) and components::AnimationTime, from which the GUI and the sensors draw the actor.
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector2.hh>
#include <gz/msgs/pose.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Actor.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/transport/Node.hh>

namespace legged {

class ActorPoseFollower : public gz::sim::System,
                          public gz::sim::ISystemConfigure,
                          public gz::sim::ISystemPreUpdate {
 public:
  void Configure(const gz::sim::Entity& entity, const std::shared_ptr<const sdf::Element>& sdf,
                 gz::sim::EntityComponentManager& ecm, gz::sim::EventManager& /*events*/) override {
    const auto* actor = ecm.Component<gz::sim::components::Actor>(entity);
    if (actor == nullptr || actor->Data().AnimationCount() < 1) {
      gzerr << "ActorPoseFollower: not on an actor with an animation, it does nothing" << std::endl;
      return;
    }
    if (!sdf->HasElement("topic") || !sdf->HasElement("animation_x_vel")) {
      gzerr << "ActorPoseFollower: <topic> and <animation_x_vel> are needed, it does nothing" << std::endl;
      return;
    }
    const auto topic = sdf->Get<std::string>("topic");
    animation_x_vel_ = sdf->Get<double>("animation_x_vel");

    // With a name and a time the animation is played at that time, not at the simulation time
    ecm.SetComponentData<gz::sim::components::AnimationName>(entity, actor->Data().AnimationByIndex(0)->Name());
    ecm.SetChanged(entity, gz::sim::components::AnimationName::typeId, gz::sim::ComponentState::OneTimeChange);
    // (not SetComponentData: its comparison does not compile for a std::chrono duration)
    if (ecm.Component<gz::sim::components::AnimationTime>(entity) == nullptr) {
      ecm.CreateComponent(entity, gz::sim::components::AnimationTime());
    }

    // The actor is drawn at Pose * TrajectoryPose: x, y and yaw move to the second, the height stays in the first
    gz::math::Pose3d start;
    if (const auto* pose = ecm.Component<gz::sim::components::Pose>(entity)) {
      start = pose->Data();
    }
    ecm.SetComponentData<gz::sim::components::Pose>(entity, gz::math::Pose3d(0.0, 0.0, start.Pos().Z(), 0.0, 0.0, 0.0));
    ecm.SetComponentData<gz::sim::components::TrajectoryPose>(
        entity, gz::math::Pose3d(start.Pos().X(), start.Pos().Y(), 0.0, 0.0, 0.0, start.Rot().Yaw()));

    if (!node_.Subscribe(topic, &ActorPoseFollower::onPose, this)) {
      gzerr << "ActorPoseFollower: cannot subscribe to [" << topic << "], it does nothing" << std::endl;
      return;
    }
    actor_ = entity;
    gzmsg << "ActorPoseFollower: the actor goes where [" << topic << "] says" << std::endl;
  }

  void PreUpdate(const gz::sim::UpdateInfo& info, gz::sim::EntityComponentManager& ecm) override {
    if (info.paused || actor_ == gz::sim::kNullEntity) {
      return;
    }
    std::optional<gz::math::Vector2d> target;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      target = target_;
    }
    if (!target) {
      return;
    }
    // The first position is a jump from where the world file put the actor, not a walk
    const bool walked = placed_;
    placed_ = true;

    auto* trajectory_pose = ecm.Component<gz::sim::components::TrajectoryPose>(actor_);
    gz::math::Pose3d pose = trajectory_pose->Data();
    const double dx = target->X() - pose.Pos().X();
    const double dy = target->Y() - pose.Pos().Y();
    const double distance = std::hypot(dx, dy);
    if (distance < kMinStep) {
      return;
    }
    pose.Pos().X(target->X());
    pose.Pos().Y(target->Y());
    if (walked) {
      pose.Rot() = gz::math::Quaterniond(0.0, 0.0, std::atan2(dy, dx));
      auto* animation_time = ecm.Component<gz::sim::components::AnimationTime>(actor_);
      animation_time->Data() += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(distance * animation_x_vel_));
      ecm.SetChanged(actor_, gz::sim::components::AnimationTime::typeId, gz::sim::ComponentState::OneTimeChange);
    }
    trajectory_pose->Data() = pose;
    // One-time changes reach the GUI; the sensors read the components on the server
    ecm.SetChanged(actor_, gz::sim::components::TrajectoryPose::typeId, gz::sim::ComponentState::OneTimeChange);
  }

 private:
  void onPose(const gz::msgs::Pose& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    target_ = gz::math::Vector2d(msg.position().x(), msg.position().y());
  }

  static constexpr double kMinStep = 1e-4;  // [m] below this the actor stands: no turn on numerical noise

  gz::sim::Entity actor_{gz::sim::kNullEntity};
  double animation_x_vel_{0.0};
  bool placed_{false};
  gz::transport::Node node_;
  std::mutex mutex_;                           // onPose runs on a gz-transport thread
  std::optional<gz::math::Vector2d> target_;
};

}  // namespace legged

GZ_ADD_PLUGIN(legged::ActorPoseFollower, gz::sim::System, legged::ActorPoseFollower::ISystemConfigure,
              legged::ActorPoseFollower::ISystemPreUpdate)
