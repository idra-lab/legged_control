#!/usr/bin/env python3
"""
opti_pessi_scenario.py (ROS 2 Python)

Publishes the goal on /opti_pessi/goal and a moving obstacle on /opti_pessi/obstacles (or, with --person, as a
person in Gazebo) for one of the four benchmark scenarios (opti_pessi_interface/config/scenario_S*.info), with every
distance x10.

The obstacle is also drawn on /opti_pessi/scenario_obstacles (MarkerArray), independent of the controller.

Usage: ros2 run legged_controllers opti_pessi_scenario.py <1|2|3|4> [rate_hz] [--person]
  1 static     obstacle does not move
  2 patrol     obstacle sweeps a 20 m corridor in y below its start, back and forth
  3 circle     obstacle orbits the goal
  4 antagonist obstacle heads for the robot (odom -> base TF)

  --person     the obstacle is the person of the Gazebo world scenario_room (legged_unitree_description
               scenario_room_launch.xml). Its position goes to /scenario_person/pose, where that person walks to, and
               not to /opti_pessi/obstacles: the lidar people pipeline (fast_lio_slam_launch.xml people:=true) has to
               find the person and tell the controller. The node then runs on simulation time.
               The scenario is then laid out in the world of Gazebo, not in odom, which drifts away from it as the
               robot walks (13 % of the way in the simulation): the goal is sent to the controller where the world
               goal is in odom now, and the antagonist heads for the robot where FAST-LIO has it. That needs the
               odom -> lio_map TF of lio_odom_alignment.py; until it is there odom is taken for the world.
"""

import math
import sys

import rclpy
from geometry_msgs.msg import Pose
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from legged_controllers.msg import Obstacle, ObstacleArray

SCALE = 10.0
FRAME = "odom"
BASE_FRAME = "base"
# With --person. FAST-LIO's map is the base when FAST-LIO started: started with the robot standing where
# scenario_room_launch.xml spawns it (the world origin, facing +x), it is the world frame of Gazebo
WORLD_FRAME = "lio_map"
WORLD_BASE_FRAME = "lio_base"
HUMAN_RADIUS = 0.3  # opti_pessi_interface/config/task.info: obstacleTypes.human.radius
# Read in Gazebo by the ActorPoseFollower of the person (bridged by scenario_room_launch.xml): a world position
PERSON_TOPIC = "/scenario_person/pose"

# goal, obstacle start, movement, speed [m/s], direction -- straight from scenario_S*.info
SCENARIOS = {
    1: dict(goal=(1.0, 0.0), obstacle=(0.1, 1.0), movement="straight", speed=0.0, direction=(0.0, -1.0)),
    2: dict(goal=(2.0, 0.2), obstacle=(1.0, 1.0), movement="patrol", speed=0.45, direction=(0.0, -1.0)),
    3: dict(goal=(1.0, 0.2), obstacle=(0.3, 0.15), movement="circle", speed=0.375, direction=(0.0, -1.0)),
    4: dict(goal=(1.0, 0.2), obstacle=(0.8, 0.3), movement="antagonist", speed=0.3, direction=(0.0, -1.0)),
}
PATROL_LENGTH = 2.0 * SCALE


class ScenarioPublisher(Node):
    def __init__(self, scenario: int, rate: float, person: bool = False):
        # The person walks in the time of Gazebo: its speed with respect to the robot must not depend on how fast
        # the simulation runs
        super().__init__("opti_pessi_scenario",
                         parameter_overrides=[Parameter("use_sim_time", value=True)] if person else None)
        self.person = person
        cfg = SCENARIOS[scenario]
        self.movement = cfg["movement"]
        self.speed = cfg["speed"]
        self.goal = (cfg["goal"][0] * SCALE, cfg["goal"][1] * SCALE)
        self.start = (cfg["obstacle"][0] * SCALE, cfg["obstacle"][1] * SCALE)
        self.pos = list(self.start)
        self.direction = list(cfg["direction"])

        dx, dy = self.pos[0] - self.goal[0], self.pos[1] - self.goal[1]
        self.orbit_radius = math.hypot(dx, dy)
        self.phase = math.atan2(dy, dx)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.goal_pub = self.create_publisher(MarkerArray, "/opti_pessi/goal", 1)
        # One or the other: a message on /opti_pessi/obstacles replaces the obstacles of the controller, so next to
        # the person it would erase what the people pipeline detects
        self.obstacle_pub = None if person else self.create_publisher(ObstacleArray, "/opti_pessi/obstacles", 1)
        self.person_pub = self.create_publisher(Pose, PERSON_TOPIC, 1) if person else None
        self.obstacle_viz_pub = self.create_publisher(MarkerArray, "/opti_pessi/scenario_obstacles", 1)

        self.last_time = None
        self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(
            f"S{scenario} ({self.movement}): goal ({self.goal[0]:.1f}, {self.goal[1]:.1f}), "
            f"obstacle ({self.pos[0]:.1f}, {self.pos[1]:.1f}), speed {self.speed} m/s, "
            f"sent to {PERSON_TOPIC if person else '/opti_pessi/obstacles'}")

    def robot_position(self):
        """Where the robot is in the frame of the scenario; None while TF does not know."""
        # With the person that frame is the world, where FAST-LIO has the robot; odom is the world until it has drifted
        frames = [(WORLD_FRAME, WORLD_BASE_FRAME), (FRAME, BASE_FRAME)] if self.person else [(FRAME, BASE_FRAME)]
        for frame, base in frames:
            try:
                tf = self.tf_buffer.lookup_transform(frame, base, Time())
            except Exception:
                continue
            return tf.transform.translation.x, tf.transform.translation.y
        return None

    def in_odom(self, position):
        """A position of the scenario in odom, the frame of the controller."""
        if not self.person:
            return position
        try:
            tf = self.tf_buffer.lookup_transform(FRAME, WORLD_FRAME, Time()).transform
        except Exception:
            self.get_logger().warn(f"no {FRAME} -> {WORLD_FRAME} TF (lio_odom_alignment.py): {FRAME} is taken for the "
                                   "world, the goal is off by the drift of the estimator", throttle_duration_sec=10.0)
            return position
        yaw = 2.0 * math.atan2(tf.rotation.z, tf.rotation.w)   # the two frames are gravity-aligned
        cos, sin = math.cos(yaw), math.sin(yaw)
        return (tf.translation.x + cos * position[0] - sin * position[1],
                tf.translation.y + sin * position[0] + cos * position[1])

    def step(self, dt: float):
        if self.movement == "straight":
            self.pos[0] += self.direction[0] * self.speed * dt
            self.pos[1] += self.direction[1] * self.speed * dt
        elif self.movement == "patrol":
            if self.pos[1] < self.start[1] - PATROL_LENGTH:
                self.direction = [abs(self.direction[0]), abs(self.direction[1])]
            elif self.pos[1] > self.start[1]:
                self.direction = [-abs(self.direction[0]), -abs(self.direction[1])]
            self.pos[0] += self.direction[0] * self.speed * dt
            self.pos[1] += self.direction[1] * self.speed * dt
        elif self.movement == "circle":
            self.phase += self.speed / self.orbit_radius * dt
            self.pos[0] = self.goal[0] + self.orbit_radius * math.cos(self.phase)
            self.pos[1] = self.goal[1] + self.orbit_radius * math.sin(self.phase)
        elif self.movement == "antagonist":
            robot = self.robot_position()
            if robot is None:
                self.get_logger().warn("TF does not know where the robot is, obstacle holds", throttle_duration_sec=5.0)
                return
            vx, vy = robot[0] - self.pos[0], robot[1] - self.pos[1]
            norm = math.hypot(vx, vy)
            if norm > 1e-12:
                self.pos[0] += vx / norm * self.speed * dt
                self.pos[1] += vy / norm * self.speed * dt

    def tick(self):
        now = self.get_clock().now()
        # Nothing has passed at the first tick: on simulation time the clock jumps from 0 to the time Gazebo has run
        dt = 0.0 if self.last_time is None else (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        self.step(dt)
        stamp = now.to_msg()

        marker = Marker()
        marker.header.frame_id = FRAME
        marker.header.stamp = stamp
        marker.ns = "goal"
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x, marker.pose.position.y = self.in_odom(self.goal)
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.3
        marker.color.g = marker.color.a = 1.0
        self.goal_pub.publish(MarkerArray(markers=[marker]))

        if self.person_pub is not None:
            person = Pose()
            person.position.x, person.position.y = self.pos
            self.person_pub.publish(person)
        else:
            obstacles = ObstacleArray()
            obstacles.header.frame_id = FRAME
            obstacles.header.stamp = stamp
            obstacle = Obstacle()
            obstacle.type = Obstacle.HUMAN
            obstacle.position.x, obstacle.position.y = self.pos
            obstacles.obstacles.append(obstacle)
            self.obstacle_pub.publish(obstacles)

        # Drawn from here, not by the controller, so it keeps moving in RViz while the controller is slow or stuck.
        disk = Marker()
        disk.header.frame_id = FRAME
        disk.header.stamp = stamp
        disk.ns = "scenario_obstacle"
        disk.type = Marker.CYLINDER
        disk.action = Marker.ADD
        disk.pose.position.x, disk.pose.position.y = self.in_odom(self.pos)
        disk.pose.position.z = 0.25
        disk.pose.orientation.w = 1.0
        disk.scale.x = disk.scale.y = 2.0 * HUMAN_RADIUS
        disk.scale.z = 0.5
        disk.color.r, disk.color.g, disk.color.b, disk.color.a = 0.9, 0.5, 0.1, 0.6
        self.obstacle_viz_pub.publish(MarkerArray(markers=[disk]))


def parse_arguments(argv):
    """(scenario, rate, person) from the command line without its ROS arguments, None if it names no scenario."""
    args = [arg for arg in argv[1:] if arg != "--person"]
    if not args or not args[0].isdigit() or int(args[0]) not in SCENARIOS:
        return None
    return int(args[0]), float(args[1]) if len(args) > 1 else 20.0, len(args) < len(argv) - 1


def main():
    arguments = parse_arguments(rclpy.utilities.remove_ros_args(sys.argv))
    if arguments is None:
        print(__doc__)
        sys.exit(1)

    rclpy.init()
    node = ScenarioPublisher(*arguments)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
