#!/usr/bin/env python3
"""
opti_pessi_scenario.py (ROS 2 Python)

Publishes the goal on /opti_pessi/goal and a moving obstacle on /opti_pessi/obstacles for one of the
four benchmark scenarios (opti_pessi_interface/config/scenario_S*.info), with every distance x10.

The obstacle is also drawn on /opti_pessi/scenario_obstacles (MarkerArray), independent of the controller.

Usage: ros2 run legged_controllers opti_pessi_scenario.py <1|2|3|4> [rate_hz]
  1 static     obstacle does not move
  2 patrol     obstacle sweeps a 20 m corridor in y below its start, back and forth
  3 circle     obstacle orbits the goal
  4 antagonist obstacle heads for the robot (odom -> base TF)
"""

import math
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from legged_controllers.msg import Obstacle, ObstacleArray

SCALE = 1.0
FRAME = "odom"
BASE_FRAME = "base"
HUMAN_RADIUS = 0.3  # opti_pessi_interface/config/task.info: obstacleTypes.human.radius

# goal, obstacle start, movement, speed [m/s], direction -- straight from scenario_S*.info
SCENARIOS = {
    1: dict(goal=(10.0, 0.0), obstacle=(5.0, 1.0), movement="straight", speed=0.0, direction=(0.0, -1.0)),
    2: dict(goal=(10.0, 0.2), obstacle=(5.0, 1.0), movement="patrol", speed=0.25, direction=(0.0, -1.0)),
    3: dict(goal=(1.0, 0.2), obstacle=(0.0, 1.0), movement="circle", speed=0.375, direction=(0.0, -1.0)),
    4: dict(goal=(1.0, 0.2), obstacle=(0.3, 0.8), movement="antagonist", speed=0.25, direction=(0.0, -1.0)),
}
PATROL_LENGTH = 2.0 * SCALE


class ScenarioPublisher(Node):
    def __init__(self, scenario: int, rate: float):
        super().__init__("opti_pessi_scenario")
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
        self.obstacle_pub = self.create_publisher(ObstacleArray, "/opti_pessi/obstacles", 1)
        self.obstacle_viz_pub = self.create_publisher(MarkerArray, "/opti_pessi/scenario_obstacles", 1)

        self.last_time = self.get_clock().now()
        self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(
            f"S{scenario} ({self.movement}): goal ({self.goal[0]:.1f}, {self.goal[1]:.1f}), "
            f"obstacle ({self.pos[0]:.1f}, {self.pos[1]:.1f}), speed {self.speed} m/s")

    def robot_position(self):
        try:
            tf = self.tf_buffer.lookup_transform(FRAME, BASE_FRAME, Time())
        except Exception:
            return None
        return tf.transform.translation.x, tf.transform.translation.y

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
                self.get_logger().warn(f"no {FRAME} -> {BASE_FRAME} TF, obstacle holds", throttle_duration_sec=5.0)
                return
            vx, vy = robot[0] - self.pos[0], robot[1] - self.pos[1]
            norm = math.hypot(vx, vy)
            if norm > 1e-12:
                self.pos[0] += vx / norm * self.speed * dt
                self.pos[1] += vy / norm * self.speed * dt

    def tick(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        self.step(dt)
        stamp = now.to_msg()

        marker = Marker()
        marker.header.frame_id = FRAME
        marker.header.stamp = stamp
        marker.ns = "goal"
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x, marker.pose.position.y = self.goal
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.3
        marker.color.g = marker.color.a = 1.0
        self.goal_pub.publish(MarkerArray(markers=[marker]))

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
        disk.pose.position.x, disk.pose.position.y = self.pos
        disk.pose.position.z = 0.25
        disk.pose.orientation.w = 1.0
        disk.scale.x = disk.scale.y = 2.0 * HUMAN_RADIUS
        disk.scale.z = 0.5
        disk.color.r, disk.color.g, disk.color.b, disk.color.a = 0.9, 0.5, 0.1, 0.6
        self.obstacle_viz_pub.publish(MarkerArray(markers=[disk]))


def main():
    args = rclpy.utilities.remove_ros_args(sys.argv)
    if len(args) < 2 or not args[1].isdigit() or int(args[1]) not in SCENARIOS:
        print(__doc__)
        sys.exit(1)
    rate = float(args[2]) if len(args) > 2 else 20.0

    rclpy.init()
    node = ScenarioPublisher(int(args[1]), rate)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
