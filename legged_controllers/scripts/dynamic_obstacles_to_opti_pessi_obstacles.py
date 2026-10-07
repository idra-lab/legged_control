#!/usr/bin/env python3
"""
dynamic_obstacles_to_opti_pessi_obstacles.py (ROS 2 Python)

Relays the people tracked on the lidar by people_perception (/perception/dynamic_obstacles,
people_perception_msgs/DynamicObstacleArray, frame camera_init, one message per scan, an empty one included) to
/opti_pessi/obstacles (legged_controllers/ObstacleArray), the obstacles OptiPessiController keeps away from. The
controller takes them in odom only and does no TF lookup itself, so the positions are transformed to odom here
(odom -> lio_map -> camera_init: lio_odom_alignment.py must be running, as in explore_warehouse_launch.xml).

Only type and position reach the controller. A pedestrian becomes a HUMAN: the radius and the maximum speed are the
ones the controller has for that type (opti_pessi_interface/config/task.info: obstacleTypes), not the ones in the
message.

Each message on /opti_pessi/obstacles replaces the previous one, so this node has to be its only publisher: with
yolo26_rgbd_detect running too, each erases the obstacles of the other.

Usage: ros2 run legged_controllers dynamic_obstacles_to_opti_pessi_obstacles.py
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PointStamped
from legged_controllers.msg import Obstacle, ObstacleArray
from people_perception_msgs.msg import DynamicObstacleArray
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformListener

FRAME = "odom"
INPUT_TOPIC = "/perception/dynamic_obstacles"
OUTPUT_TOPIC = "/opti_pessi/obstacles"

# DynamicObstacle.label is an autoware_perception_msgs/ObjectClassification value; obstacle_adapter sends pedestrians only
PEDESTRIAN = 7
TYPE_OF_LABEL = {PEDESTRIAN: Obstacle.HUMAN}


def to_opti_pessi_obstacles(msg: DynamicObstacleArray, tf, frame):
    """The obstacles of msg for the controller, in frame and at the stamp of msg, and the labels left out because the
    controller has no type for them. tf brings a point of msg to frame; None when msg is in frame already."""
    out = ObstacleArray()
    out.header.stamp = msg.header.stamp
    out.header.frame_id = frame
    unknown = set()
    for obstacle in msg.obstacles:
        if obstacle.label not in TYPE_OF_LABEL:
            unknown.add(obstacle.label)
            continue
        position = obstacle.position
        if tf is not None:
            position = do_transform_point(PointStamped(header=msg.header, point=position), tf).point
        out.obstacles.append(Obstacle(type=TYPE_OF_LABEL[obstacle.label], position=position))
    return out, sorted(unknown)


class DynamicObstaclesRelay(Node):
    def __init__(self):
        super().__init__("dynamic_obstacles_to_opti_pessi_obstacles")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.obstacle_pub = self.create_publisher(ObstacleArray, OUTPUT_TOPIC, 1)
        self.create_subscription(DynamicObstacleArray, INPUT_TOPIC, self.on_obstacles, 5)
        self.relayed = False

    def on_obstacles(self, msg: DynamicObstacleArray):
        tf = None
        if msg.header.frame_id and msg.header.frame_id != FRAME:
            # Latest TF, not the stamp of the scan: odom -> lio_map only follows the slow drift between the two
            # odometries, and its sample for this scan may not be here yet.
            try:
                tf = self.tf_buffer.lookup_transform(FRAME, msg.header.frame_id, Time())
            except Exception as e:
                # Nothing is sent, not an empty array: that would tell the controller there is nobody
                self.get_logger().warn(f"no {FRAME} -> {msg.header.frame_id} TF, obstacles not relayed: {e}",
                                       throttle_duration_sec=5.0)
                return

        obstacles, unknown = to_opti_pessi_obstacles(msg, tf, FRAME)
        if unknown:
            self.get_logger().warn(f"obstacles with label {unknown} left out: the controller has no type for them",
                                   throttle_duration_sec=5.0)
        if self.count_publishers(OUTPUT_TOPIC) > 1:
            self.get_logger().warn(f"{OUTPUT_TOPIC} has another publisher: each message replaces the obstacles of the "
                                   "other one", throttle_duration_sec=10.0)
        self.obstacle_pub.publish(obstacles)

        if not self.relayed:
            self.relayed = True
            self.get_logger().info(f"relaying {INPUT_TOPIC} ({msg.header.frame_id}) to {OUTPUT_TOPIC} ({FRAME})")


def main():
    rclpy.init()
    node = DynamicObstaclesRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
