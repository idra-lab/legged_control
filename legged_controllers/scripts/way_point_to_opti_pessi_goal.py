#!/usr/bin/env python3
"""
way_point_to_opti_pessi_goal.py (ROS 2 Python)

Relays the autonomy stack's /way_point (geometry_msgs/PointStamped, FAR planner / RViz waypoint tool, frame "map")
to /opti_pessi/goal (visualization_msgs/MarkerArray, one ADD marker). The controller takes the goal in odom only and
does no TF lookup itself, so the point is transformed to odom here (odom -> lio_map -> camera_init -> map, see
far_planner_nav_launch.xml).

Usage: ros2 run legged_controllers way_point_to_opti_pessi_goal.py
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PointStamped
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

FRAME = "odom"


class WayPointRelay(Node):
    def __init__(self):
        super().__init__("way_point_to_opti_pessi_goal")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.goal_pub = self.create_publisher(MarkerArray, "/opti_pessi/goal", 1)
        self.create_subscription(PointStamped, "/way_point", self.on_way_point, 5)
        self.last_goal = None

    def on_way_point(self, msg: PointStamped):
        # Latest TF, not msg stamp: a waypoint from RViz or the planner may be stamped ahead of the odom tree.
        if msg.header.frame_id and msg.header.frame_id != FRAME:
            try:
                tf = self.tf_buffer.lookup_transform(FRAME, msg.header.frame_id, Time())
            except Exception as e:
                self.get_logger().warn(f"no {FRAME} -> {msg.header.frame_id} TF, waypoint dropped: {e}",
                                       throttle_duration_sec=5.0)
                return
            point = do_transform_point(msg, tf).point
        else:
            point = msg.point

        marker = Marker()
        marker.header.frame_id = FRAME
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "goal"
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = point
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.3
        marker.color.g = marker.color.a = 1.0
        self.goal_pub.publish(MarkerArray(markers=[marker]))

        goal = (round(point.x, 2), round(point.y, 2))
        if goal != self.last_goal:
            self.last_goal = goal
            self.get_logger().info(f"goal ({point.x:.2f}, {point.y:.2f}) in {FRAME}")


def main():
    rclpy.init()
    node = WayPointRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
