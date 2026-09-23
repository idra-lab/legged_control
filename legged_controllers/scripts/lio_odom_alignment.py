#!/usr/bin/env python3
"""
lio_odom_alignment.py (ROS 2 Python)

Publishes the odom -> lio_map TF that joins the legged estimator tree (odom -> base, /odom, what OptiPessi tracks) to
the FAST-LIO tree (lio_map -> camera_init -> unilidar_lio -> lio_base, /Odometry, what TARE / FAR plan in).

The two estimators drift apart: the Kalman filter integrates leg kinematics and slips, FAST-LIO does not, and neither
starts exactly where a static TF would assume. A static odom -> lio_map therefore maps a planner waypoint to an odom
goal that is off by that drift. Here, on every /Odometry message, the TF is set so that lio_base (FAST-LIO's base)
lands on base (the estimator's base) at that stamp:

    T_odom_lio_map = T_odom_base(t) * T_lio_map_lio_base(t)^-1,  kept to x, y, z and yaw (both trees are gravity-aligned)

Like a map -> odom correction (REP 105), but pointing the other way so the controller's frame stays the root and /odom
stays smooth. Waypoints relayed through TF (way_point_to_opti_pessi_goal.py) then land where the planner put them
relative to the robot.

T_odom_base comes from /odom matched by stamp, not from the odom -> base TF: that TF is sent by OptiPessi's
visualizer on a wall-clock node, while /odom and /Odometry carry sim time.

Usage: ros2 run legged_controllers lio_odom_alignment.py
"""

import math
from collections import deque

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformBroadcaster, TransformListener


def quat_to_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def to_matrix(translation, rotation):
    m = np.eye(4)
    m[:3, :3] = quat_to_matrix(rotation)
    m[:3, 3] = [translation.x, translation.y, translation.z]
    return m


def invert(m):
    inv = np.eye(4)
    inv[:3, :3] = m[:3, :3].T
    inv[:3, 3] = -m[:3, :3].T @ m[:3, 3]
    return inv


class LioOdomAlignment(Node):
    def __init__(self):
        super().__init__("lio_odom_alignment")
        self.odom_frame = self.declare_parameter("odom_frame", "odom").value
        self.lio_map_frame = self.declare_parameter("lio_map_frame", "lio_map").value
        self.lio_base_frame = self.declare_parameter("lio_base_frame", "lio_base").value
        self.max_stamp_gap = self.declare_parameter("max_stamp_gap", 0.05).value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        # (stamp in ns, /odom pose). /odom comes at up to 200 Hz, so 2 s: more than FAST-LIO's lag behind its stamps.
        self.odom_history = deque(maxlen=400)
        self.create_subscription(Odometry, "/odom", self.on_odom, 50)
        self.create_subscription(Odometry, "/Odometry", self.on_lio_odometry, 5)
        self.published = False

    def on_odom(self, msg: Odometry):
        self.odom_history.append((Time.from_msg(msg.header.stamp).nanoseconds, msg.pose.pose))

    def on_lio_odometry(self, msg: Odometry):
        if not self.odom_history:
            self.get_logger().warn("alignment skipped: no /odom yet", throttle_duration_sec=5.0)
            return
        stamp = Time.from_msg(msg.header.stamp).nanoseconds
        odom_stamp, odom_base = min(self.odom_history, key=lambda s: abs(s[0] - stamp))
        if abs(odom_stamp - stamp) > self.max_stamp_gap * 1e9:
            self.get_logger().warn(f"alignment skipped: nearest /odom is {(odom_stamp - stamp) * 1e-9:+.3f} s from "
                                   f"/Odometry at {stamp * 1e-9:.3f} s (different clocks?)", throttle_duration_sec=5.0)
            return
        try:
            # Statics from fast_lio_slam_launch.xml; the camera_init -> lidar pose is the message itself, so the
            # FAST-LIO TF (sent just after /Odometry) is not needed here.
            lio_map_camera_init = self.tf_buffer.lookup_transform(self.lio_map_frame, msg.header.frame_id, Time())
            lidar_lio_base = self.tf_buffer.lookup_transform(msg.child_frame_id, self.lio_base_frame, Time())
        except Exception as e:
            self.get_logger().warn(f"alignment skipped: {e}", throttle_duration_sec=5.0)
            return

        lio_map_lio_base = (to_matrix(lio_map_camera_init.transform.translation, lio_map_camera_init.transform.rotation)
                            @ to_matrix(msg.pose.pose.position, msg.pose.pose.orientation)
                            @ to_matrix(lidar_lio_base.transform.translation, lidar_lio_base.transform.rotation))
        odom_lio_map = to_matrix(odom_base.position, odom_base.orientation) @ invert(lio_map_lio_base)
        yaw = math.atan2(odom_lio_map[1, 0], odom_lio_map[0, 0])

        tf = TransformStamped()
        tf.header.stamp = msg.header.stamp
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.lio_map_frame
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = odom_lio_map[:3, 3]
        tf.transform.rotation.z = math.sin(yaw / 2)
        tf.transform.rotation.w = math.cos(yaw / 2)
        self.tf_broadcaster.sendTransform(tf)

        if not self.published:
            self.published = True
            x, y, z = odom_lio_map[:3, 3]
            self.get_logger().info(f"{self.odom_frame} -> {self.lio_map_frame}: "
                                   f"({x:.3f}, {y:.3f}, {z:.3f}), yaw {math.degrees(yaw):.2f} deg")


def main():
    rclpy.init()
    node = LioOdomAlignment()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
