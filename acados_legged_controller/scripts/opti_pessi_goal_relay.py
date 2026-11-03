#!/usr/bin/env python3
# Relays the goal of the planner bridges (TARE, FAR, the finish node: /opti_pessi/goal, first ADD marker, frame odom)
# to /goal_pose, the goal AcadosLeggedController takes. A goal is sent only when it moves by more than min_change:
# the controller walks again on every /goal_pose it gets while standing, a reached goal sent again would make it
# step in place.

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


class OptiPessiGoalRelay(Node):
    def __init__(self):
        super().__init__('opti_pessi_goal_relay')
        self.min_change = self.declare_parameter('min_change', 0.02).value
        self.last = None
        self.publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.create_subscription(MarkerArray, '/opti_pessi/goal', self.callback, 10)

    def callback(self, msg):
        marker = next((m for m in msg.markers if m.action == Marker.ADD), None)
        if marker is None:
            return
        # The controller compares the goal with its estimator state, which is in odom
        if marker.header.frame_id != 'odom':
            self.get_logger().error(f"goal in frame '{marker.header.frame_id}', not odom: dropped",
                                    throttle_duration_sec=5.0)
            return
        p = marker.pose.position
        if self.last is not None and math.hypot(p.x - self.last[0], p.y - self.last[1]) < self.min_change:
            return
        self.last = (p.x, p.y)
        goal = PoseStamped(header=marker.header)
        goal.pose = marker.pose
        self.publisher.publish(goal)


def main():
    rclpy.init()
    rclpy.spin(OptiPessiGoalRelay())


if __name__ == '__main__':
    main()
