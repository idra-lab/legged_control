#!/usr/bin/env python3
# Publishes the TF odom -> base from the /odom topic of the state estimator (needed to see the robot in RViz)
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class OdomToTf(Node):
    def __init__(self):
        super().__init__('odom_to_tf')
        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(Odometry, '/odom', self.callback, 10)

    def callback(self, msg):
        t = TransformStamped()
        t.header = msg.header
        t.child_frame_id = msg.child_frame_id
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.broadcaster.sendTransform(t)


def main():
    rclpy.init()
    rclpy.spin(OdomToTf())


if __name__ == '__main__':
    main()
