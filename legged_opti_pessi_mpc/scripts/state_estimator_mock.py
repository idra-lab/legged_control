#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math

class StateEstimatorMock(Node):
    def __init__(self):
        super().__init__('state_estimator_mock')
        self.publisher_ = self.create_publisher(PoseStamped, '/robot/current_state', 10)
        self.timer = self.create_timer(0.05, self.timer_callback) # 20Hz
        self.start_time = self.get_clock().now()
        self.pos_x = 0.0

    def timer_callback(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        
        # Simula un movimento semplice (es. il robot avanza di 0.1 m/s)
        self.pos_x += 0.005 
        
        msg.pose.position.x = self.pos_x
        msg.pose.position.y = 0.0
        msg.pose.position.z = 0.0
        
        # Orientamento neutro (quaternione)
        msg.pose.orientation.w = 1.0
        
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = StateEstimatorMock()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()