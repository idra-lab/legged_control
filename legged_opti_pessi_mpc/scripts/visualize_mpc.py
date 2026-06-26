#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from legged_opti_pessi_mpc.msg import MpcOutput

class MpcVisualizer(Node):
    def __init__(self):
        super().__init__('mpc_visualizer')
        # Publisher per RViz
        self.marker_pub = self.create_publisher(Marker, '/mpc/visualization_marker', 10)
        # Subscriber al topic del nostro solutore
        self.subscription = self.create_subscription(
            MpcOutput, '/mpc/output', self.listener_callback, 10)
        
        self.get_logger().info("Visualizer attivo. In RViz aggiungi Marker sul topic /mpc/visualization_marker")

    def listener_callback(self, msg):
        # self.get_logger().info(f"Ricevuto messaggio con {len(msg.com_trajectory_optimistic)} punti.")
        # Disegna Traiettoria OTTIMISTA (Verde)
        self.publish_line_strip(msg.com_trajectory_optimistic, 0, 0.0, 1.0, 0.0, "optimistic")
        # Disegna Traiettoria PESSIMISTA (Rosso)
        self.publish_line_strip(msg.com_trajectory_pessimistic, 1, 1.0, 0.0, 0.0, "pessimistic")

    def publish_line_strip(self, path, id, r, g, b, ns):
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.03 # Spessore linea
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = 0.8
        
        marker.lifetime = Duration(seconds=0.5).to_msg() # Durata breve per aggiornamento fluido
        
        for point in path:
            p = Point(x=point.x, y=point.y, z=0.05)
            marker.points.append(p)
            
        self.marker_pub.publish(marker)
        

def main(args=None):
    rclpy.init(args=args)
    node = MpcVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()