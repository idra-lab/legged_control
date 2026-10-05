#!/usr/bin/env python3
"""
far_load_graph.py (ROS 2 Python)

Loads a saved FAR visibility graph (.vgh, written by graph_decoder on /save_file_dir, e.g. at the end of a TARE
exploration by tare_finish_merge_map.py) into a running far_planner, then exits:
  1. waits for far_planner's first /robot_vgraph (it publishes one per loop once it has odometry, so its graph and
     kd-tree are set up) and for graph_decoder on /read_file_dir;
  2. publishes graph_file on /read_file_dir: graph_decoder reads it and publishes it on /decoded_vgraph, which
     far_planner merges into its graph;
  3. logs the number of nodes read and the size of the next /robot_vgraph.

The graph is loaded as saved, in far_planner's world frame (map = camera_init of the session that saved it): it only
lines up when FAST-LIO starts from the same pose as in that session (in the simulation: the robot standing at the spawn).

Usage: ros2 run legged_controllers far_load_graph.py --ros-args -p graph_file:=/path/warehouse.vgh
"""

import os
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from visibility_graph_msg.msg import Graph


class FarLoadGraph(Node):
    def __init__(self):
        super().__init__("far_load_graph")
        self.graph_file = os.path.abspath(os.path.expanduser(self.declare_parameter("graph_file", "").value))
        self.decoder_node = self.declare_parameter("decoder_node", "graph_decoder").value
        self.timeout = self.declare_parameter("timeout", 60.0).value

        self.robot_graph = None
        self.loaded_nodes = None
        self.create_subscription(Graph, "/robot_vgraph", lambda msg: setattr(self, "robot_graph", msg), 5)
        self.create_subscription(Graph, "/decoded_vgraph", self.on_decoded, 5)
        self.read_pub = self.create_publisher(String, "/read_file_dir", 5)
        self.started = time.monotonic()
        self.state = "waiting"  # -> sent -> done

    def on_decoded(self, msg):
        if self.state == "sent":
            self.loaded_nodes = len(msg.nodes)
            self.robot_graph = None  # wait for the graph far_planner publishes after the merge

    def decoder_ready(self):
        return any(info.node_name == self.decoder_node for info in self.get_subscriptions_info_by_topic("/read_file_dir"))

    def tick(self):
        if self.state == "waiting":
            if self.robot_graph is None or not self.decoder_ready():
                if time.monotonic() - self.started > self.timeout:
                    self.get_logger().error(f"no far_planner (/robot_vgraph) or no {self.decoder_node} after "
                                            f"{self.timeout:.0f} s: graph not loaded")
                    self.state = "done"
                return
            self.get_logger().info(f"far_planner up ({len(self.robot_graph.nodes)} nodes): loading {self.graph_file}")
            self.read_pub.publish(String(data=self.graph_file))
            self.state, self.sent_at = "sent", time.monotonic()
        elif self.state == "sent":
            if self.loaded_nodes is not None and self.robot_graph is not None:
                self.get_logger().info(f"{self.loaded_nodes} nodes read from {self.graph_file}, far_planner graph now "
                                       f"{len(self.robot_graph.nodes)} nodes")
                self.state = "done"
            elif time.monotonic() - self.sent_at > 10.0:
                self.get_logger().error(f"{self.decoder_node} did not publish the graph read from {self.graph_file}")
                self.state = "done"


def main():
    rclpy.init()
    node = FarLoadGraph()
    try:
        if not os.path.isfile(node.graph_file) or os.path.getsize(node.graph_file) == 0:
            node.get_logger().error(f"graph file '{node.graph_file}' missing or empty: far_planner starts without it")
            return
        while rclpy.ok() and node.state != "done":
            rclpy.spin_once(node, timeout_sec=0.2)
            node.tick()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
