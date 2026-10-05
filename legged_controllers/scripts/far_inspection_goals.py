#!/usr/bin/env python3
"""
far_inspection_goals.py (ROS 2 Python)

Inspection mission: sends the points picked with map_point_picker (selected_points.yaml) to far_planner as goals,
one after the other, in the order of the file:
  1. waits for far_planner's v-graph (/robot_vgraph) and, with wait_for_graph, for the saved graph that
     far_load_graph.py loads (/decoded_vgraph, then one more /robot_vgraph), so that the first route is planned on
     the whole graph and not only on what the robot sees;
  2. publishes goal i on /goal_point and watches it:
       reached: far_planner reports /far_reach_goal_status true, or /Odometry comes within reach_tolerance (xy);
       failed:  far_planner stops publishing /way_point for idle_timeout s (it gives up on an unreachable goal
                without saying so), or the goal takes longer than goal_timeout s.
     far_planner drops goals that arrive before its v-graph is initialized, so a goal that gets no /way_point
     within ack_timeout s is published again;
  3. waits dwell_time s at the point (reached or not) and goes on to the next; with loop it starts over after the
     last one, otherwise it logs a summary and exits.

Frames: the points are in the frame of the map they were picked on (optimized_map.pcd, i.e. camera_init of the
exploration), which is far_planner's map here (static camera_init -> map). Like the loaded graph, they only line
up when FAST-LIO starts from the same pose as in the exploration (in the simulation: the robot standing at the spawn).
The z of the file (ground height under the point) is not used: far_planner compares the 3D distance between
/Odometry (the lidar, about 0.3 m above the floor) and the goal with converge_distance (0.25 m), so each goal is
sent at the robot's current height.

The goals and their status are published on /inspection_goals (MarkerArray, transient local) for RViz.

Usage: ros2 run legged_controllers far_inspection_goals.py --ros-args -p goals_file:=/path/selected_points.yaml
"""

import math
import os

import rclpy
import yaml
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool
from visibility_graph_msg.msg import Graph
from visualization_msgs.msg import Marker, MarkerArray

STATUS_COLORS = {  # rgba
    "pending": (0.85, 0.85, 0.85, 0.9),
    "active": (1.0, 0.31, 0.85, 1.0),
    "reached": (0.29, 0.87, 0.5, 1.0),
    "failed": (1.0, 0.36, 0.36, 1.0),
}


def load_goals(path):
    """Points of a map_point_picker .yaml or .json file (JSON is YAML too), as (x, y, z) tuples."""
    with open(path) as f:
        data = yaml.safe_load(f)
    points = data.get("points") if isinstance(data, dict) else data
    return [(float(p["x"]), float(p["y"]), float(p.get("z", 0.0))) for p in points or []]


class FarInspectionGoals(Node):
    def __init__(self):
        super().__init__("far_inspection_goals")
        # dynamic typing: a launch argument like dwell_time:=3 arrives as an int
        param = lambda name, default: self.declare_parameter(
            name, default, ParameterDescriptor(dynamic_typing=True)).value
        self.goals_file = os.path.abspath(os.path.expanduser(param("goals_file", "")))
        self.frame_id = param("frame_id", "map")
        graph_file = os.path.expanduser(param("graph_file", ""))
        # far_load_graph.py gives up on a missing graph file: nothing to wait for then
        self.wait_for_graph = bool(param("wait_for_graph", True)) and (not graph_file or os.path.isfile(graph_file))
        self.graph_timeout = float(param("graph_timeout", 120.0))
        self.reach_tolerance = float(param("reach_tolerance", 0.3))
        self.dwell_time = float(param("dwell_time", 2.0))
        self.ack_timeout = float(param("ack_timeout", 5.0))
        self.idle_timeout = float(param("idle_timeout", 10.0))
        self.goal_timeout = float(param("goal_timeout", 600.0))
        self.loop = bool(param("loop", False))

        self.goals = []
        self.status = []
        self.index = 0
        self.odom = None
        self.graph_size = 0          # nodes in the last /robot_vgraph
        self.decoded = False         # graph_decoder published the loaded graph
        self.merged = False          # far_planner published a v-graph after that
        self.last_waypoint = None    # ROS time of the last /way_point
        self.reached = False         # far_planner reported the active goal reached
        self.state = "waiting"       # -> active -> dwell -> active ... -> done
        self.started = None
        self.goal_started = None     # first publication of the active goal
        self.sent_at = None          # last publication of the active goal
        self.dwell_until = None

        self.create_subscription(Odometry, "/Odometry", lambda msg: setattr(self, "odom", msg.pose.pose.position), 5)
        self.create_subscription(Graph, "/robot_vgraph", self.on_vgraph, 5)
        self.create_subscription(Graph, "/decoded_vgraph", self.on_decoded, 5)
        self.create_subscription(PointStamped, "/way_point", self.on_waypoint, 5)
        self.create_subscription(Bool, "/far_reach_goal_status", self.on_reach, 5)
        self.goal_pub = self.create_publisher(PointStamped, "/goal_point", 5)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_pub = self.create_publisher(MarkerArray, "/inspection_goals", latched)

    # ------------------------------------------------------------ callbacks

    def on_vgraph(self, msg):
        self.graph_size = len(msg.nodes)
        if self.decoded:
            self.merged = True

    def on_decoded(self, msg):
        self.get_logger().info(f"saved graph loaded by graph_decoder ({len(msg.nodes)} nodes)")
        self.decoded = True

    def on_waypoint(self, msg):
        self.last_waypoint = self.now()

    def on_reach(self, msg):
        # far_planner publishes false on every planning loop with a goal, and true once, on the loop that reaches it
        if msg.data and self.state == "active":
            self.reached = True

    # ------------------------------------------------------------ mission

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def send_goal(self):
        x, y, _ = self.goals[self.index]
        goal = PointStamped()
        goal.header.frame_id = self.frame_id
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.point.x, goal.point.y, goal.point.z = x, y, self.odom.z
        self.goal_pub.publish(goal)
        self.sent_at = self.now()

    def start_goal(self):
        self.status[self.index] = "active"
        self.reached = False
        self.last_waypoint = None
        self.send_goal()
        self.goal_started = self.sent_at
        x, y, _ = self.goals[self.index]
        self.get_logger().info(f"goal {self.index + 1}/{len(self.goals)}: ({x:.2f}, {y:.2f})")
        self.state = "active"
        self.publish_markers()

    def finish_goal(self, status, why):
        self.status[self.index] = status
        x, y, _ = self.goals[self.index]
        text = f"goal {self.index + 1}/{len(self.goals)} ({x:.2f}, {y:.2f}) {status} " \
               f"after {self.now() - self.goal_started:.0f} s: {why}"
        # rclpy refuses two severities from the same line of code
        if status == "reached":
            self.get_logger().info(text)
        else:
            self.get_logger().warning(text)
        self.state, self.dwell_until = "dwell", self.now() + self.dwell_time
        self.publish_markers()

    def tick(self):
        t = self.now()
        if t == 0.0:  # use_sim_time and no /clock yet
            return
        if self.state == "waiting":
            if self.started is None:
                self.started = t
            graph_ready = self.graph_size > 0 and (not self.wait_for_graph or self.merged)
            if self.odom is None or not graph_ready:
                if t - self.started < self.graph_timeout:
                    return
                if self.odom is None or self.graph_size == 0:
                    self.get_logger().error(f"no /Odometry or no far_planner v-graph after {self.graph_timeout:.0f} s: "
                                            "inspection not started")
                    self.state = "done"
                    return
                self.get_logger().warning(f"saved graph not loaded after {self.graph_timeout:.0f} s: "
                                          "starting on far_planner's own graph")
            self.get_logger().info(f"far_planner ready ({self.graph_size} nodes): inspecting {len(self.goals)} points "
                                   f"from {self.goals_file}")
            self.start_goal()
        elif self.state == "active":
            x, y, _ = self.goals[self.index]
            if self.reached:
                self.finish_goal("reached", "far_planner reached the goal")
            elif self.odom is not None and math.hypot(self.odom.x - x, self.odom.y - y) < self.reach_tolerance:
                self.finish_goal("reached", f"within {self.reach_tolerance:.2f} m")
            elif t - self.goal_started > self.goal_timeout:
                self.finish_goal("failed", f"not reached within goal_timeout ({self.goal_timeout:.0f} s)")
            elif self.last_waypoint is None or self.last_waypoint < self.sent_at:
                if t - self.sent_at > self.ack_timeout:  # dropped: v-graph not initialized yet, or no subscriber yet
                    self.send_goal()
            elif t - self.last_waypoint > self.idle_timeout:
                self.finish_goal("failed", f"far_planner stopped planning (no /way_point for {self.idle_timeout:.0f} s): "
                                           "no route found")
        elif self.state == "dwell" and t >= self.dwell_until:
            self.index += 1
            if self.index == len(self.goals):
                self.log_summary()
                if not self.loop:
                    self.state = "done"
                    return
                self.index = 0
                self.status = ["pending"] * len(self.goals)
            self.start_goal()

    def log_summary(self):
        failed = [str(i + 1) for i, s in enumerate(self.status) if s == "failed"]
        reached = len(self.goals) - len(failed)
        self.get_logger().info(f"inspection finished: {reached}/{len(self.goals)} points reached"
                               + (f", failed: {', '.join(failed)}" if failed else ""))

    def publish_markers(self):
        markers = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        for i, ((x, y, z), status) in enumerate(zip(self.goals, self.status)):
            for kind in ("sphere", "label"):
                m = Marker()
                m.header.frame_id, m.header.stamp = self.frame_id, stamp
                m.ns, m.id, m.action = "inspection_" + kind, i, Marker.ADD
                m.pose.position.x, m.pose.position.y = x, y
                m.pose.orientation.w = 1.0
                if kind == "sphere":
                    m.type = Marker.SPHERE
                    m.pose.position.z = z + 0.15
                    m.scale.x = m.scale.y = m.scale.z = 0.4 if status == "active" else 0.3
                    m.color.r, m.color.g, m.color.b, m.color.a = STATUS_COLORS[status]
                else:
                    m.type = Marker.TEXT_VIEW_FACING
                    m.pose.position.z = z + 0.7
                    m.scale.z = 0.4
                    m.text = str(i + 1)
                    m.color.r = m.color.g = m.color.b = m.color.a = 1.0
                markers.markers.append(m)
        self.marker_pub.publish(markers)


def main():
    rclpy.init()
    node = FarInspectionGoals()
    try:
        if not os.path.isfile(node.goals_file):
            node.get_logger().warning(f"goals file '{node.goals_file}' not found: no inspection "
                                      "(far_planner still takes goals from RViz)")
            return
        try:
            node.goals = load_goals(node.goals_file)
        except Exception as e:
            node.get_logger().error(f"cannot read goals file '{node.goals_file}': {e}")
            return
        if not node.goals:
            node.get_logger().warning(f"no points in '{node.goals_file}': no inspection")
            return
        node.status = ["pending"] * len(node.goals)
        node.publish_markers()
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
