#!/usr/bin/env python3
"""
tare_path_to_opti_pessi_goal.py (ROS 2 Python)

Pure-pursuit bridge from TARE to OptiPessiController. OptiPessi walks straight to /opti_pessi/goal, with no local
planner in between, so relaying TARE's /way_point (its lookahead point, up to kLookAheadDistance along the path, round
corners and through doors) sends the robot across walls. Here the goal is a point lookahead_distance ahead of the robot
along the TARE path that leads to /way_point:

  1. the paths in path_topics (/exploration_path: local path, /global_path_full: global TSP loop), /way_point (all in
     frame map) and the robot (/odom, base) are brought to goal_frame (odom, the only frame the controller takes) with
     the latest TF (odom -> lio_map -> camera_init -> map, see tare_explore_nav_launch.xml);
  2. the first path /way_point lies on (within waypoint_tolerance) is followed, from the robot's projection on it
     towards the waypoint's. TARE keeps choosing where to go (viewpoint, forward or backward, local or global path):
     the goal never passes the waypoint. A TARE path can loop back past the robot, so the robot's projection is the
     nearest one within max_waypoint_path_distance of the waypoint along the path (TARE's lookahead is never farther);
  3. the goal is lookahead_distance further along the path, pulled back to the path vertex it would cut past when a
     vertex in between lies more than max_chord_deviation off the straight line robot -> goal (OptiPessi walks that
     line: this keeps it off wall corners and door frames);
  4. it is published (one ADD marker in goal_frame) when it has moved more than republish_distance.

Until /way_point lies on a fresh path (no plan yet, or TARE's next waypoint came before its path) the last goal is
kept: /way_point itself is never relayed, so kExtendWayPoint must stay false in the TARE config. lookahead_distance and
the other distances can be changed at run time (ros2 param set).

Publishes ~/followed_path (nav_msgs/Path, goal_frame): the stretch robot -> /way_point being followed.
The robot comes from /odom by message, not from the odom -> base TF, which OptiPessi's visualizer stamps with wall time.

Usage: ros2 run legged_controllers tare_path_to_opti_pessi_goal.py --ros-args -p lookahead_distance:=1.0
"""

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry, Path
from rcl_interfaces.msg import FloatingPointRange, ParameterDescriptor
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

# Projections within this distance of the nearest one are the same place on the path: a waypoint is a path vertex, so
# it touches two segments, and a looped path passes the robot twice.
MATCH_SLACK = 0.15
# Where the robot may be on the path: TARE's path starts at the viewpoint nearest the lidar, up to ~0.6 m from base.
ROBOT_MATCH_RADIUS = 1.0
# A path that has not matched /way_point for this long is reported (one TARE cycle of mismatch is normal).
HOLD_WARN_TIME = 2.0


def quat_to_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def project(point, path_xy):
    """Distance from point (2,) to each segment of the polyline path_xy (n, 2) and arc length of the projection on it,
    plus the arc length of each vertex. Segments must not be empty."""
    a, ab = path_xy[:-1], np.diff(path_xy, axis=0)
    length = np.linalg.norm(ab, axis=1)
    t = np.clip(np.einsum("ij,ij->i", point - a, ab) / length ** 2, 0.0, 1.0)
    distance = np.linalg.norm(a + t[:, None] * ab - point, axis=1)
    vertex_s = np.concatenate(([0.0], np.cumsum(length)))
    return distance, vertex_s[:-1] + t * length, vertex_s


def point_at(path, vertex_s, s):
    """Point at arc length s along path (n, 3)."""
    i = int(np.clip(np.searchsorted(vertex_s, s, side="right") - 1, 0, len(path) - 2))
    t = np.clip((s - vertex_s[i]) / (vertex_s[i + 1] - vertex_s[i]), 0.0, 1.0)
    return path[i] + t * (path[i + 1] - path[i])


def distance_to_segment(points, a, b):
    """Distance from points (m, 2) to the segment a -> b."""
    ab = b - a
    t = np.clip((points - a) @ ab / max(ab @ ab, 1e-12), 0.0, 1.0)
    return np.linalg.norm(a + t[:, None] * ab - points, axis=1)


def follow(path, robot, waypoint, lookahead, waypoint_tolerance, max_waypoint_path_distance, max_chord_deviation):
    """Pure pursuit along path (n, 3) from robot (2,) towards waypoint (2,).

    Returns (goal (3,), followed stretch robot -> waypoint (m, 3), its length), or None if the waypoint is not on path.
    """
    xy = path[:, :2]
    waypoint_distance, waypoint_s, vertex_s = project(waypoint, xy)
    if waypoint_distance.min() > waypoint_tolerance:
        return None
    waypoint_s = waypoint_s[waypoint_distance <= waypoint_distance.min() + MATCH_SLACK]
    robot_distance, robot_s, _ = project(robot, xy)
    near = robot_distance <= max(robot_distance.min() + MATCH_SLACK, ROBOT_MATCH_RADIUS)
    robot_distance, robot_s = robot_distance[near], robot_s[near]
    # The robot is where the path is nearest, unless that is farther from the waypoint along the path than TARE puts
    # it: a TARE path loops back past the robot, and on its way out or back may pass closer than where TARE starts it.
    gap = np.abs(waypoint_s[None, :] - robot_s[:, None])  # (robot, waypoint) projections
    plausible = np.flatnonzero(gap.min(axis=1) <= max_waypoint_path_distance)
    i = plausible[np.argmin(robot_distance[plausible])] if len(plausible) else np.argmin(gap.min(axis=1))
    start, end = robot_s[i], waypoint_s[np.argmin(gap[i])]
    direction = 1.0 if end >= start else -1.0
    ahead = (vertex_s - start) * direction  # arc length of each vertex ahead of the robot's projection

    def stretch(length):
        """Vertices passed walking length from the robot's projection, in walking order, and the point reached."""
        passed = np.flatnonzero((ahead > 0.0) & (ahead < length))
        passed = passed if direction > 0 else passed[::-1]
        return np.vstack([path[passed], point_at(path, vertex_s, start + direction * length)])

    remaining = abs(end - start)
    candidates = stretch(min(lookahead, remaining))
    # Farthest candidate whose straight line from the robot stays within max_chord_deviation of the vertices before it
    goal = candidates[0]
    for k in range(len(candidates) - 1, 0, -1):
        if distance_to_segment(candidates[:k, :2], robot, candidates[k, :2]).max() <= max_chord_deviation:
            goal = candidates[k]
            break
    followed = np.vstack([point_at(path, vertex_s, start), stretch(remaining)])
    return goal, followed, remaining


class TarePathFollower(Node):
    def __init__(self):
        super().__init__("tare_path_to_opti_pessi_goal")

        def distance(name, value, description, low, high):
            descriptor = ParameterDescriptor(description=description,
                                             floating_point_range=[FloatingPointRange(from_value=low, to_value=high)])
            self.declare_parameter(name, value, descriptor)

        distance("lookahead_distance", 1.0, "goal distance ahead of the robot along the TARE path [m]", 0.2, 5.0)
        distance("max_chord_deviation", 0.2, "how far path vertices may lie off the line robot -> goal [m]", 0.0, 2.0)
        distance("waypoint_tolerance", 0.3, "how far /way_point may lie off a path it is taken from [m]", 0.0, 2.0)
        distance("max_waypoint_path_distance", 8.0, "farthest /way_point along the path from the robot "
                 "(TARE: kLookAheadDistance plus a path segment) [m]", 1.0, 100.0)
        distance("republish_distance", 0.1, "goal change that is published [m]", 0.0, 1.0)
        distance("input_timeout", 3.0, "age after which a path, /way_point or /odom is ignored [s]", 0.1, 60.0)
        self.goal_frame = self.declare_parameter("goal_frame", "odom").value
        self.path_topics = self.declare_parameter("path_topics", ["/exploration_path", "/global_path_full"]).value
        rate = self.declare_parameter("rate", 10.0).value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.goal_pub = self.create_publisher(MarkerArray, "/opti_pessi/goal", 1)
        self.followed_pub = self.create_publisher(Path, "~/followed_path", 1)

        # Latest message and receive time per input
        self.inputs = {}

        def keep(key):
            return lambda msg: self.inputs.__setitem__(key, (msg, self.get_clock().now()))

        for topic in self.path_topics:
            self.create_subscription(Path, topic, keep(topic), 1)
        self.create_subscription(PointStamped, "/way_point", keep("/way_point"), 5)
        self.create_subscription(Odometry, "/odom", keep("/odom"), 5)

        self.last_goal = None
        self.last_source = None
        self.hold_since = None
        self.create_timer(1.0 / rate, self.tick)

    def param(self, name):
        return self.get_parameter(name).value

    def fresh(self, key):
        msg, received = self.inputs.get(key, (None, None))
        if msg is None or self.get_clock().now() - received > Duration(seconds=self.param("input_timeout")):
            return None
        return msg

    def to_goal_frame(self, frame, points):
        """points (n, 3) from frame to goal_frame, with the latest TF (a planner message may be stamped ahead of the
        odom tree)."""
        if not frame or frame == self.goal_frame:
            return points
        tf = self.tf_buffer.lookup_transform(self.goal_frame, frame, Time()).transform
        t = tf.translation
        return points @ quat_to_matrix(tf.rotation).T + np.array([t.x, t.y, t.z])

    def tick(self):
        odom, way_point = self.fresh("/odom"), self.fresh("/way_point")
        if odom is None or way_point is None:
            self.get_logger().info("waiting for /odom and /way_point", throttle_duration_sec=10.0)
            return
        p, w = odom.pose.pose.position, way_point.point
        paths = []  # (topic, path (n, 3) in goal_frame)
        try:
            robot = self.to_goal_frame(odom.header.frame_id, np.array([[p.x, p.y, p.z]]))[0, :2]
            waypoint = self.to_goal_frame(way_point.header.frame_id, np.array([[w.x, w.y, w.z]]))[0, :2]
            for topic in self.path_topics:
                path_msg = self.fresh(topic)
                if path_msg is not None and len(path_msg.poses) >= 2:
                    path = np.array([[q.pose.position.x, q.pose.position.y, q.pose.position.z]
                                     for q in path_msg.poses])
                    paths.append((topic, self.to_goal_frame(path_msg.header.frame_id, path)))
        except TransformException as e:
            self.get_logger().warn(f"no TF to {self.goal_frame}, goal kept: {e}", throttle_duration_sec=5.0)
            return

        result = None
        for topic, path in paths:
            # Repeated nodes (TARE appends the robot or a loop's start again) make empty segments
            path = path[np.concatenate(([True], np.linalg.norm(np.diff(path[:, :2], axis=0), axis=1) > 1e-6))]
            if len(path) >= 2:
                result = follow(path, robot, waypoint, self.param("lookahead_distance"),
                                self.param("waypoint_tolerance"), self.param("max_waypoint_path_distance"),
                                self.param("max_chord_deviation"))
            if result is not None:
                break

        now = self.get_clock().now()
        if result is None:
            if self.hold_since is None:
                self.hold_since = now
            if now - self.hold_since > Duration(seconds=HOLD_WARN_TIME):
                self.get_logger().warn(f"/way_point ({waypoint[0]:.2f}, {waypoint[1]:.2f}) is on no path in "
                                       f"{self.path_topics}, goal kept", throttle_duration_sec=5.0)
            return
        self.hold_since = None
        goal, followed, remaining = result

        # A new TARE waypoint, in TARE's frame: in goal_frame it moves with every odom -> lio_map update
        source = (topic, round(w.x, 1), round(w.y, 1))
        if source != self.last_source:
            self.last_source = source
            self.get_logger().info(f"following {topic} to /way_point ({waypoint[0]:.2f}, {waypoint[1]:.2f}), "
                                   f"{remaining:.2f} m along it")

        stamp = now.to_msg()
        followed_msg = Path()
        followed_msg.header.frame_id = self.goal_frame
        followed_msg.header.stamp = stamp
        for x, y, z in followed:
            pose = PoseStamped()
            pose.header = followed_msg.header
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = float(x), float(y), float(z)
            pose.pose.orientation.w = 1.0
            followed_msg.poses.append(pose)
        self.followed_pub.publish(followed_msg)

        if self.last_goal is not None and np.linalg.norm(goal[:2] - self.last_goal[:2]) <= self.param(
                "republish_distance"):
            return
        self.last_goal = goal
        marker = Marker()
        marker.header.frame_id = self.goal_frame
        marker.header.stamp = stamp
        marker.ns = "goal"
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = map(float, goal)
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.3
        marker.color.g = marker.color.a = 1.0
        self.goal_pub.publish(MarkerArray(markers=[marker]))


def main():
    rclpy.init()
    node = TarePathFollower()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
