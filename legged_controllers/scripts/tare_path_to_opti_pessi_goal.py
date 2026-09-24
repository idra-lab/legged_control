#!/usr/bin/env python3
"""
tare_path_to_opti_pessi_goal.py (ROS 2 Python)

Bridge from TARE to OptiPessiController. OptiPessi walks straight to /opti_pessi/goal, with no local planner in between,
so the goal is chosen here and checked against the obstacles of /terrain_map (points more than obstacle_height above
the ground, farther than self_radius from the robot):

  1. /way_point, the paths in path_topics (/exploration_path: local path, /global_path_full: global TSP loop),
     /terrain_map (all in frame map) and the robot (/odom, base) are brought to goal_frame (odom, the only frame the
     controller takes) with the latest TF (odom -> lio_map -> camera_init -> map, see tare_explore_nav_launch.xml);
  2. the candidate goal is the first of
     - midpoint: the middle point between the robot and /way_point, if the straight line robot -> /way_point keeps
       clearance from every obstacle (walking to the midpoint again and again walks that whole line);
     - path: lookahead_distance ahead of the robot along the TARE path /way_point lies on (pure pursuit, see follow()),
       if the line robot -> it keeps clearance: the path goes round walls and through doors, but flickers when TARE
       replans, so it is only taken when the straight way is blocked;
     - detour: a point up to detour_distance away, in the direction of the path goal (or else of the midpoint) turned
       by the smallest multiple of detour_step (up to max_detour_angle each side) that keeps clearance;
     - stop: the robot itself (OptiPessi stands at a reached goal);
  3. the goal follows the candidate, but a candidate that jumps (more than jump_distance in one tick: TARE replanned,
     the path flickered, the choice above changed) is only taken once it has stayed put for switch_time, as long as
     the current goal is still clear and more than reach_distance away;
  4. it is published (one ADD marker in goal_frame, colored by where it comes from) when it has moved more than
     republish_distance.

Without a fresh /terrain_map nothing is checked: the path goal is taken, or else the midpoint. Until /way_point and
/odom are fresh the last goal is kept. kExtendWayPoint must stay false in the TARE config (the path goal needs
/way_point on the path). All distances can be changed at run time (ros2 param set).

The robot comes from /odom by message, not from the odom -> base TF, which OptiPessi's visualizer stamps with wall time.

Usage: ros2 run legged_controllers tare_path_to_opti_pessi_goal.py --ros-args -p clearance:=0.3
"""

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry, Path
from rcl_interfaces.msg import FloatingPointRange, ParameterDescriptor
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

# Projections within this distance of the nearest one are the same place on the path: a waypoint is a path vertex, so
# it touches two segments, and a looped path passes the robot twice.
MATCH_SLACK = 0.15
# Where the robot may be on the path: TARE's path starts at the viewpoint nearest the lidar, up to ~0.6 m from base.
ROBOT_MATCH_RADIUS = 1.0
# Goal marker color (r, g, b) by where the goal comes from
COLORS = {"midpoint": (0.0, 1.0, 0.0), "path": (0.0, 0.5, 1.0), "detour": (1.0, 0.8, 0.0), "stop": (1.0, 0.0, 0.0)}


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

    The robot's projection on the path is the nearest one within max_waypoint_path_distance of the waypoint's along the
    path (a TARE path can loop back past the robot). The goal is lookahead further along the path, never past the
    waypoint, pulled back to the path vertex it would cut past when a vertex in between lies more than
    max_chord_deviation off the straight line robot -> goal.

    Returns the goal (3,), or None if the waypoint is not on path.
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

    # Vertices passed walking min(lookahead, remaining) from the robot's projection, in walking order, and the point
    # reached
    length = min(lookahead, abs(end - start))
    passed = np.flatnonzero((ahead > 0.0) & (ahead < length))
    passed = passed if direction > 0 else passed[::-1]
    candidates = np.vstack([path[passed], point_at(path, vertex_s, start + direction * length)])
    # Farthest candidate whose straight line from the robot stays within max_chord_deviation of the vertices before it
    for k in range(len(candidates) - 1, 0, -1):
        if distance_to_segment(candidates[:k, :2], robot, candidates[k, :2]).max() <= max_chord_deviation:
            return candidates[k]
    return candidates[0]


def clear(robot, goal, obstacles, clearance):
    """Whether the segment robot -> goal (2,) keeps clearance from obstacles (m, 2). Obstacles behind the robot do not
    count: walking away from the robot does not bring it closer to them."""
    ab = goal - robot
    if ab @ ab < 1e-12:
        return True
    ahead = obstacles[(obstacles - robot) @ ab > 0.0]
    return len(ahead) == 0 or bool(distance_to_segment(ahead, robot, goal).min() >= clearance)


def detour(robot, target, obstacles, clearance, distance, step, max_angle):
    """Point at most distance (and at most as far as target (2,)) from robot, in the direction of target turned by the
    smallest multiple of step [rad] up to max_angle each side, whose segment from robot keeps clearance; or None."""
    offset = target - robot
    length = min(np.linalg.norm(offset), distance)
    if length < 1e-6:
        return None
    heading = np.arctan2(offset[1], offset[0])
    for k in range(int(round(max_angle / step)) + 1):
        for sign in (1.0, -1.0) if k else (1.0,):
            angle = heading + sign * k * step
            goal = robot + length * np.array([np.cos(angle), np.sin(angle)])
            if clear(robot, goal, obstacles, clearance):
                return goal
    return None


class TareGoalBridge(Node):
    def __init__(self):
        super().__init__("tare_path_to_opti_pessi_goal")

        def number(name, value, description, low, high):
            descriptor = ParameterDescriptor(description=description,
                                             floating_point_range=[FloatingPointRange(from_value=low, to_value=high)])
            self.declare_parameter(name, value, descriptor)

        number("clearance", 0.3, "distance the line robot -> goal keeps from obstacles [m]", 0.0, 2.0)
        number("obstacle_height", 0.2, "/terrain_map points higher above the ground are obstacles [m]", 0.0, 2.0)
        number("self_radius", 0.3, "obstacles nearer the robot are ignored (the robot's own legs, a wall it "
               "brushes) [m]", 0.0, 1.0)
        number("lookahead_distance", 3.0, "path goal distance ahead of the robot along the TARE path [m]", 0.2, 5.0)
        number("max_chord_deviation", 0.2, "how far path vertices may lie off the line robot -> path goal [m]", 0.0,
               2.0)
        number("waypoint_tolerance", 0.3, "how far /way_point may lie off a path it is taken from [m]", 0.0, 2.0)
        number("max_waypoint_path_distance", 8.0, "farthest /way_point along the path from the robot "
               "(TARE: kLookAheadDistance plus a path segment) [m]", 1.0, 100.0)
        number("detour_distance", 1.0, "farthest detour goal from the robot [m]", 0.2, 5.0)
        number("detour_step", 15.0, "angle between detour directions [deg]", 1.0, 90.0)
        number("max_detour_angle", 90.0, "largest detour turn each side [deg]", 0.0, 180.0)
        number("jump_distance", 0.5, "candidate move in one tick that is a jump [m]", 0.0, 10.0)
        number("switch_time", 1.0, "how long a jumped candidate must stay put before it is taken [s]", 0.0, 10.0)
        number("reach_distance", 0.3, "goal distance at which the next candidate is taken at once [m]", 0.0, 2.0)
        number("republish_distance", 0.1, "goal change that is published [m]", 0.0, 1.0)
        number("input_timeout", 3.0, "age after which a path, /way_point, /odom or /terrain_map is ignored [s]", 0.1,
               60.0)
        self.goal_frame = self.declare_parameter("goal_frame", "odom").value
        self.path_topics = self.declare_parameter("path_topics", ["/exploration_path", "/global_path_full"]).value
        rate = self.declare_parameter("rate", 10.0).value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.goal_pub = self.create_publisher(MarkerArray, "/opti_pessi/goal", 1)

        # Latest message (for /terrain_map: its frame and obstacle points (n, 3)) and receive time per input
        self.inputs = {}

        def keep(key):
            return lambda msg: self.inputs.__setitem__(key, (msg, self.get_clock().now()))

        def keep_obstacles(msg):
            # terrainAnalysis: intensity is the height above the ground
            points = point_cloud2.read_points_numpy(msg, field_names=["x", "y", "z", "intensity"], skip_nans=True)
            obstacles = points[points[:, 3] > self.param("obstacle_height"), :3].astype(float)
            self.inputs["/terrain_map"] = ((msg.header.frame_id, obstacles), self.get_clock().now())

        for topic in self.path_topics:
            self.create_subscription(Path, topic, keep(topic), 1)
        self.create_subscription(PointStamped, "/way_point", keep("/way_point"), 5)
        self.create_subscription(Odometry, "/odom", keep("/odom"), 5)
        self.create_subscription(PointCloud2, "/terrain_map", keep_obstacles, 1)

        self.goal = None  # (2,) in goal_frame, and where it comes from
        self.goal_source = None
        self.published_goal = None
        self.candidate = None  # last candidate, and since when it has not jumped
        self.candidate_since = None
        self.last_way_point = None
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

    def path_goal(self, paths, robot, waypoint):
        """Pure-pursuit goal (2,) along the first path /way_point lies on, or None."""
        for path in paths:
            # Repeated nodes (TARE appends the robot or a loop's start again) make empty segments
            path = path[np.concatenate(([True], np.linalg.norm(np.diff(path[:, :2], axis=0), axis=1) > 1e-6))]
            if len(path) < 2:
                continue
            goal = follow(path, robot, waypoint, self.param("lookahead_distance"), self.param("waypoint_tolerance"),
                          self.param("max_waypoint_path_distance"), self.param("max_chord_deviation"))
            if goal is not None:
                return goal[:2]
        return None

    def choose(self, robot, waypoint, path_goal, obstacles):
        """Candidate goal (2,) and where it comes from (see the module docstring)."""
        midpoint = 0.5 * (robot + waypoint)
        if obstacles is None:
            return (midpoint, "midpoint") if path_goal is None else (path_goal, "path")
        clearance = self.param("clearance")
        if clear(robot, waypoint, obstacles, clearance):
            return midpoint, "midpoint"
        if path_goal is not None and clear(robot, path_goal, obstacles, clearance):
            return path_goal, "path"
        for target in (path_goal, midpoint):
            if target is not None:
                goal = detour(robot, target, obstacles, clearance, self.param("detour_distance"),
                              np.radians(self.param("detour_step")), np.radians(self.param("max_detour_angle")))
                if goal is not None:
                    return goal, "detour"
        return robot, "stop"

    def tick(self):
        odom, way_point = self.fresh("/odom"), self.fresh("/way_point")
        if odom is None or way_point is None:
            self.get_logger().info("waiting for /odom and /way_point", throttle_duration_sec=10.0)
            return
        terrain = self.fresh("/terrain_map")
        p, w = odom.pose.pose.position, way_point.point
        paths = []  # (n, 3) in goal_frame
        obstacles = None
        try:
            robot = self.to_goal_frame(odom.header.frame_id, np.array([[p.x, p.y, p.z]]))[0]
            waypoint = self.to_goal_frame(way_point.header.frame_id, np.array([[w.x, w.y, w.z]]))[0, :2]
            for topic in self.path_topics:
                path_msg = self.fresh(topic)
                if path_msg is not None and len(path_msg.poses) >= 2:
                    path = np.array([[q.pose.position.x, q.pose.position.y, q.pose.position.z]
                                     for q in path_msg.poses])
                    paths.append(self.to_goal_frame(path_msg.header.frame_id, path))
            if terrain is not None:
                obstacles = self.to_goal_frame(*terrain)[:, :2]
        except TransformException as e:
            self.get_logger().warn(f"no TF to {self.goal_frame}, goal kept: {e}", throttle_duration_sec=5.0)
            return
        robot_z, robot = robot[2], robot[:2]

        path_goal = self.path_goal(paths, robot, waypoint)
        if obstacles is None:
            self.get_logger().warn("no /terrain_map, goal not checked for obstacles", throttle_duration_sec=5.0)
        else:
            # Only obstacles a goal (at most /way_point, lookahead_distance, detour_distance or the current goal away)
            # can come near
            reach = max(np.linalg.norm(waypoint - robot), self.param("lookahead_distance"),
                        self.param("detour_distance"),
                        0.0 if self.goal is None else np.linalg.norm(self.goal - robot)) + self.param("clearance")
            distance = np.linalg.norm(obstacles - robot, axis=1)
            obstacles = obstacles[(distance > self.param("self_radius")) & (distance < reach)]
        candidate, source = self.choose(robot, waypoint, path_goal, obstacles)

        # A new TARE waypoint, in TARE's frame: in goal_frame it moves with every odom -> lio_map update
        if self.last_way_point != (round(w.x, 1), round(w.y, 1)):
            self.last_way_point = (round(w.x, 1), round(w.y, 1))
            self.get_logger().info(f"/way_point ({waypoint[0]:.2f}, {waypoint[1]:.2f}), "
                                   f"{np.linalg.norm(waypoint - robot):.2f} m away")

        now = self.get_clock().now()
        if self.candidate is None or np.linalg.norm(candidate - self.candidate) > self.param("jump_distance"):
            self.candidate_since = now
        self.candidate = candidate
        keep_goal = (self.goal is not None
                     and now - self.candidate_since < Duration(seconds=self.param("switch_time"))
                     and np.linalg.norm(self.goal - robot) > self.param("reach_distance")
                     and (obstacles is None or clear(robot, self.goal, obstacles, self.param("clearance"))))
        if not keep_goal:
            if source != self.goal_source:
                self.get_logger().info(f"goal from {source}")
            self.goal, self.goal_source = candidate, source

        if self.published_goal is not None and np.linalg.norm(self.goal - self.published_goal) <= self.param(
                "republish_distance"):
            return
        self.published_goal = self.goal
        marker = Marker()
        marker.header.frame_id = self.goal_frame
        marker.header.stamp = now.to_msg()
        marker.ns = "goal"
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x, marker.pose.position.y = map(float, self.goal)
        marker.pose.position.z = float(robot_z)
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.3
        marker.color.r, marker.color.g, marker.color.b = COLORS[self.goal_source]
        marker.color.a = 1.0
        self.goal_pub.publish(MarkerArray(markers=[marker]))


def main():
    rclpy.init()
    node = TareGoalBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
