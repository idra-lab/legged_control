#!/usr/bin/env python3
"""
tare_finish_merge_map.py (ROS 2 Python)

Ends a TARE exploration unattended. Once TARE reports the exploration finished (/exploration_finish, std_msgs/Bool,
true from "Exploration completed, returning home" on), it
  1. sends SIGINT to the nodes in stop_nodes (TARE and the goal bridges), so the robot does not go on to TARE's
     return home; the ones still alive after flush_time get SIGTERM;
  2. stops the robot: a /opti_pessi/goal at its /odom position (OptiPessi stands at a reached goal), sent again every
     tick until the merge starts, in case a bridge tick raced the SIGINT;
  3. after flush_time, once SC-PGO (pgo_node) has rewritten its pose files (1 Hz) with the last keyframes, runs
     makeMergedMap.py <save_directory> --no-vis --output <map_name> with the python that has open3d, its output going
     to <map_name stem>.log next to the map;
  4. exits when the merge ends (a merge already started when this node is stopped still finishes). FAST-LIO and SC-PGO
     keep running.

Empty save_directory, python and merge_script (the defaults) mean: SC-PGO's save_directory parameter (a relative one
is relative to SC-PGO's working directory), <workspace>/.venv-yolo/bin/python and
<workspace>/src/FAST_LIO_SLAM_ROS2/SC_PGO_ROS2/utils/python/makeMergedMap.py, <workspace> being the first directory
above the legged_controllers install that has a src/.

Usage: ros2 run legged_controllers tare_finish_merge_map.py --ros-args -p map_name:=optimized_map.pcd
"""

import os
import signal
import subprocess
import time
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_prefix
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter_client import AsyncParameterClient
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray


def node_pids(name):
    """PIDs of the processes running node name: started with -r __node:=name (ros2 launch, ros2 run --ros-args), or
    else from an executable or script called name (ros2 run without remapping)."""
    pids = []
    for entry in os.listdir("/proc"):
        if not entry.isdigit() or int(entry) == os.getpid():
            continue
        try:
            with open(f"/proc/{entry}/cmdline", "rb") as f:
                args = f.read().decode(errors="replace").split("\0")
        except OSError:
            continue
        executables = {os.path.basename(arg) for arg in args[:2]}
        if f"__node:={name}" in args or name in executables or f"{name}.py" in executables:
            pids.append(int(entry))
    return pids


def workspace():
    """First directory above the legged_controllers install that has a src/, or None."""
    for parent in Path(get_package_prefix("legged_controllers")).resolve().parents:
        if (parent / "src").is_dir():
            return parent
    return None


class TareFinishMergeMap(Node):
    def __init__(self):
        super().__init__("tare_finish_merge_map")
        self.stop_nodes = self.declare_parameter(
            "stop_nodes", ["tare_planner_node", "tare_path_to_opti_pessi_goal", "way_point_to_opti_pessi_goal"]).value
        self.pgo_node = self.declare_parameter("pgo_node", "alaserPGO").value
        self.save_directory = self.declare_parameter("save_directory", "").value
        self.map_name = self.declare_parameter("map_name", "optimized_map.pcd").value
        self.python = self.declare_parameter("python", "").value
        self.merge_script = self.declare_parameter("merge_script", "").value
        self.flush_time = self.declare_parameter("flush_time", 5.0).value

        root = workspace()
        if root is not None:
            self.python = self.python or str(root / ".venv-yolo" / "bin" / "python")
            self.merge_script = self.merge_script or str(
                root / "src" / "FAST_LIO_SLAM_ROS2" / "SC_PGO_ROS2" / "utils" / "python" / "makeMergedMap.py")
        for name, path in (("python", self.python), ("merge_script", self.merge_script)):
            if not os.path.isfile(path):
                self.get_logger().error(f"{name} '{path}' not found: the map will not be merged")

        self.odom = None
        self.create_subscription(Odometry, "/odom", lambda msg: setattr(self, "odom", msg), 5)
        self.create_subscription(Bool, "/exploration_finish", self.on_exploration_finish, 5)
        self.goal_pub = self.create_publisher(MarkerArray, "/opti_pessi/goal", 1)
        self.pgo_params = AsyncParameterClient(self, self.pgo_node)
        self.save_directory_future = None

        self.state = "exploring"  # -> stopping -> merging -> done
        self.finished_at = None
        self.stop_goal = None
        self.merge = None
        self.map_path = None
        self.create_timer(0.5, self.tick)
        where = self.save_directory or f"{self.pgo_node}'s save_directory"
        self.get_logger().info(f"waiting for /exploration_finish, then merging {self.map_name} in {where}")

    def on_exploration_finish(self, msg):
        if not msg.data or self.state != "exploring":
            return
        self.get_logger().info(f"exploration finished: stopping {', '.join(self.stop_nodes)} and the robot")
        self.state, self.finished_at = "stopping", time.monotonic()
        self.signal_nodes(signal.SIGINT)
        if self.odom is None:
            self.get_logger().warn("no /odom: the robot is not stopped, it walks on to the last goal")
        else:
            marker = Marker()
            marker.header.frame_id = self.odom.header.frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "goal"
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position = self.odom.pose.pose.position
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 0.3
            marker.color.r, marker.color.a = 1.0, 1.0
            self.stop_goal = MarkerArray(markers=[marker])
            self.goal_pub.publish(self.stop_goal)
        if not self.save_directory:
            self.save_directory_future = self.pgo_params.get_parameters(["save_directory"])

    def signal_nodes(self, sig):
        for name in self.stop_nodes:
            for pid in node_pids(name):
                try:
                    os.kill(pid, sig)
                except ProcessLookupError:
                    continue
                self.get_logger().info(f"{signal.Signals(sig).name} sent to {name} (PID {pid})")

    def resolve_save_directory(self):
        """The directory SC-PGO saves to, or None."""
        if self.save_directory:
            return os.path.abspath(os.path.expanduser(self.save_directory))
        future = self.save_directory_future
        if future is None or not future.done() or future.result() is None:
            self.get_logger().error(f"no answer from /{self.pgo_node} about its save_directory (not running? "
                                    "pgo:=false?); set this node's save_directory")
            return None
        value = future.result().values[0]
        if value.type != ParameterType.PARAMETER_STRING:
            self.get_logger().error(f"/{self.pgo_node} has no save_directory parameter")
            return None
        directory = value.string_value
        if not os.path.isabs(directory):
            # Relative to where SC-PGO runs (ros2 launch's working directory), not to where this node runs
            pids = node_pids(self.pgo_node)
            directory = os.path.join(os.readlink(f"/proc/{pids[0]}/cwd") if pids else os.getcwd(), directory)
        return os.path.normpath(directory)

    def start_merge(self):
        directory = self.resolve_save_directory()
        if directory is None:
            return False
        self.map_path = os.path.join(directory, self.map_name)
        log_path = os.path.splitext(self.map_path)[0] + ".log"
        command = [self.python, self.merge_script, directory, "--no-vis", "--output", self.map_name]
        self.get_logger().info(f"merging the map: {' '.join(command)} (output in {log_path})")
        try:
            # Its own session: a Ctrl-C on the launch does not reach it, so a started merge still saves the map
            with open(log_path, "w") as log:
                self.merge = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        except OSError as e:
            self.get_logger().error(f"cannot run the merge: {e}")
            return False
        return True

    def tick(self):
        if self.state == "stopping":
            if self.stop_goal is not None:
                self.goal_pub.publish(self.stop_goal)
            if time.monotonic() - self.finished_at < self.flush_time:
                return
            self.signal_nodes(signal.SIGTERM)  # only the ones SIGINT did not end are still found
            self.state = "merging" if self.start_merge() else "done"
        elif self.state == "merging":
            code = self.merge.poll()
            if code is None:
                return
            if code == 0 and os.path.isfile(self.map_path):
                size = os.path.getsize(self.map_path) / 1e6
                self.get_logger().info(f"map saved: {self.map_path} ({size:.1f} MB)")
            else:
                self.get_logger().error(f"merge failed (exit code {code}), see "
                                        f"{os.path.splitext(self.map_path)[0]}.log")
            self.state = "done"


def main():
    rclpy.init()
    node = TareFinishMergeMap()
    try:
        while rclpy.ok() and node.state != "done":
            rclpy.spin_once(node, timeout_sec=0.5)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node.merge is not None and node.merge.poll() is None:
            node.get_logger().info(f"the merge (PID {node.merge.pid}) goes on in the background: {node.map_path}")
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
