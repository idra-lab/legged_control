#!/usr/bin/env python3
"""Smoke test of airy_points_relay without Gazebo.

Needs, each in its own terminal (after sourcing install/setup.bash):
  ros2 run tf2_ros static_transform_publisher --z 0.04 --frame-id rslidar --child-frame-id rslidar_optical
  ros2 run legged_gazebo airy_points_relay
Publishes a 1 x 3 Gazebo-like cloud (x y z intensity FLOAT32, ring UINT16, point_step 32) on /rslidar/points_optical
at 10 Hz and checks the first /rslidar_points that comes back. Exit code 0 on success.
"""
import math
import sys

import numpy as np
import rclpy
from sensor_msgs.msg import PointCloud2, PointField

GZ_DTYPE = np.dtype({'names': ['x', 'y', 'z', 'intensity', 'ring'],
                     'formats': ['<f4', '<f4', '<f4', '<f4', '<u2'],
                     'offsets': [0, 4, 8, 12, 16], 'itemsize': 32})
RS_DTYPE = np.dtype({'names': ['x', 'y', 'z', 'intensity', 'ring', 'timestamp'],
                     'formats': ['<f4', '<f4', '<f4', '<f4', '<u2', '<f8'],
                     'offsets': [0, 4, 8, 12, 16, 18], 'itemsize': 26})
RS_FIELDS = [('x', 0, PointField.FLOAT32), ('y', 4, PointField.FLOAT32), ('z', 8, PointField.FLOAT32),
             ('intensity', 12, PointField.FLOAT32), ('ring', 16, PointField.UINT16),
             ('timestamp', 18, PointField.FLOAT64)]


def make_input(stamp):
    points = np.zeros(3, dtype=GZ_DTYPE)
    points[0] = (1.0, 0.0, 0.0, 5.0, 7)
    points[1] = (math.inf, math.inf, math.inf, 0.0, 8)
    points[2] = (0.0, 2.0, 1.0, 9.0, 95)
    msg = PointCloud2()
    msg.header.stamp = stamp
    msg.header.frame_id = 'rslidar_optical'
    msg.height, msg.width = 1, 3
    msg.fields = [PointField(name=n, offset=o, datatype=t, count=1) for n, o, t in RS_FIELDS[:5]]
    msg.point_step, msg.row_step = 32, 96
    msg.is_dense = False
    msg.data = points.tobytes()
    return msg


def check(out):
    errors = []
    fields = [(f.name, f.offset, f.datatype) for f in out.fields]
    if fields != RS_FIELDS:
        errors.append(f'fields {fields}')
    if (out.point_step, out.height, out.width, out.header.frame_id) != (26, 1, 3, 'rslidar'):
        errors.append(f'point_step/height/width/frame {out.point_step} {out.height} {out.width} {out.header.frame_id}')
    points = np.frombuffer(bytes(out.data), dtype=RS_DTYPE)
    stamp = out.header.stamp.sec + out.header.stamp.nanosec * 1e-9
    p0, p1, p2 = points
    if not np.allclose([p0['x'], p0['y'], p0['z']], [1.0, 0.0, 0.04], atol=1e-5):
        errors.append(f'point 0 {p0}')
    if (p0['intensity'], p0['ring']) != (5.0, 7) or abs(p0['timestamp'] - stamp) > 1e-6:
        errors.append(f'point 0 intensity/ring/timestamp {p0}')
    if not all(math.isnan(p1[k]) for k in ('x', 'y', 'z')) or p1['ring'] != 8:
        errors.append(f'point 1 {p1}')
    if not np.allclose([p2['x'], p2['y'], p2['z']], [0.0, 2.0, 1.04], atol=1e-5) or p2['ring'] != 95:
        errors.append(f'point 2 {p2}')
    return errors


def main():
    rclpy.init()
    node = rclpy.create_node('airy_relay_smoke')
    received = []
    # Reliable on both sides, as ros_gz_bridge publishes the Gazebo cloud and rslidar_sdk publishes /rslidar_points
    node.create_subscription(PointCloud2, '/rslidar_points', received.append, 10)
    publisher = node.create_publisher(PointCloud2, '/rslidar/points_optical', 10)
    node.create_timer(0.1, lambda: publisher.publish(make_input(node.get_clock().now().to_msg())))
    deadline = node.get_clock().now().nanoseconds + 5e9
    while not received and node.get_clock().now().nanoseconds < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not received:
        print('FAIL: no /rslidar_points within 5 s')
        return 1
    errors = check(received[0])
    print('FAIL: ' + '; '.join(errors) if errors else 'PASS')
    return 1 if errors else 0


if __name__ == '__main__':
    sys.exit(main())
