#!/usr/bin/env python3
"""Checks the simulated RoboSense Airy in a running sim (description launch + robot standing).

Reads --frames consecutive /rslidar_points clouds and 2 s of /rslidar_imu_data, prints each check and exits 1 if any
fails:
  layout    rslidar_sdk XYZIRT fields/offsets, point_step 26, 96 x 900, frame rslidar
  rate      /rslidar_points ~10 Hz, /rslidar_imu_data ~200 Hz
  gap       exactly the frames 10 apart have no valid point in the 180-212 deg wedge (columns 1-79)
  self-hit  no valid point closer than --self-hit-radius to the rslidar origin
  imu       mean linear acceleration close to --expected-acc (default: gravity as the mount, TF base -> rslidar, sees it)
  coverage  the dome looks where its mount says: with the pitch p of the TF base -> rslidar, the lowest return straight
            ahead is p under the horizon and the lowest straight behind p above it, within --pitch-tolerance; and there
            are returns ahead, left, behind and right (needs floor or a wall ahead and walls on every side within 60 m)
"""
import argparse
import sys
import time

import numpy as np
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu, PointCloud2, PointField
from tf2_ros import Buffer, TransformListener

RS_DTYPE = np.dtype({'names': ['x', 'y', 'z', 'intensity', 'ring', 'timestamp'],
                     'formats': ['<f4', '<f4', '<f4', '<f4', '<u2', '<f8'],
                     'offsets': [0, 4, 8, 12, 16, 18], 'itemsize': 26})
RS_FIELDS = [('x', 0, PointField.FLOAT32), ('y', 4, PointField.FLOAT32), ('z', 8, PointField.FLOAT32),
             ('intensity', 12, PointField.FLOAT32), ('ring', 16, PointField.UINT16),
             ('timestamp', 18, PointField.FLOAT64)]
# gz azimuth starts at -180 deg, 0.4 deg per column: columns 0-79 are 180-212 deg. Column 0 lies on the wedge start and
# its float azimuth rounds to 179.99999 deg, outside the wedge, so it is left out.
GAP_COLUMNS = slice(1, 80)


def quat_to_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--frames', type=int, default=20)
    parser.add_argument('--self-hit-radius', type=float, default=0.4)
    parser.add_argument('--expected-acc', type=float, nargs=3, default=None)
    parser.add_argument('--acc-tolerance', type=float, default=0.5)
    parser.add_argument('--pitch-tolerance', type=float, default=1.5, help='[deg]')
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node('airy_sim_check', parameter_overrides=[
        Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    clouds, imus = [], []
    # Reliable, as rslidar_sdk publishes: a best-effort subscription loses most of the ~2.3 MB clouds
    cloud_subscription = node.create_subscription(PointCloud2, '/rslidar_points', clouds.append, 10)
    node.create_subscription(Imu, '/rslidar_imu_data', imus.append, qos_profile_sensor_data)
    tf_buffer = Buffer()
    TransformListener(tf_buffer, node)

    deadline = time.monotonic() + 60.0
    while len(clouds) < args.frames and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    results = []
    if len(clouds) < args.frames:
        print(f'FAIL receive: {len(clouds)}/{args.frames} clouds in 60 s')
        return 1

    first = clouds[0]
    fields = [(f.name, f.offset, f.datatype) for f in first.fields]
    results.append(('layout', fields == RS_FIELDS and first.point_step == 26 and first.height == 96 and
                    first.width == 900 and first.header.frame_id == 'rslidar' and not first.is_dense,
                    f'fields {fields} point_step {first.point_step} {first.height}x{first.width} '
                    f'frame {first.header.frame_id} is_dense {first.is_dense}'))

    stamps = np.array([c.header.stamp.sec + c.header.stamp.nanosec * 1e-9 for c in clouds])
    cloud_hz = (len(stamps) - 1) / (stamps[-1] - stamps[0])

    grids = [np.frombuffer(bytes(c.data), dtype=RS_DTYPE).reshape(c.height, c.width) for c in clouds]
    wedge_valid = [int(np.isfinite(g['x'][:, GAP_COLUMNS]).sum()) for g in grids]
    gap_frames = [i for i, n in enumerate(wedge_valid) if n == 0]
    gap_ok = len(gap_frames) == args.frames // 10 and all(b - a == 10 for a, b in zip(gap_frames, gap_frames[1:]))
    results.append(('gap', gap_ok, f'valid points in wedge per frame {wedge_valid}'))

    last = grids[-1]
    valid = np.isfinite(last['x'])
    xyz = np.stack([last['x'][valid], last['y'][valid], last['z'][valid]], axis=1)
    ranges = np.linalg.norm(xyz, axis=1)
    near = xyz[ranges < args.self_hit_radius]
    results.append(('self-hit', len(near) == 0,
                    f'{len(near)} points within {args.self_hit_radius} m, min range {ranges.min():.3f} m'
                    + (f', mean {near.mean(axis=0).round(3).tolist()}' if len(near) else '')))

    # The IMU on its own: while this process takes in the 2.3 MB clouds it loses best-effort IMU samples
    node.destroy_subscription(cloud_subscription)
    imus.clear()
    imu_deadline = time.monotonic() + 3.0
    while time.monotonic() < imu_deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    if len(imus) < 2:
        print('FAIL receive: no /rslidar_imu_data')
        return 1
    imu_stamps = np.array([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for m in imus])
    imu_hz = (len(imu_stamps) - 1) / (imu_stamps[-1] - imu_stamps[0])
    results.insert(1, ('rate', 9.0 < cloud_hz < 11.0 and 180.0 < imu_hz < 220.0,
                       f'cloud {cloud_hz:.1f} Hz, imu {imu_hz:.1f} Hz (sim time)'))
    acc = np.array([[m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z] for m in imus[-400:]])
    mean_acc = acc.mean(axis=0)

    try:
        tf = tf_buffer.lookup_transform('base', 'rslidar', Time()).transform
        rotation = quat_to_matrix(tf.rotation)
        translation = np.array([tf.translation.x, tf.translation.y, tf.translation.z])
        pitch = np.degrees(np.arcsin(rotation[0, 2]))   # forward pitch of the dome on the base
        # At rest the IMU reads the reaction to gravity, in the frame of the lidar
        expected_acc = np.array(args.expected_acc) if args.expected_acc else rotation.T @ np.array([0.0, 0.0, 9.81])
        results.append(('imu', np.all(np.abs(mean_acc - expected_acc) < args.acc_tolerance) and
                        imus[-1].header.frame_id == 'rslidar',
                        f'mean acc {mean_acc.round(2).tolist()}, expected {expected_acc.round(2).tolist()}, '
                        f'frame {imus[-1].header.frame_id}'))

        in_base = xyz @ rotation.T + translation
        # Seen from the optical centre, in the orientation of the base (level when the robot stands)
        rays = in_base - (translation + rotation @ np.array([0.0, 0.0, 0.04]))
        elevation = np.degrees(np.arctan2(rays[:, 2], np.linalg.norm(rays[:, :2], axis=1)))
        azimuth = np.degrees(np.arctan2(rays[:, 1], rays[:, 0]))
        sides = [int((np.abs((azimuth - centre + 180.0) % 360.0 - 180.0) < 45.0).sum()) for centre in (0, 90, 180, -90)]
        ahead = elevation[np.abs(azimuth) < 5.0]
        behind = elevation[np.abs(azimuth) > 175.0]
        lowest_ahead = ahead.min() if len(ahead) else np.nan
        lowest_behind = behind.min() if len(behind) else np.nan
        results.append(('coverage', abs(lowest_ahead + pitch) < args.pitch_tolerance and
                        abs(lowest_behind - pitch) < args.pitch_tolerance and min(sides) > 0,
                        f'mount pitch {pitch:.1f} deg; lowest return ahead {lowest_ahead:.1f} deg, behind '
                        f'{lowest_behind:.1f} deg from the horizon; returns ahead/left/behind/right {sides}'))
    except Exception as e:  # noqa: BLE001
        results.append(('imu, coverage', False, f'no TF base -> rslidar: {e}'))

    for name, ok, detail in results:
        print(f'{"PASS" if ok else "FAIL"} {name}: {detail}')
    return 0 if all(ok for _, ok, _ in results) else 1


if __name__ == '__main__':
    sys.exit(main())
