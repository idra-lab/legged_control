#!/usr/bin/env python3
"""Analysis of the manual tests saved by record_test.py. It only reads the bags: run it again at will.

  ros2 run acados_legged_controller analyze_tests.py ~/colcon_ws/tests/real_final [--no-test-plots]

Writes in the campaign folder:
  tests.csv     one row per test
  summary.csv   one row per obstacle speed
  plots/        comparison of all the tests:
                  1_outcome.png        success rate per obstacle speed
                  2_computation.png    MPC solve time (distribution per speed, mean / p95 / max per test)
                  3_distance.png       minimum distance between the robot body (trunk) and the obstacle
                  4_speed.png          mean and maximum speed, maximum frontal |v_x| and lateral |v_y| speed
                  5_time_to_goal.png   time to reach the goal (successful tests)
                  6_paths.png          paths, one panel per obstacle speed
                  7_solver_failures.png mean number of failed solves per test, per obstacle speed
                  8_torques.png        knee torques (peak, time near the limit, measured / commanded), joint speeds
                  9_straightness.png   distance from the straight line start - goal, yaw drift
                  (8 and 9 group the tests by --note when the tests have one, else by obstacle speed)
  plots/tests/<test>.png   time series of one test: roll, pitch, local v_x, v_y, controls alpha, beta, gamma, dt

Window of the metrics: from the goal of the test (t_goal) to its end (t_end), as detected by record_test.py.
The outcome is the one you confirmed: success or failed (older outcomes such as damping count as failed).
"""

import argparse
import csv
import math
import os
import sys
from collections import defaultdict

import numpy as np
import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

TOPICS = ['/monitor', '/mpc_stats', '/odom', '/opti_pessi/obstacles', '/joint_states']
LEGS = ['LF', 'RF', 'LH', 'RH']
TRUNK = {'trunk_length': 0.647, 'trunk_width': 0.150}   # body of the robot (URDF trunk), if meta.yaml has none

# Colours: status good / critical for the outcome (always with a marker shape too), categorical blue / orange
SUCCESS, FAILED = '#0ca30c', '#d03b3b'
BLUE, ORANGE = '#2a78d6', '#eb6834'
GRAY, INK, INK2 = '#8a8984', '#0b0b0b', '#52514e'
# acados status codes that the controller does not apply (0 success and 2 max iterations are applied)
FAILURE_CODES = {1: 'nan', 3: 'minstep', 4: 'qp'}
FAILURE_STYLE = {'nan': ('#d03b3b', 'NaN (1)'), 'minstep': ('#eb6834', 'min step (3)'),
                 'qp': ('#2a78d6', 'QP failure (4)'), 'other': ('#8a8a8a', 'other')}
OUTCOME_STYLE = {'success': (SUCCESS, 'o'), 'failed': (FAILED, 'X')}


# ---------------------------------------------------------------------------------------------------------------------
# Reading

def stamp(msg, t_bag):
    s = msg.header.stamp if hasattr(msg, 'header') else None
    t = s.sec + s.nanosec * 1e-9 if s is not None else 0.0
    return t if t > 0.0 else t_bag


def read_bag(uri):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=uri, storage_id='mcap'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types() if t.name in TOPICS}
    data = defaultdict(list)
    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        if topic in types:
            msg = deserialize_message(raw, types[topic])
            data[topic].append((stamp(msg, t_ns * 1e-9), msg))
    return data


def named_series(messages, t0, t1):
    """JointState used as a dictionary (/monitor, /mpc_stats): {name: array}, plus 't'."""
    rows = [(t, dict(zip(m.name, m.position))) for t, m in messages if t0 <= t <= t1]
    keys = []
    for _, r in rows:
        keys += [k for k in r if k not in keys]
    out = {'t': np.array([t for t, _ in rows])}
    for key in keys:
        out[key] = np.array([r.get(key, np.nan) for _, r in rows])
    return out


def odom_series(messages, t0, t1):
    """t, x, y, yaw, roll, pitch of /odom (estimated base pose)."""
    rows = []
    for t, m in messages:
        if not t0 <= t <= t1:
            continue
        p, q = m.pose.pose.position, m.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        pitch = math.asin(max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x))))
        roll = math.atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y))
        rows.append((t, p.x, p.y, yaw, roll, pitch))
    a = np.array(rows).reshape(-1, 6)
    return {'t': a[:, 0], 'x': a[:, 1], 'y': a[:, 2], 'yaw': a[:, 3], 'roll': a[:, 4], 'pitch': a[:, 5]}


# ---------------------------------------------------------------------------------------------------------------------
# Metrics

def rect_distance(xb, yb, half_length, half_width):
    """Distance of a point (body frame) from a rectangle centred in the origin, 0 inside."""
    return math.hypot(max(abs(xb) - half_length, 0.0), max(abs(yb) - half_width, 0.0))


def body_distance(odom, obstacles, config, body):
    """Distance between the trunk of the robot and the border of the obstacle, at every pose (nan without obstacle)."""
    types = config['obstacle_types']
    radius_of = {0: types['human']['radius'], 1: types['car']['radius']}
    dist = np.full(odom['t'].size, np.nan)
    if not obstacles:
        return dist
    t_obs = np.array([t for t, _ in obstacles])
    idx = np.searchsorted(t_obs, odom['t'], side='right') - 1     # last obstacle message before each pose
    for i in range(odom['t'].size):
        if idx[i] < 0:
            continue
        c, s = math.cos(odom['yaw'][i]), math.sin(odom['yaw'][i])
        for obs in obstacles[idx[i]][1].obstacles:
            dx, dy = obs.position.x - odom['x'][i], obs.position.y - odom['y'][i]
            d = rect_distance(c * dx + s * dy, -s * dx + c * dy, body['trunk_length'] / 2, body['trunk_width'] / 2)
            d -= radius_of.get(obs.type, types['human']['radius'])
            dist[i] = d if math.isnan(dist[i]) else min(dist[i], d)
    return dist


def r3(x):
    return round(float(x), 3) if x is not None and math.isfinite(x) else math.nan


def analyze_test(test_dir):
    with open(os.path.join(test_dir, 'meta.yaml')) as f:
        meta = yaml.safe_load(f)
    with open(os.path.join(test_dir, meta.get('config', 'config.yaml'))) as f:
        config = yaml.safe_load(f)
    outcome = 'success' if meta.get('outcome') == 'success' else 'failed'
    row = {'test': meta.get('trial', os.path.basename(test_dir)), 'speed': meta.get('speed'), 'rep': meta.get('rep'),
           'mode': meta.get('mode'), 'date': meta.get('date'), 'note': meta.get('note', ''), 'outcome': outcome}
    bag = os.path.join(test_dir, 'bag')
    if not os.path.isdir(bag) or meta.get('t_goal') is None:
        row['note'] = (row['note'] + ' [no bag or no goal]').strip()
        return row, None
    data = read_bag(bag)
    t0 = meta['t_goal']
    t1 = meta['t_end'] if meta.get('t_end') is not None else t0 + 120.0

    odom = odom_series(data['/odom'], t0, t1)
    before = odom_series(data['/odom'], -math.inf, t0)
    start = (before['x'][-1], before['y'][-1]) if before['t'].size else \
        ((odom['x'][0], odom['y'][0]) if odom['t'].size else (math.nan, math.nan))
    goal = meta.get('goal') or [math.nan, math.nan]
    row['direction'] = '+x' if goal[0] >= start[0] else '-x'
    row['duration_s'] = r3(t1 - t0)
    row['time_to_goal_s'] = row['duration_s'] if outcome == 'success' else math.nan

    # Path and speed: mean = path length / time, max = largest |v| of the local velocities of /monitor
    if odom['t'].size > 1:
        path = float(np.sum(np.hypot(np.diff(odom['x']), np.diff(odom['y']))))
        row['path_length_m'] = r3(path)
        row['mean_speed_mps'] = r3(path / max(odom['t'][-1] - odom['t'][0], 1e-6))
        row['final_goal_dist_m'] = r3(math.hypot(odom['x'][-1] - goal[0], odom['y'][-1] - goal[1]))
    mon = named_series(data['/monitor'], t0, t1)
    if mon['t'].size and 'v_x' in mon:
        row['max_speed_mps'] = r3(np.nanmax(np.hypot(mon['v_x'], mon['v_y'])))
        row['max_abs_vx_mps'] = r3(np.nanmax(np.abs(mon['v_x'])))
        row['max_abs_vy_mps'] = r3(np.nanmax(np.abs(mon['v_y'])))

    # Distance robot body - obstacle
    obstacles = [(t, m) for t, m in data['/opti_pessi/obstacles'] if t0 - 1.0 <= t <= t1]
    dist = body_distance(odom, obstacles, config, meta.get('body') or TRUNK)
    row['min_dist_body_m'] = r3(np.nanmin(dist)) if np.isfinite(dist).any() else math.nan
    row['collision'] = bool(np.nanmin(dist) < 0.0) if np.isfinite(dist).any() else ''

    # Joints, while the robot moves (|v| > 0.15 m/s): commanded torque = tau_* of /monitor (feedforward + PD),
    # measured torque = effort of /joint_states (tauEst of the motors, estimated from the current)
    js = [(t, m) for t, m in data.get('/joint_states', []) if t0 <= t <= t1]
    if mon['t'].size and js and 'v_x' in mon:
        names = list(js[0][1].name)
        tj = np.array([t for t, _ in js])
        eff = np.array([m.effort for _, m in js])
        vel = np.array([m.velocity for _, m in js])
        idx = np.minimum(np.searchsorted(tj, mon['t']), len(tj) - 1)
        moving = np.hypot(mon['v_x'], mon['v_y']) > 0.15
        tau_lim = config.get('limits', {}).get('tau_max_knee', 44.4)
        if moving.sum() > 10 and eff.shape[1] == len(names):
            for leg in LEGS:
                n = f'{leg}_KFE'
                if f'tau_{n}' not in mon or n not in names:
                    continue
                cmd = np.abs(mon[f'tau_{n}'][moving])
                meas = np.abs(eff[idx, names.index(n)][moving])
                row[f'knee_p95_{leg}_Nm'] = r3(np.percentile(cmd, 95))
                row[f'knee_sat_{leg}_pct'] = r3(100.0 * np.mean(cmd > 0.95 * tau_lim))
                high = cmd > 15.0
                row[f'knee_ratio_{leg}'] = r3(np.mean(meas[high] / cmd[high])) if high.sum() > 5 else math.nan
            for joint, key in (('HFE', 'hip_speed_rads'), ('KFE', 'knee_speed_rads')):
                cols = [names.index(f'{leg}_{joint}') for leg in LEGS if f'{leg}_{joint}' in names]
                if cols:
                    row[key] = r3(np.mean(np.abs(vel[idx][moving][:, cols])))
            front = [row.get(f'knee_p95_{l}_Nm', math.nan) for l in ('LF', 'RF')]
            rear = [row.get(f'knee_p95_{l}_Nm', math.nan) for l in ('LH', 'RH')]
            row['knee_p95_front_Nm'] = r3(np.nanmax(front)) if np.isfinite(front).any() else math.nan
            row['knee_p95_rear_Nm'] = r3(np.nanmax(rear)) if np.isfinite(rear).any() else math.nan
            sat = [row.get(f'knee_sat_{l}_pct', math.nan) for l in ('LH', 'RH')]
            row['knee_sat_rear_pct'] = r3(np.nanmax(sat)) if np.isfinite(sat).any() else math.nan
            ratio = [row.get(f'knee_ratio_{l}', math.nan) for l in ('LH', 'RH')]
            row['knee_ratio_rear'] = r3(np.nanmin(ratio)) if np.isfinite(ratio).any() else math.nan

    # Straightness: distance from the straight line start - goal, yaw drift from the yaw at the goal while the robot
    # is farther from the goal than heading.blend_dist[1] (there the goal yaw does not count yet)
    if odom['t'].size > 1 and all(math.isfinite(v) for v in (*start, *goal[:2])):
        sx, sy = start
        lx, ly = goal[0] - sx, goal[1] - sy
        length = math.hypot(lx, ly)
        if length > 0.1:
            row['cross_track_max_m'] = r3(np.max(np.abs((odom['x'] - sx) * ly - (odom['y'] - sy) * lx) / length))
            d_far = (config.get('heading', {}).get('blend_dist') or [0.4, 1.2])[1]
            far = np.hypot(odom['x'] - goal[0], odom['y'] - goal[1]) > d_far
            yaw0 = before['yaw'][-1] if before['t'].size else odom['yaw'][0]
            if far.any():
                drift = np.angle(np.exp(1j * (odom['yaw'][far] - yaw0)))
                row['yaw_drift_max_deg'] = r3(np.degrees(np.max(np.abs(drift))))

    # Computation: one /mpc_stats message per solve
    stats = named_series(data['/mpc_stats'], t0, t1)
    solve = stats['solve_time_ms'] if stats['t'].size else np.array([])
    if solve.size:
        row['n_solves'] = int(solve.size)
        row['solve_mean_ms'] = r3(np.mean(solve))
        row['solve_std_ms'] = r3(np.std(solve))
        row['solve_median_ms'] = r3(np.median(solve))
        row['solve_p95_ms'] = r3(np.percentile(solve, 95))
        row['solve_max_ms'] = r3(np.max(solve))
        status = stats['status'].astype(int)
        failed = (status != 0) & (status != 2)   # 0 success, 2 max iterations: both are applied by the controller
        row['solver_failures'] = int(np.count_nonzero(failed))
        row['solver_failure_pct'] = r3(100.0 * np.count_nonzero(failed) / solve.size)
        for code, name in FAILURE_CODES.items():
            row[f'fail_{name}'] = int(np.count_nonzero(status == code))
        row['fail_other'] = int(np.count_nonzero(failed & ~np.isin(status, list(FAILURE_CODES))))
        streak = longest = 0
        for f in failed:
            streak = streak + 1 if f else 0
            longest = max(longest, streak)
        row['fail_longest_streak'] = int(longest)
        row['solve_rate_hz'] = r3(solve.size / max(t1 - t0, 1e-6))

    # Time series of the test: from 1 s before the goal to the end of the recording
    tp0, tp1 = t0 - 1.0, t1 + 5.0
    series = {
        'row': row, 'meta': meta, 'config': config, 't0': t0, 't1': t1, 'start': start, 'goal': goal,
        'solve': solve,
        'odom': odom_series(data['/odom'], tp0, tp1), 'path': odom,
        'mon': named_series(data['/monitor'], tp0, tp1),
        'stats': named_series(data['/mpc_stats'], tp0, tp1),
        'obs': np.array([(t, m.obstacles[0].position.x, m.obstacles[0].position.y)
                         for t, m in obstacles if m.obstacles]).reshape(-1, 3),
    }
    return row, series


# ---------------------------------------------------------------------------------------------------------------------
# Summary

def finite(values):
    return [float(x) for x in values if isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)]


def stat(values, fn):
    v = finite(values)
    return r3(fn(v)) if v else math.nan


def summarize(rows, solves_by_speed):
    groups = defaultdict(list)
    for r in rows:
        groups[r['speed']].append(r)
    summary = []
    for speed in sorted(groups):
        g = groups[speed]
        ok = [r for r in g if r['outcome'] == 'success']
        solve = solves_by_speed.get(speed, np.array([]))
        s = {'speed': speed, 'n_tests': len(g), 'n_success': len(ok), 'n_failed': len(g) - len(ok),
             'success_rate': r3(len(ok) / len(g)),
             'time_to_goal_mean_s': stat([r.get('time_to_goal_s') for r in ok], np.mean),
             'time_to_goal_std_s': stat([r.get('time_to_goal_s') for r in ok], np.std),
             'mean_speed_mean_mps': stat([r.get('mean_speed_mps') for r in g], np.mean),
             'max_speed_mean_mps': stat([r.get('max_speed_mps') for r in g], np.mean),
             'max_speed_max_mps': stat([r.get('max_speed_mps') for r in g], np.max),
             'max_abs_vx_mean_mps': stat([r.get('max_abs_vx_mps') for r in g], np.mean),
             'max_abs_vx_max_mps': stat([r.get('max_abs_vx_mps') for r in g], np.max),
             'max_abs_vy_mean_mps': stat([r.get('max_abs_vy_mps') for r in g], np.mean),
             'max_abs_vy_max_mps': stat([r.get('max_abs_vy_mps') for r in g], np.max),
             'min_dist_body_mean_m': stat([r.get('min_dist_body_m') for r in g], np.mean),
             'min_dist_body_min_m': stat([r.get('min_dist_body_m') for r in g], np.min),
             'n_collisions': sum(1 for r in g if r.get('collision') is True),
             'n_solves': int(solve.size),
             'solve_mean_ms': r3(np.mean(solve)) if solve.size else math.nan,
             'solve_std_ms': r3(np.std(solve)) if solve.size else math.nan,
             'solve_p95_ms': r3(np.percentile(solve, 95)) if solve.size else math.nan,
             'solve_max_ms': r3(np.max(solve)) if solve.size else math.nan,
             'solver_failures': int(sum(r.get('solver_failures') or 0 for r in g)),
             'knee_p95_rear_mean_Nm': stat([r.get('knee_p95_rear_Nm') for r in g], np.mean),
             'knee_sat_rear_mean_pct': stat([r.get('knee_sat_rear_pct') for r in g], np.mean),
             'knee_ratio_rear_mean': stat([r.get('knee_ratio_rear') for r in g], np.mean),
             'hip_speed_mean_rads': stat([r.get('hip_speed_rads') for r in g], np.mean),
             'cross_track_max_mean_m': stat([r.get('cross_track_max_m') for r in g], np.mean),
             'yaw_drift_max_mean_deg': stat([r.get('yaw_drift_max_deg') for r in g], np.mean)}
        summary.append(s)
    return summary


def write_csv(path, rows):
    keys = []
    for r in rows:
        keys += [k for k in r if k not in keys]
    with open(path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)


# ---------------------------------------------------------------------------------------------------------------------
# Plots

def _plt():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({
        'axes.spines.top': False, 'axes.spines.right': False, 'axes.edgecolor': GRAY, 'axes.labelcolor': INK2,
        'xtick.color': INK2, 'ytick.color': INK2, 'axes.titlecolor': INK, 'axes.titlesize': 11,
        'axes.grid': True, 'grid.color': '#e6e5e1', 'grid.linewidth': 0.6, 'lines.linewidth': 1.5,
        'legend.frameon': False, 'font.size': 9})
    return plt


def save(fig, path):
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    import matplotlib.pyplot as plt
    plt.close(fig)


def jitter(speed, i, n, width=0.025):
    return speed + (i - (n - 1) / 2) * (width / max(n - 1, 1)) if n > 1 else speed


def outcome_legend(ax):
    from matplotlib.lines import Line2D
    handles = [Line2D([], [], color=c, marker=m, ls='', markersize=8, label=o) for o, (c, m) in OUTCOME_STYLE.items()]
    ax.legend(handles=handles, loc='best')


def scatter_by_speed(ax, rows, key, ylabel, title, summary_key=None, summary=None):
    """One marker per test (colour + shape = outcome), black bar = mean per speed."""
    groups = defaultdict(list)
    for r in rows:
        groups[r['speed']].append(r)
    for speed, g in groups.items():
        for i, r in enumerate(g):
            v = r.get(key)
            if isinstance(v, (int, float)) and math.isfinite(v):
                c, m = OUTCOME_STYLE[r['outcome']]
                ax.plot(jitter(speed, i, len(g)), v, marker=m, color=c, ls='', markersize=8,
                        markeredgecolor='white', markeredgewidth=0.8, alpha=0.9)
    if summary_key and summary:
        for s in summary:
            v = s.get(summary_key)
            if isinstance(v, (int, float)) and math.isfinite(v):
                ax.plot([s['speed'] - 0.03, s['speed'] + 0.03], [v, v], color=INK, lw=2)
    ax.set_xlabel('obstacle speed [m/s]')
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.set_xticks(sorted(groups))


def plot_campaign(out_dir, rows, summary, solves_by_speed, series_list):
    plt = _plt()
    speeds = [s['speed'] for s in summary]

    # 1. Outcome
    fig, ax = plt.subplots(figsize=(7, 4))
    rates = [100.0 * s['success_rate'] for s in summary]
    ax.bar(speeds, rates, width=0.06, color=SUCCESS, edgecolor='white', linewidth=2)
    for s, rate in zip(summary, rates):
        ax.text(s['speed'], rate + 2, f"{s['n_success']}/{s['n_tests']}", ha='center', color=INK2)
    ax.set_ylim(0, 110)
    ax.set_xticks(speeds)
    ax.set_xlabel('obstacle speed [m/s]')
    ax.set_ylabel('success rate [%]')
    ax.set_title('Success rate (label: successful / total tests)')
    save(fig, os.path.join(out_dir, '1_outcome.png'))

    # 2. Computation
    fig, axes = plt.subplots(1, 2, figsize=(13, 4.5))
    data = [solves_by_speed.get(s, np.array([])) for s in speeds]
    keep = [i for i, d in enumerate(data) if d.size]
    if keep:
        bp = axes[0].boxplot([data[i] for i in keep], positions=[speeds[i] for i in keep], widths=0.05,
                             whis=(5, 95), showfliers=False, patch_artist=True, manage_ticks=False)
        for b in bp['boxes']:
            b.set(facecolor='#cde2fb', edgecolor=BLUE)
        for part in ('whiskers', 'caps'):
            for line in bp[part]:
                line.set_color(BLUE)
        for line in bp['medians']:
            line.set(color=INK, linewidth=2)
        for i in keep:
            axes[0].plot(speeds[i], np.max(data[i]), marker='^', color=ORANGE, ls='', markersize=8)
            axes[0].text(speeds[i], np.max(data[i]), f' {np.mean(data[i]):.1f}', color=INK2, va='bottom')
    axes[0].set_xticks(speeds)
    axes[0].set_xlabel('obstacle speed [m/s]')
    axes[0].set_ylabel('solve time [ms]')
    axes[0].set_title('All solves per speed: box 25-75%, whiskers 5-95%, ▲ max (label: mean)')
    for key, color, marker, label in (('solve_mean_ms', BLUE, 'o', 'mean'), ('solve_p95_ms', ORANGE, 's', 'p95'),
                                      ('solve_max_ms', INK2, '^', 'max')):
        xs = [r['speed'] for r in rows if math.isfinite(r.get(key, math.nan))]
        ys = [r[key] for r in rows if math.isfinite(r.get(key, math.nan))]
        axes[1].plot(xs, ys, marker=marker, color=color, ls='', markersize=8, alpha=0.8, label=label,
                     markeredgecolor='white', markeredgewidth=0.8)
    axes[1].set_xticks(speeds)
    axes[1].set_xlabel('obstacle speed [m/s]')
    axes[1].set_ylabel('solve time [ms]')
    axes[1].set_title('Per test')
    axes[1].legend()
    save(fig, os.path.join(out_dir, '2_computation.png'))

    # 3. Distance body - obstacle
    fig, ax = plt.subplots(figsize=(8, 4.5))
    scatter_by_speed(ax, rows, 'min_dist_body_m', 'min distance [m]',
                     'Minimum distance robot body - obstacle (bar: mean)', 'min_dist_body_mean_m', summary)
    ax.axhline(0.0, color=FAILED, lw=1, ls='--')
    ax.text(ax.get_xlim()[0], 0.0, ' contact', color=FAILED, va='bottom')
    outcome_legend(ax)
    save(fig, os.path.join(out_dir, '3_distance.png'))

    # 4. Speed: mean and maximum, maximum frontal (local x) and lateral (local y) with the limits of the OCP
    lim = series_list[0]['config']['limits'] if series_list else {}
    fig, axes = plt.subplots(2, 2, figsize=(13, 9))
    scatter_by_speed(axes[0, 0], rows, 'mean_speed_mps', 'speed [m/s]', 'Mean speed (path / time, bar: mean)',
                     'mean_speed_mean_mps', summary)
    scatter_by_speed(axes[0, 1], rows, 'max_speed_mps', 'speed [m/s]', 'Maximum speed |v| (bar: mean)',
                     'max_speed_mean_mps', summary)
    scatter_by_speed(axes[1, 0], rows, 'max_abs_vx_mps', 'speed [m/s]',
                     'Maximum frontal speed |v_x| (bar: mean, dashed: limit)', 'max_abs_vx_mean_mps', summary)
    scatter_by_speed(axes[1, 1], rows, 'max_abs_vy_mps', 'speed [m/s]',
                     'Maximum lateral speed |v_y| (bar: mean, dashed: limit)', 'max_abs_vy_mean_mps', summary)
    for ax, key in ((axes[1, 0], 'v_max_x'), (axes[1, 1], 'v_max_y')):
        if key in lim:
            ax.axhline(lim[key], color=GRAY, lw=1, ls='--')
    outcome_legend(axes[0, 0])
    save(fig, os.path.join(out_dir, '4_speed.png'))

    # 5. Time to goal
    fig, ax = plt.subplots(figsize=(10, 6))
    scatter_by_speed(ax, [r for r in rows if r['outcome'] == 'success'], 'time_to_goal_s', 'time [s]',
                     'Time to reach the goal, successful tests (bar: mean)', 'time_to_goal_mean_s', summary)
    save(fig, os.path.join(out_dir, '5_time_to_goal.png'))

    # 6. Paths: one panel per speed
    by_speed = defaultdict(list)
    for s in series_list:
        by_speed[s['row']['speed']].append(s)
    n = len(by_speed)
    if n:
        cols = min(n, 3)
        rows_n = math.ceil(n / cols)
        fig, axes = plt.subplots(rows_n, cols, figsize=(5.5 * cols, 4.2 * rows_n), squeeze=False)
        for ax in axes.flat[n:]:
            ax.set_visible(False)
        for ax, speed in zip(axes.flat, sorted(by_speed)):
            for s in by_speed[speed]:
                c, _ = OUTCOME_STYLE[s['row']['outcome']]
                p = s['path']
                ax.plot(p['x'], p['y'], color=c, lw=1.2, alpha=0.8)
                if p['t'].size:
                    ax.plot(p['x'][-1], p['y'][-1], marker=OUTCOME_STYLE[s['row']['outcome']][1], color=c,
                            markersize=7, ls='')
                if s['obs'].size:
                    ax.plot(s['obs'][:, 1], s['obs'][:, 2], color=GRAY, lw=0.8, alpha=0.6)
                    ax.plot(s['obs'][0, 1], s['obs'][0, 2], marker='o', color=GRAY, markersize=6, ls='')
            ax.set_title(f'obstacle {speed:.2f} m/s')
            ax.set_xlabel('x [m]')
            ax.set_ylabel('y [m]')
            ax.set_aspect('equal', adjustable='datalim')
        from matplotlib.lines import Line2D
        axes.flat[0].legend(handles=[Line2D([], [], color=SUCCESS, label='success'),
                                     Line2D([], [], color=FAILED, label='failed'),
                                     Line2D([], [], color=GRAY, label='obstacle')], loc='best')
        fig.suptitle('Paths of the robot (coloured marker: end of the test, gray dot: obstacle at the goal)')
        save(fig, os.path.join(out_dir, '6_paths.png'))


def plot_solver_failures(path, rows):
    """Mean number of failed solves per test, one bar per obstacle speed."""
    plt = _plt()
    groups = defaultdict(list)
    for r in rows:
        if 'solver_failures' in r:
            groups[r['speed']].append(r['solver_failures'])
    if not groups:
        return
    speeds = sorted(groups)
    means = [float(np.mean(groups[v])) for v in speeds]
    fig, ax = plt.subplots(figsize=(7, 4))
    ax.set_axisbelow(True)
    ax.bar(speeds, means, width=0.06, color=BLUE, edgecolor='white', linewidth=2)
    for v, m in zip(speeds, means):
        ax.text(v, m, f"{m:.1f}\n({len(groups[v])} test{'s' if len(groups[v]) != 1 else ''})", ha='center', va='bottom', color=INK2, fontsize=9)
    ax.set_ylim(0, max(1.0, max(means)) * 1.3)
    ax.set_xticks(speeds)
    ax.set_xlabel('obstacle speed [m/s]')
    ax.set_ylabel('failed solves per test')
    ax.set_title('Mean solver failures per test (status not 0 / 2)')
    save(fig, path)


def group_label(r, by_note):
    return (r.get('note') or '(no note)') if by_note else f"{r['speed']:.2f} m/s"


def scatter_by_group(ax, rows, key, ylabel, title, labels, by_note):
    """Categorical version of scatter_by_speed: one marker per test, black bar = mean of the group."""
    for gx, label in enumerate(labels):
        g = [r for r in rows if group_label(r, by_note) == label]
        vals = []
        for i, r in enumerate(g):
            v = r.get(key)
            if isinstance(v, (int, float)) and math.isfinite(v):
                c, m = OUTCOME_STYLE[r['outcome']]
                ax.plot(jitter(gx, i, len(g), width=0.3), v, marker=m, color=c, ls='', markersize=8,
                        markeredgecolor='white', markeredgewidth=0.8, alpha=0.9)
                vals.append(v)
        if vals:
            ax.plot([gx - 0.25, gx + 0.25], [np.mean(vals)] * 2, color=INK, lw=2)
    ax.set_xticks(range(len(labels)))
    ax.set_xticklabels(labels)
    ax.set_xlim(-0.6, len(labels) - 0.4)
    ax.set_ylabel(ylabel)
    ax.set_title(title)


def plot_joints(out_dir, rows, config):
    plt = _plt()
    rows = [r for r in rows if 'knee_p95_rear_Nm' in r or 'cross_track_max_m' in r]
    if not rows:
        return
    by_note = any(r.get('note') for r in rows)
    labels = []
    for r in rows:
        if group_label(r, by_note) not in labels:
            labels.append(group_label(r, by_note))
    xlabel = 'note of the test (--note)' if by_note else 'obstacle speed'
    tau_lim = config.get('limits', {}).get('tau_max_knee', 44.4) if config else 44.4

    fig, axes = plt.subplots(2, 3, figsize=(16, 9))
    panels = [
        (axes[0, 0], 'knee_p95_front_Nm', 'torque [Nm]', 'Front knees: peak commanded torque (p95, worst leg)'),
        (axes[0, 1], 'knee_p95_rear_Nm', 'torque [Nm]', 'Rear knees: peak commanded torque (p95, worst leg)'),
        (axes[0, 2], 'knee_sat_rear_pct', 'time [%]', f'Rear knees: time above 95% of the limit ({tau_lim:g} Nm)'),
        (axes[1, 0], 'knee_ratio_rear', 'measured / commanded', 'Rear knees: measured / commanded torque (worst leg)'),
        (axes[1, 1], 'hip_speed_rads', 'speed [rad/s]', 'Hips (HFE): mean |joint speed|'),
        (axes[1, 2], 'knee_speed_rads', 'speed [rad/s]', 'Knees (KFE): mean |joint speed|'),
    ]
    for ax, key, ylabel, title in panels:
        scatter_by_group(ax, rows, key, ylabel, title, labels, by_note)
        ax.set_xlabel(xlabel)
    for ax in (axes[0, 0], axes[0, 1]):
        ax.axhline(tau_lim, color=GRAY, lw=1, ls='--')
    axes[1, 0].axhline(1.0, color=GRAY, lw=1, ls='--')
    outcome_legend(axes[0, 0])
    fig.suptitle('Joints while the robot moves (|v| > 0.15 m/s), bar: mean of the group, dashed: limit / ideal')
    save(fig, os.path.join(out_dir, '8_torques.png'))

    fig, axes = plt.subplots(1, 2, figsize=(12, 4.5))
    scatter_by_group(axes[0], rows, 'cross_track_max_m', 'distance [m]',
                     'Max distance from the straight line start - goal', labels, by_note)
    scatter_by_group(axes[1], rows, 'yaw_drift_max_deg', 'yaw [deg]',
                     'Max yaw drift from the start, far from the goal', labels, by_note)
    for ax in axes:
        ax.set_xlabel(xlabel)
    outcome_legend(axes[0])
    save(fig, os.path.join(out_dir, '9_straightness.png'))


def plot_test(path, s):
    plt = _plt()
    t0, t1 = s['t0'], s['t1']
    od, mon, st = s['odom'], s['mon'], s['stats']
    lim = s['config']['limits']
    fig, axes = plt.subplots(4, 2, figsize=(13, 11), sharex=True)
    panels = [
        (axes[0, 0], od['t'], np.degrees(od['roll']), 'roll [deg]', None),
        (axes[0, 1], od['t'], np.degrees(od['pitch']), 'pitch [deg]', None),
        (axes[1, 0], mon['t'], mon.get('v_x'), 'local v_x [m/s]', lim.get('v_max_x')),
        (axes[1, 1], mon['t'], mon.get('v_y'), 'local v_y [m/s]', lim.get('v_max_y')),
        (axes[2, 0], st['t'], st.get('alpha'), 'alpha (CoP position)', None),
        (axes[2, 1], st['t'], st.get('beta'), 'beta (force split x)', None),
        (axes[3, 0], st['t'], st.get('gamma'), 'gamma (force split y)', None),
    ]
    for ax, t, y, title, limit in panels:
        ax.set_title(title)
        if y is None or not np.size(t):
            ax.text(0.5, 0.5, 'not in this bag', transform=ax.transAxes, ha='center', color=INK2)
            continue
        ax.plot(t - t0, y, color=BLUE)
        if limit is not None:
            for v in (limit, -limit):
                ax.axhline(v, color=GRAY, lw=1, ls='--')
    ax = axes[3, 1]
    # Only the next step: the dt of node 0 is fixed to the time left of the step in progress (saw tooth to 0)
    ax.set_title('dt_next: duration of the next step [s] (dashed: dt_min, dt_max)')
    if 'dt_next' in st:
        ax.plot(st['t'] - t0, st['dt_next'], color=BLUE)
        for key in ('dt_min', 'dt_max'):
            if key in lim:
                ax.axhline(lim[key], color=GRAY, lw=1, ls='--')
        bad = (st['status'] != 0) & (st['status'] != 2)
        if bad.any():
            ax.plot(st['t'][bad] - t0, np.zeros(bad.sum()), marker='|', color=FAILED, ls='', markersize=12,
                    transform=ax.get_xaxis_transform(), clip_on=False, label='solver failure')
            ax.legend(loc='right')
    else:
        ax.text(0.5, 0.5, 'not in this bag', transform=ax.transAxes, ha='center', color=INK2)
    for ax in axes.flat:
        ax.axvline(0.0, color=INK2, lw=0.8, ls=':')
        ax.axvline(t1 - t0, color=INK2, lw=0.8, ls=':')
    for ax in axes[3]:
        ax.set_xlabel('time from the goal [s] (dotted: goal, end of the test)')
    r = s['row']
    dist = r.get('min_dist_body_m', math.nan)
    fig.suptitle(f"{r['test']}: obstacle {r['speed']:.2f} m/s, {r['direction']}, {r['outcome']}, "
                 f"{r['duration_s']:.1f} s, min distance body - obstacle {dist:.2f} m, "
                 f"solver failures {r.get('solver_failures', 0)}", color=INK)
    save(fig, path)


# ---------------------------------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('campaign_dir')
    parser.add_argument('--no-test-plots', action='store_true', help='skip the plot of every single test')
    args = parser.parse_args()
    campaign = os.path.abspath(os.path.expanduser(args.campaign_dir))
    tests = sorted(d for d in os.listdir(campaign) if os.path.isfile(os.path.join(campaign, d, 'meta.yaml')))
    if not tests:
        print(f'No test in {campaign} (folders with meta.yaml)')
        return 1

    rows, series_list = [], []
    solves_by_speed = defaultdict(list)
    for name in tests:
        try:
            row, series = analyze_test(os.path.join(campaign, name))
        except Exception as e:   # one broken bag does not stop the analysis
            print(f'  ! {name}: {e}')
            continue
        rows.append(row)
        if series is not None:
            series_list.append(series)
            solves_by_speed[row['speed']].append(series['solve'])
        print(f"  {name}: {row['outcome']}, {row.get('duration_s', math.nan):.1f} s, "
              f"min distance {row.get('min_dist_body_m', math.nan):.2f} m")
    solves_by_speed = {k: np.concatenate(v) if v else np.array([]) for k, v in solves_by_speed.items()}
    rows.sort(key=lambda r: (r['speed'], r['rep'] or 0))
    summary = summarize(rows, solves_by_speed)
    write_csv(os.path.join(campaign, 'tests.csv'), rows)
    write_csv(os.path.join(campaign, 'summary.csv'), summary)

    plots = os.path.join(campaign, 'plots')
    os.makedirs(plots, exist_ok=True)
    plot_campaign(plots, rows, summary, solves_by_speed, series_list)
    plot_solver_failures(os.path.join(plots, '7_solver_failures.png'), rows)
    plot_joints(plots, rows, series_list[0]['config'] if series_list else None)
    if not args.no_test_plots:
        os.makedirs(os.path.join(plots, 'tests'), exist_ok=True)
        for s in series_list:
            plot_test(os.path.join(plots, 'tests', f"{s['row']['test']}.png"), s)

    print(f'\n{"speed":>6} {"tests":>6} {"success":>8} {"time[s]":>8} {"v_mean":>7} {"v_max":>6} '
          f'{"d_min[m]":>9} {"solve[ms]":>10} {"p95":>6}')
    for s in summary:
        print(f"{s['speed']:6.2f} {s['n_tests']:6d} {s['n_success']:4d}/{s['n_tests']:<3d} "
              f"{s['time_to_goal_mean_s']:8.2f} {s['mean_speed_mean_mps']:7.2f} {s['max_speed_mean_mps']:6.2f} "
              f"{s['min_dist_body_min_m']:9.2f} {s['solve_mean_ms']:10.2f} {s['solve_p95_ms']:6.2f}")
    print(f'\ntests.csv, summary.csv and plots/ in {campaign}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
