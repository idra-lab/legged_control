#!/usr/bin/env python3
"""Paper figures of the obstacle scenarios: path of the robot and of the obstacle, one figure per test.

Usage:
  python3 paper_scen.py [campaign_dir] [--out DIR]

  campaign_dir  folder of record_test.py, one sub-folder per test (default ~/colcon_ws/tests/leonardelli_tests)
  --out         output folder of the figures (default <campaign_dir>/plots)

Figures, in the order of the tests (date of meta.yaml), the scenario name is the --note of the test:
  scenario_<n>_<note>.png / .pdf   one scenario
  scenarios.png / .pdf             the four scenarios side by side

Terminal: per test the distance from the goal at the end, the duration of the test (t_goal -> t_end of meta.yaml),
the number of steps of the robot and the solve time of the MPC (mean, std, 95th percentile).

Path of the robot: base position of /odom (the xy of the CoM is a few mm from it), moving average over SMOOTH s. Dots: one per step of the robot,
the obstacle dots are at the same instants. Steps: from /mpc_stats, dt is the time left of the current step,
a new step starts when it jumps up.
"""

import argparse
import math
import os
import sys

import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from analyze_tests import read_bag, named_series, odom_series  # noqa: E402

ROBOT, OBSTACLE, MARK = '#5cc4c4', '#e64a35', '#1f3fd6'
ROBOT_DOT, OBSTACLE_DOT = '#3aa8a8', '#d63a26'
WALK = 2
STEP_JUMP = 0.05      # s, increase of dt that marks a new step
SMOOTH = 0.3          # s, moving average of the path of the robot (about one step: removes the sway of the trot)


def smooth(t, v, window):
    """Centred moving average over window seconds, edges padded with the first/last value."""
    if t.size < 3:
        return v
    n = max(int(round(window / np.median(np.diff(t)))), 1) | 1
    pad = np.concatenate([np.full(n // 2, v[0]), v, np.full(n // 2, v[-1])])
    return np.convolve(pad, np.ones(n) / n, mode='valid')


def load_test(test_dir):
    with open(os.path.join(test_dir, 'meta.yaml')) as f:
        meta = yaml.safe_load(f)
    data = read_bag(os.path.join(test_dir, 'bag'))
    t0 = meta['t_goal']
    t1 = meta['t_end'] if meta.get('t_end') is not None else math.inf

    odom = odom_series(data['/odom'], t0, t1)
    before = odom_series(data['/odom'], -math.inf, t0)
    start = (before['x'][-1], before['y'][-1]) if before['t'].size else (odom['x'][0], odom['y'][0])
    goal = meta['goal']

    # MPC solves in WALK: steps and computation time
    stats = named_series(data['/mpc_stats'], t0, t1)
    walk = stats['phase'] == WALK if stats['t'].size else np.array([], bool)
    t_walk, dt = stats['t'][walk], stats['dt'][walk]
    new_step = np.where(np.diff(dt) > STEP_JUMP)[0] + 1
    step_times = np.concatenate([t_walk[:1], t_walk[new_step]])
    solve = stats['solve_time_ms'][walk]

    obs = np.array([(t, m.obstacles[0].position.x, m.obstacles[0].position.y)
                    for t, m in data['/opti_pessi/obstacles'] if t0 <= t <= t1 and m.obstacles]).reshape(-1, 3)

    t_last = odom['t'][-1]
    return {
        'name': os.path.basename(test_dir), 'meta': meta, 'note': meta.get('note') or os.path.basename(test_dir),
        'odom': odom, 'start': start, 'goal': goal, 'obs': obs, 'step_times': step_times,
        'final_dist': math.hypot(odom['x'][-1] - goal[0], odom['y'][-1] - goal[1]),
        'duration': (meta['t_end'] if meta.get('t_end') is not None else t_last) - t0,
        'steps': int(step_times.size), 'solve': solve,
    }


def interp_xy(t, x, y, times):
    return np.interp(times, t, x), np.interp(times, t, y)


def fading_line(ax, x, y, color, width, alpha0=0.25, z=2):
    """Path drawn lighter at the start and full colour at the end."""
    from matplotlib.collections import LineCollection
    from matplotlib.colors import to_rgba
    if x.size < 2:
        return
    seg = np.stack([np.column_stack([x[:-1], y[:-1]]), np.column_stack([x[1:], y[1:]])], axis=1)
    rgba = np.tile(to_rgba(color), (seg.shape[0], 1))
    rgba[:, 3] = np.linspace(alpha0, 1.0, seg.shape[0])
    ax.add_collection(LineCollection(seg, colors=rgba, linewidths=width, capstyle='round', zorder=z))


def draw_scenario(ax, s, title, legend=True):
    from matplotlib.lines import Line2D
    from matplotlib.colors import to_rgba
    o, obs, steps = s['odom'], s['obs'], s['step_times']
    px, py = smooth(o['t'], o['x'], SMOOTH), smooth(o['t'], o['y'], SMOOTH)

    # Obstacle
    if obs.shape[0]:
        fading_line(ax, obs[:, 1], obs[:, 2], OBSTACLE, 5.0, alpha0=0.35, z=2)
        ox, oy = interp_xy(obs[:, 0], obs[:, 1], obs[:, 2], steps[(steps >= obs[0, 0]) & (steps <= obs[-1, 0])])
        ax.scatter(ox, oy, s=14, color=OBSTACLE_DOT, zorder=3, linewidths=0)
        ax.plot(obs[0, 1], obs[0, 2], 's', color=OBSTACLE, ms=9, zorder=5)
        ax.plot(obs[-1, 1], obs[-1, 2], 'o', color=OBSTACLE, ms=9, zorder=5)

    # Robot
    fading_line(ax, px, py, ROBOT, 5.0, alpha0=0.2, z=3)
    rx, ry = interp_xy(o['t'], px, py, steps)
    alpha = np.linspace(0.35, 1.0, max(rx.size, 1))
    colors = [(*to_rgba(ROBOT_DOT)[:3], a) for a in alpha]
    ax.scatter(rx, ry, s=18, c=colors, zorder=4, linewidths=0)
    ax.plot(px[-1], py[-1], 'o', color=ROBOT, ms=10, zorder=6)

    ax.plot(*s['start'], 's', color=MARK, ms=8, zorder=7)
    ax.plot(*s['goal'], 'v', color=MARK, ms=9, zorder=7)

    ax.set_aspect('equal', adjustable='datalim')
    ax.autoscale_view()
    ax.margins(0.08)
    ax.grid(True, color='#bdbdbd', linewidth=0.8)
    ax.set_axisbelow(True)
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_title(title)
    if legend:
        handles = [Line2D([], [], color=ROBOT, lw=4, label='Opti-Pessi MPC'),
                   Line2D([], [], color=OBSTACLE, lw=4, label='Obstacle'),
                   Line2D([], [], color=MARK, marker='s', ls='', ms=8, label='Start'),
                   Line2D([], [], color=MARK, marker='v', ls='', ms=9, label='Goal')]
        ax.legend(handles=handles, loc='best', frameon=True, framealpha=0.9)


def plot_all(tests, out_dir):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.size': 13, 'axes.titlesize': 14, 'axes.spines.top': True, 'axes.spines.right': True})
    os.makedirs(out_dir, exist_ok=True)

    titles = [f"Scenario {i + 1}: {s['note']}" for i, s in enumerate(tests)]
    for i, (s, title) in enumerate(zip(tests, titles)):
        fig, ax = plt.subplots(figsize=(6.0, 6.0))
        draw_scenario(ax, s, title)
        fig.tight_layout()
        for ext in ('png', 'pdf'):
            fig.savefig(os.path.join(out_dir, f"scenario_{i + 1}_{s['note']}.{ext}".replace(' ', '_')), dpi=200)
        plt.close(fig)

    cols = min(len(tests), 2)
    rows = math.ceil(len(tests) / cols)
    fig, axes = plt.subplots(rows, cols, figsize=(6.0 * cols, 6.0 * rows), squeeze=False)
    for ax, s, title in zip(axes.flat, tests, titles):
        draw_scenario(ax, s, title)
    for ax in list(axes.flat)[len(tests):]:
        ax.set_visible(False)
    fig.tight_layout()
    for ext in ('png', 'pdf'):
        fig.savefig(os.path.join(out_dir, f'scenarios.{ext}'), dpi=200)
    plt.close(fig)


def print_table(tests):
    header = (f"{'scenario':<22}{'test':<12}{'outcome':<10}{'goal dist [m]':>14}{'duration [s]':>14}"
              f"{'steps':>7}{'solve mean [ms]':>17}{'std [ms]':>10}{'p95 [ms]':>10}")
    print(header)
    print('-' * len(header))
    for i, s in enumerate(tests):
        solve = s['solve']
        mean, std, p95 = ((np.mean(solve), np.std(solve), np.percentile(solve, 95)) if solve.size
                          else (math.nan,) * 3)
        print(f"{f'{i + 1}: ' + s['note']:<22}{s['name']:<12}{s['meta'].get('outcome', ''):<10}"
              f"{s['final_dist']:>14.3f}{s['duration']:>14.2f}{s['steps']:>7d}"
              f"{mean:>17.2f}{std:>10.2f}{p95:>10.2f}")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('campaign', nargs='?', default=os.path.expanduser('~/colcon_ws/tests/leonardelli_tests'))
    parser.add_argument('--out', default=None)
    args = parser.parse_args()

    dirs = [os.path.join(args.campaign, d) for d in sorted(os.listdir(args.campaign))
            if os.path.isfile(os.path.join(args.campaign, d, 'meta.yaml'))
            and os.path.isdir(os.path.join(args.campaign, d, 'bag'))]
    if not dirs:
        sys.exit(f'No test with meta.yaml and bag in {args.campaign}')
    tests = [load_test(d) for d in dirs]
    tests.sort(key=lambda s: str(s['meta'].get('date', '')))

    out = args.out or os.path.join(args.campaign, 'plots')
    plot_all(tests, out)
    print_table(tests)
    print(f'\nFigures in {out}')


if __name__ == '__main__':
    main()
