#!/usr/bin/env python3
"""Records ONE manual test, repeatable: same goal, same obstacle start and speed at every repetition.

  1. you bring the robot to the start (STANCE), obstacle_simulator already running
  2. record_test.py starts the bag, puts the obstacle at its start (paused) and prints "PRONTO"
  3. with --goal: Enter sends the goal and starts the obstacle at the same instant
     without --goal: send the goal from RViz, the obstacle starts when the goal arrives
  4. it stops by itself when the robot reaches the goal or the controller dies (Power Protection, controller error);
     Enter stops it earlier (a fall, a test to stop). The obstacle is paused again.
  5. it asks the outcome, success or failed (the detected one is the default), and saves everything in
       ~/colcon_ws/tests/<campaign>/v<speed>_r<NN>/   (bag/, meta.yaml, config.yaml)
Then: ros2 run acados_legged_controller analyze_tests.py ~/colcon_ws/tests/<campaign>

Goal in the odom frame of the controller (x, y in m, yaw in DEGREES, optional). Obstacle: first obstacle of
config.yaml, start = --obstacle-start or its pos, speed = --speed or its speed; pos_end stays the one of config.yaml.

  ros2 run acados_legged_controller record_test.py --campaign real_final --goal 4.0 0.0
  ros2 run acados_legged_controller record_test.py --campaign real_final --goal 0.0 0.0 180 --speed 0.4
  ros2 run acados_legged_controller record_test.py --campaign real_final --goal 4.0 0.0 --obstacle-start 2.0 0.8
  ros2 run acados_legged_controller record_test.py --campaign real_final            (goal from RViz)
  ros2 run acados_legged_controller record_test.py --campaign sim_test --sim --goal 4.0 0.0
"""

import argparse
import math
import os
import select
import shutil
import signal
import subprocess
import sys
import threading
import time
from datetime import datetime

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import SetParameters
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool, Trigger

PKG = 'acados_legged_controller'
RECORDER_NODE = 'test_recorder'
CONTROLLER_NODE = 'acados_legged_controller'
OBSTACLE_NODE = '/obstacle_simulator'
STANCE, WALK = 1, 2
# No /rosout: it is transient local, a new subscriber gets its whole history and that burst stalled the MPC thread
TOPICS = ['/monitor', '/mpc_stats', '/odom', '/joint_states', '/imu_sensor_broadcaster/imu', '/goal_pose',
          '/opti_pessi/obstacles', '/mpc_predictions', '/mpc_obstacles', '/tf', '/tf_static']
# The recorder subscribes to a topic only once it has a publisher: required are the topics of the controller and
# /goal_pose (published by this node), plus the obstacles if obstacle_simulator is running
REQUIRED = ['/monitor', '/mpc_stats', '/odom', '/joint_states', '/goal_pose']
OPTIONAL_IF_PUBLISHED = ['/opti_pessi/obstacles']
# Robot for the clearance of analyze_tests.py: trunk of the URDF and footprint with the legs
BODY = {'trunk_length': 0.647, 'trunk_width': 0.150, 'footprint_length': 0.647, 'footprint_width': 0.32}
OUTCOMES = {'s': 'success', 'f': 'failed', 'x': 'discard'}


class Watcher(Node):
    """Follows the controller while the bag records (phase of /monitor, goals, pose), sends the goal and drives
    obstacle_simulator. Every publisher and client is created here, before the bag starts: nothing new joins DDS
    once the robot walks (new participants stalled the MPC thread)."""

    def __init__(self, sim):
        super().__init__('test_watcher', parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, sim)])
        self.lock = threading.Lock()
        self.phase = None
        self.monitor_wall = None
        self.pose = None
        self.goals = []          # (t, x, y)
        self.events = []         # (t, phase) at every change of phase
        self.create_subscription(JointState, '/monitor', self.on_monitor, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.on_goal, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.reset_cli = self.create_client(Trigger, f'{OBSTACLE_NODE}/reset')
        self.moving_cli = self.create_client(SetBool, f'{OBSTACLE_NODE}/set_moving')
        self.param_cli = self.create_client(SetParameters, f'{OBSTACLE_NODE}/set_parameters')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_monitor(self, msg):
        v = dict(zip(msg.name, msg.position))
        with self.lock:
            phase = int(v.get('phase', -1))
            if phase != self.phase:
                self.events.append((self.now(), phase))
            self.phase = phase
            self.monitor_wall = time.monotonic()

    def on_goal(self, msg):
        with self.lock:
            self.goals.append((self.now(), msg.pose.position.x, msg.pose.position.y))

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with self.lock:
            self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    def missing_subscriptions(self):
        topics = REQUIRED + [t for t in OPTIONAL_IF_PUBLISHED if self.count_publishers(t) > 0]
        return [t for t in topics
                if RECORDER_NODE not in [i.node_name for i in self.get_subscriptions_info_by_topic(t)]]

    def controller_listens_goal(self):
        return CONTROLLER_NODE in [i.node_name for i in self.get_subscriptions_info_by_topic('/goal_pose')]

    def obstacles_running(self):
        return self.reset_cli.service_is_ready() and self.moving_cli.service_is_ready()

    def snapshot(self):
        with self.lock:
            alive = self.monitor_wall is not None and time.monotonic() - self.monitor_wall < 1.5
            return dict(phase=self.phase, alive=alive, pose=self.pose, goals=list(self.goals),
                        events=list(self.events))

    # obstacle_simulator ------------------------------------------------------------------------------------------
    @staticmethod
    def _wait(future, timeout=3.0):
        end = time.monotonic() + timeout
        while not future.done() and time.monotonic() < end:
            time.sleep(0.01)
        return future.result() if future.done() else None

    def reset_obstacle(self, start, speed):
        """Start and speed of the first obstacle, then back to the start, paused. Returns the answer or None."""
        params = [Parameter('obstacle_start', Parameter.Type.DOUBLE_ARRAY, [float(start[0]), float(start[1])]),
                  Parameter('obstacle_speed', Parameter.Type.DOUBLE, float(speed))]
        res = self._wait(self.param_cli.call_async(SetParameters.Request(
            parameters=[p.to_parameter_msg() for p in params])))
        if res is None or not all(r.successful for r in res.results):
            return None
        res = self._wait(self.reset_cli.call_async(Trigger.Request()))
        return res.message if res is not None and res.success else None

    def set_moving(self, moving):
        return self.moving_cli.call_async(SetBool.Request(data=moving))   # not awaited: goal at the same instant

    def send_goal(self, x, y, yaw):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        if yaw is not None:
            msg.pose.orientation.z = math.sin(yaw / 2.0)
            msg.pose.orientation.w = math.cos(yaw / 2.0)
        # else zero quaternion: the controller takes it as "no yaw"
        self.goal_pub.publish(msg)


def log(text):
    print(f'[{datetime.now().strftime("%H:%M:%S")}] {text}', flush=True)


def enter_pressed(timeout):
    """True if Enter was pressed. Reads the file descriptor directly: select + sys.stdin.readline() missed keys,
    because a line already in the buffer of Python is not seen by select."""
    fd = sys.stdin.fileno()
    ready, _, _ = select.select([fd], [], [], timeout)
    if ready:
        data = os.read(fd, 1024)
        return b'\n' in data or data == b''
    return False


def flush_keys():
    """Forgets the keys pressed before (an Enter pressed too early does not start the test)."""
    try:
        import termios
        if sys.stdin.isatty():
            termios.tcflush(sys.stdin.fileno(), termios.TCIFLUSH)
    except Exception:
        pass


def first_obstacle(config):
    instances = config.get('obstacles', {}).get('instances') or []
    return instances[0] if instances else None


def next_rep(campaign_dir, speed):
    prefix = f'v{speed:.2f}_r'
    reps = [int(d[len(prefix):]) for d in os.listdir(campaign_dir)
            if d.startswith(prefix) and d[len(prefix):].isdigit()]
    return max(reps, default=0) + 1


def git_commit():
    src = os.path.dirname(os.path.realpath(__file__))
    try:
        commit = subprocess.run(['git', 'rev-parse', '--short', 'HEAD'], cwd=src, capture_output=True,
                                text=True, timeout=5).stdout.strip()
        dirty = subprocess.run(['git', 'status', '--porcelain'], cwd=src, capture_output=True,
                               text=True, timeout=5).stdout.strip()
        return commit + ('-dirty' if dirty else '')
    except Exception:
        return 'unknown'


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--campaign', default='manual', help='folder of the tests in ~/colcon_ws/tests')
    parser.add_argument('--goal', type=float, nargs='+', metavar='V',
                        help='goal in odom: X Y [YAW_DEG]; without it the goal comes from RViz')
    parser.add_argument('--speed', type=float, help='obstacle speed, also set in obstacle_simulator '
                                                    '(default: speed of the first obstacle of config.yaml)')
    parser.add_argument('--obstacle-start', type=float, nargs=2, metavar=('X', 'Y'),
                        help='start of the obstacle (default: pos of the first obstacle of config.yaml)')
    parser.add_argument('--no-obstacle', action='store_true', help='do not touch obstacle_simulator')
    parser.add_argument('--note', default='', help='free text saved in meta.yaml')
    parser.add_argument('--sim', action='store_true', help='simulation: sim time for the bag and the timestamps')
    parser.add_argument('--hw-log', help='log of the hardware terminal (tee): "Power Protection" in it is detected')
    parser.add_argument('--after', type=float, default=2.0, help='s recorded after the end of the test (default 2)')
    args = parser.parse_args()
    if args.goal is not None and len(args.goal) not in (2, 3):
        parser.error('--goal wants X Y or X Y YAW_DEG')
    goal_cmd = None
    if args.goal is not None:
        goal_cmd = (args.goal[0], args.goal[1], math.radians(args.goal[2]) if len(args.goal) == 3 else None)

    config_path = os.path.join(get_package_share_directory(PKG), 'config', 'config.yaml')
    with open(config_path) as f:
        config = yaml.safe_load(f)
    obs_cfg = first_obstacle(config)
    speed = args.speed if args.speed is not None else float((obs_cfg or {}).get('speed', 0.0))
    obstacle_start = list(args.obstacle_start) if args.obstacle_start else \
        ([float(v) for v in obs_cfg['pos']] if obs_cfg else None)

    campaign_dir = os.path.join(os.path.expanduser('~/colcon_ws/tests'), args.campaign)
    os.makedirs(campaign_dir, exist_ok=True)
    rep = next_rep(campaign_dir, speed)
    test_id = f'v{speed:.2f}_r{rep:02d}'
    test_dir = os.path.join(campaign_dir, test_id)
    os.makedirs(test_dir)
    shutil.copy(config_path, os.path.join(test_dir, 'config.yaml'))
    pp_before = open(args.hw_log, errors='ignore').read().count('Power Protection') \
        if args.hw_log and os.path.exists(args.hw_log) else 0

    rclpy.init()
    node = Watcher(args.sim)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()

    cmd = ['ros2', 'bag', 'record', '-s', 'mcap', '-o', os.path.join(test_dir, 'bag'), '--node-name', RECORDER_NODE]
    if args.sim:
        cmd.append('--use-sim-time')
    bag_log = open(os.path.join(test_dir, 'bag_record.log'), 'w')
    bag = subprocess.Popen(cmd + ['--topics'] + TOPICS, stdout=bag_log, stderr=subprocess.STDOUT,
                           start_new_session=True)
    outcome_detected, t_goal, t_end, goal = None, None, None, None
    obstacle_used = False
    try:
        log(f'{test_id} (ostacolo {speed:.2f} m/s) in {test_dir}')
        log('Avvio la registrazione ...')
        deadline = time.monotonic() + 20.0
        while time.monotonic() < deadline:
            if bag.poll() is not None:
                raise RuntimeError(f'ros2 bag record terminato, vedi {test_dir}/bag_record.log')
            if not node.missing_subscriptions() and node.snapshot()['alive'] and node.controller_listens_goal():
                break
            time.sleep(0.1)
        else:
            snap = node.snapshot()
            raise RuntimeError('registrazione non pronta: ' +
                               ('controller non attivo (/monitor fermo)' if not snap['alive'] else
                                'il controller non è iscritto a /goal_pose' if not node.controller_listens_goal() else
                                f'topic non sottoscritti {node.missing_subscriptions()}'))

        # Obstacle at its start, paused
        if args.no_obstacle:
            log('Ostacolo: non lo controllo (--no-obstacle)')
        elif not node.obstacles_running():
            log('Nota: nessun obstacle_simulator attivo (test senza ostacolo). Se ti serve, avvialo PRIMA di questo comando.')
        elif obstacle_start is None:
            log('Nota: nessun ostacolo in config.yaml')
        else:
            answer = node.reset_obstacle(obstacle_start, speed)
            if answer is None:
                raise RuntimeError('obstacle_simulator non risponde al reset (è la versione nuova? riavvialo)')
            obstacle_used = True
            log(f'Ostacolo in partenza: {answer}')
        time.sleep(2.0)   # discovery settled and the obstacle at its start seen by the controller before the goal
        if node.snapshot()['phase'] != STANCE:
            log('ATTENZIONE: il controller non è in STANCE')
        n_goals = len(node.snapshot()['goals'])

        if goal_cmd is not None:
            yaw_txt = f', yaw {math.degrees(goal_cmd[2]):.0f} deg' if goal_cmd[2] is not None else ''
            log(f'PRONTO: Invio = parte il test (goal [{goal_cmd[0]:.2f}, {goal_cmd[1]:.2f}]{yaw_txt}'
                f'{" + ostacolo" if obstacle_used else ""}), Ctrl+C = annulla')
            # The goal is sent again every 0.3 s until the controller walks: a single message can be lost if the
            # controller has not matched this publisher yet. The obstacle starts when the robot starts (WALK), so
            # a repeated goal does not shift it: same encounter at every repetition
            while t_goal is None:
                flush_keys()
                while not enter_pressed(0.1):
                    pass
                log('Invio ricevuto: mando il goal ...')
                deadline = time.monotonic() + 3.0
                while time.monotonic() < deadline:
                    node.send_goal(*goal_cmd)
                    t_sent = node.now()
                    end = time.monotonic() + 0.3
                    while time.monotonic() < end and node.snapshot()['phase'] != WALK:
                        time.sleep(0.01)
                    if node.snapshot()['phase'] == WALK:
                        if obstacle_used:
                            node.set_moving(True)
                        t_goal, goal = t_sent, [goal_cmd[0], goal_cmd[1]]
                        break
                if t_goal is None:
                    log(f'Il controller non parte (fase {node.snapshot()["phase"]}, deve essere 1 = STANCE). '
                        'Invio = riprova, Ctrl+C = annulla')
            n_goals = len(node.snapshot()['goals'])   # own goals are not "new goals" for the loop below
            log('Partito' + (' con l\'ostacolo' if obstacle_used else '') + '. Invio = ferma la registrazione.')
        else:
            flush_keys()
            log('PRONTO: manda il goal da RViz' + (' (l\'ostacolo parte con il goal)' if obstacle_used else '') +
                '. Invio = ferma la registrazione.')

        saw_walk = False
        while True:
            snap = node.snapshot()
            new_goals = snap['goals'][n_goals:]
            if new_goals and t_goal is None:
                t_goal, goal = new_goals[0][0], list(new_goals[0][1:])
                if obstacle_used:
                    node.set_moving(True)
                log(f'Goal ricevuto: [{goal[0]:.2f}, {goal[1]:.2f}]')
            if t_goal is not None and snap['phase'] == WALK:
                saw_walk = True
            if saw_walk and snap['phase'] == STANCE:
                outcome_detected, t_end = 'success', node.now()
                log('Goal raggiunto (STANCE)')
                break
            if t_goal is not None and not snap['alive']:
                outcome_detected, t_end = 'controller_dead', node.now()
                log('Il controller non risponde (Power Protection o errore)')
                break
            if enter_pressed(0.05):
                t_end = node.now()
                outcome_detected = 'stopped' if t_goal is not None else 'no_goal'
                log('Fermata a mano')
                break
        if outcome_detected != 'no_goal':
            # A bit more of recording: the robot standing after the goal, the end of a fall
            end_wall = time.monotonic() + args.after
            while time.monotonic() < end_wall and not enter_pressed(0.05):
                pass
    except KeyboardInterrupt:
        outcome_detected = outcome_detected or ('interrupted' if t_goal is not None else 'no_goal')
        t_end = t_end or node.now()
    except RuntimeError as e:
        log(f'ERRORE: {e}')
        outcome_detected = 'error'
    finally:
        if obstacle_used:
            node.set_moving(False)
            time.sleep(0.2)
        if bag.poll() is None:
            os.killpg(bag.pid, signal.SIGINT)
            try:
                bag.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(bag.pid, signal.SIGKILL)
        bag_log.close()

    snap = node.snapshot()
    if outcome_detected == 'controller_dead' and args.hw_log and os.path.exists(args.hw_log):
        time.sleep(1.0)
        if open(args.hw_log, errors='ignore').read().count('Power Protection') > pp_before:
            outcome_detected = 'power_protection'
    default = {'success': 'success', 'no_goal': 'discard', 'error': 'discard'}.get(outcome_detected, 'failed')

    print()
    print(f'Esito rilevato: {outcome_detected}')
    print('  s = success   f = failed   x = scarta il test')
    try:
        answer = input(f'Esito [Invio = {default}]: ').strip().lower()
    except (EOFError, KeyboardInterrupt):
        answer = ''
    outcome = OUTCOMES.get(answer[:1], default) if answer else default

    if outcome == 'discard':
        shutil.rmtree(test_dir, ignore_errors=True)
        log(f'Test scartato, cartella {test_id} cancellata.')
    else:
        meta = {
            'trial': test_id, 'campaign': args.campaign, 'speed': float(speed), 'rep': rep,
            'outcome': outcome, 'outcome_detected': outcome_detected,
            'note': args.note, 'mode': 'sim' if args.sim else 'real', 'commit': git_commit(),
            'date': datetime.now().isoformat(timespec='seconds'),
            't_goal': t_goal, 't_end': t_end, 'goal': list(goal) if goal else None,
            'goal_yaw_deg': args.goal[2] if args.goal is not None and len(args.goal) == 3 else None,
            'goal_source': 'command' if goal_cmd is not None else 'rviz',
            'obstacle_controlled': obstacle_used,
            'obstacle_start': obstacle_start if obstacle_used else None,
            'obstacle_end': [float(v) for v in obs_cfg['pos_end']]
                            if obstacle_used and obs_cfg.get('pos_end') else None,
            'obstacle_type': obs_cfg.get('type') if obstacle_used else None,
            'obstacle_center': [float(v) for v in obs_cfg['center']]
                               if obstacle_used and obs_cfg.get('center') else None,
            'final_pose': list(snap['pose']) if snap['pose'] else None,
            'phase_events': [[round(t, 3), p] for t, p in snap['events']],
            'body': BODY, 'config': 'config.yaml',
        }
        with open(os.path.join(test_dir, 'meta.yaml'), 'w') as f:
            yaml.safe_dump(meta, f, sort_keys=False)
        log(f'Salvato {test_id}: {outcome}. Analisi: ros2 run {PKG} analyze_tests.py {campaign_dir}')

    executor.shutdown(timeout_sec=2.0)
    spinner.join(timeout=2.0)
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
