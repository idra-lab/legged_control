"""When far_inspection_goals.py sends the picked points to far_planner: on the start command of map_point_picker."""
import importlib.util
import os

import pytest
import rclpy
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, String

os.environ["ROS_DOMAIN_ID"] = "93"  # the node publishes goals: never on the domain of a robot

_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'scripts', 'far_inspection_goals.py')
_spec = importlib.util.spec_from_file_location('far_inspection_goals', _path)
script = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(script)


def points_file(path, *points):
    path.write_text("map: /maps/optimized_map.pcd\npoints:\n"
                    + "".join("  - {x: %.4f, y: %.4f, z: 0.0500}\n" % p for p in points))
    return str(path)


class Inspection:
    """The node with far_planner ready (graph loaded, robot localized), a clock the test moves, goals caught."""

    def __init__(self, goals_file, auto_start):
        rclpy.init(args=['--ros-args', '-p', f'goals_file:={goals_file}', '-p', 'wait_for_graph:=false',
                         '-p', f'auto_start:={str(auto_start).lower()}', '-p', 'dwell_time:=2.0'])
        self.node = script.FarInspectionGoals()
        self.clock = 50.0
        self.node.now = lambda: self.clock
        self.sent = []
        self.node.goal_pub.publish = lambda goal: self.sent.append((goal.point.x, goal.point.y))
        self.node.graph_size = 40
        self.node.odom = Point(x=0.0, y=0.0, z=0.47)

    def start_command(self, path=''):
        self.node.on_start(String(data=path))

    def tick(self, seconds=0.2):
        self.clock += seconds
        self.node.tick()

    def reach_goal(self):
        self.node.on_reach(Bool(data=True))
        self.tick()          # reached -> dwell
        self.tick(2.1)       # dwell over -> next point, or the end


@pytest.fixture
def inspection():
    made = []

    def make(goals_file, auto_start=False):
        made.append(Inspection(goals_file, auto_start))
        return made[0]

    yield make
    for i in made:
        i.node.destroy_node()
    rclpy.try_shutdown()


def test_no_goal_is_sent_before_the_start_command(inspection, tmp_path):
    i = inspection(points_file(tmp_path / 'selected_points.yaml', (4.0, 1.0), (6.0, -2.0)))
    for _ in range(20):
        i.tick()

    assert i.sent == []
    assert i.node.state == 'idle'


def test_the_start_command_sends_the_first_point_of_the_file_it_names(inspection, tmp_path):
    i = inspection(points_file(tmp_path / 'selected_points.yaml', (4.0, 1.0)))
    other = points_file(tmp_path / 'round_two.yaml', (-3.0, 5.0), (6.0, -2.0))
    i.start_command(other)
    i.tick()

    assert i.sent == [(-3.0, 5.0)]
    assert i.node.state == 'active'


def test_a_command_without_a_file_takes_goals_file(inspection, tmp_path):
    i = inspection(points_file(tmp_path / 'selected_points.yaml', (4.0, 1.0), (6.0, -2.0)))
    i.start_command()
    i.tick()

    assert i.sent == [(4.0, 1.0)]


def test_the_points_are_the_ones_in_the_file_when_the_command_comes(inspection, tmp_path):
    path = tmp_path / 'selected_points.yaml'
    i = inspection(points_file(path, (4.0, 1.0)))
    points_file(path, (7.5, 2.5))  # picked and saved after the launch
    i.start_command()
    i.tick()

    assert i.sent == [(7.5, 2.5)]


def test_it_goes_through_the_points_then_waits_for_the_next_command(inspection, tmp_path):
    path = tmp_path / 'selected_points.yaml'
    i = inspection(points_file(path, (4.0, 1.0), (6.0, -2.0)))
    i.start_command()
    i.tick()
    i.reach_goal()
    assert i.sent == [(4.0, 1.0), (6.0, -2.0)]
    i.reach_goal()

    assert i.node.state == 'idle'  # still running: the next round needs no relaunch
    for _ in range(20):
        i.tick()
    assert i.sent == [(4.0, 1.0), (6.0, -2.0)]

    points_file(path, (1.0, 1.0))
    i.start_command()
    i.tick()
    assert i.sent[-1] == (1.0, 1.0)


def test_a_command_during_an_inspection_starts_over_with_the_new_points(inspection, tmp_path):
    path = tmp_path / 'selected_points.yaml'
    i = inspection(points_file(path, (4.0, 1.0), (6.0, -2.0)))
    i.start_command()
    i.tick()
    points_file(path, (-2.0, -2.0), (0.0, 9.0))
    i.start_command()
    i.tick()

    assert i.sent == [(4.0, 1.0), (-2.0, -2.0)]
    assert i.node.index == 0 and len(i.node.goals) == 2


def test_a_command_for_a_missing_or_empty_file_leaves_things_as_they_are(inspection, tmp_path):
    i = inspection(points_file(tmp_path / 'selected_points.yaml', (4.0, 1.0)))
    i.start_command(str(tmp_path / 'nowhere.yaml'))
    i.tick()
    assert i.sent == [] and i.node.state == 'idle'

    i.start_command(points_file(tmp_path / 'empty.yaml'))
    i.tick()
    assert i.sent == [] and i.node.state == 'idle'


def test_auto_start_inspects_goals_file_at_once_and_ends_after_the_last_point(inspection, tmp_path):
    i = inspection(points_file(tmp_path / 'selected_points.yaml', (4.0, 1.0)), auto_start=True)
    i.tick()
    assert i.sent == [(4.0, 1.0)]
    i.reach_goal()

    assert i.node.state == 'done'
