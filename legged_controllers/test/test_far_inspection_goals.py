"""When far_inspection_goals.py sends the picked points to far_planner: on the start command of map_point_picker."""
import importlib.util
import os

import pytest
import rclpy
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, String
from visibility_graph_msg.msg import Graph
from visibility_graph_msg.msg import Node as GraphNode

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

    def __init__(self, goals_file, auto_start, graph_file=''):
        graph = ['-p', f'graph_file:={graph_file}', '-p', 'wait_for_graph:=true'] if graph_file else \
            ['-p', 'wait_for_graph:=false']
        rclpy.init(args=['--ros-args', '-p', f'goals_file:={goals_file}', *graph,
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

    def make(goals_file, auto_start=False, graph_file=''):
        made.append(Inspection(goals_file, auto_start, graph_file))
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


def saved_graph(path, nodes):
    path.write_text("".join(f"{i} 1 0.0 0.0 0.0 0 0 0 0 0 0 1 0 0 0 | | | \n" for i in range(nodes)))
    return str(path)


def test_with_a_saved_graph_it_waits_until_far_planner_has_it(inspection, tmp_path):
    i = inspection(points_file(tmp_path / 'selected_points.yaml', (4.0, 1.0)),
                   graph_file=saved_graph(tmp_path / 'warehouse.vgh', 120))
    i.node.graph_size = 3  # what far_planner sees around the robot
    i.start_command()
    i.tick()
    assert i.sent == []

    i.node.on_decoded(Graph(nodes=[GraphNode()] * 120))    # graph_decoder read the file
    i.node.on_vgraph(Graph(nodes=[GraphNode()] * 121))     # and far_planner merged it
    i.tick()
    assert i.sent == [(4.0, 1.0)]


def test_a_saved_graph_loaded_before_the_node_could_hear_of_it_still_counts(inspection, tmp_path):
    # far_planner up at once: graph_decoder publishes the loaded graph once, before this node is discovered
    i = inspection(points_file(tmp_path / 'selected_points.yaml', (4.0, 1.0)),
                   graph_file=saved_graph(tmp_path / 'warehouse.vgh', 120))
    i.node.graph_size = 121  # far_planner publishes its graph on every loop: the saved nodes are in it
    i.start_command()
    i.tick()

    assert i.sent == [(4.0, 1.0)]
