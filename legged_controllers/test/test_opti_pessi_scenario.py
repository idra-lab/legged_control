"""Where opti_pessi_scenario.py sends its obstacle: to the controller, or with --person to the person of Gazebo."""
import importlib.util
import math
import os

import pytest
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.clock import ClockType
from rclpy.time import Time

os.environ["ROS_DOMAIN_ID"] = "93"  # the node publishes goals and obstacles: never on the domain of a robot

_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'scripts', 'opti_pessi_scenario.py')
_spec = importlib.util.spec_from_file_location('opti_pessi_scenario', _path)
scenario = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(scenario)

PATROL = 2      # the obstacle walks along its direction
ANTAGONIST = 4  # the obstacle heads for the robot


def layout(number):
    """Goal, obstacle start, speed and direction of a scenario, as the node lays it out: the table is tuned by hand."""
    cfg = scenario.SCENARIOS[number]
    return (tuple(v * scenario.SCALE for v in cfg["goal"]), tuple(v * scenario.SCALE for v in cfg["obstacle"]),
            cfg["speed"], cfg["direction"])


GOAL, START, SPEED, DIRECTION = layout(PATROL)


@pytest.fixture
def node_of():
    rclpy.init()
    nodes = []

    def make(*args, **kwargs):
        nodes.append(scenario.ScenarioPublisher(*args, **kwargs))
        return nodes[-1]

    yield make
    for node in nodes:
        node.destroy_node()
    rclpy.shutdown()


def sent_by_tick(node):
    """One tick of the node: what it published, by topic."""
    sent = {}
    for publisher in node.publishers:
        publisher.publish = lambda msg, topic=publisher.topic_name: sent.__setitem__(topic, msg)
    node.tick()
    return sent


def xy(point):
    return pytest.approx((point.x, point.y))


def tell_tf(node, parent, child, x, y, yaw=0.0):
    """As if TF had `child` at (x, y) of `parent`, turned by yaw."""
    tf = TransformStamped()
    tf.header.frame_id, tf.child_frame_id = parent, child
    tf.header.stamp.sec = 1
    tf.transform.translation.x, tf.transform.translation.y = float(x), float(y)
    tf.transform.rotation.z, tf.transform.rotation.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
    node.tf_buffer.set_transform(tf, "test")


def at(node, seconds):
    node.get_clock().now = lambda: Time(seconds=seconds, clock_type=ClockType.ROS_TIME)


def test_obstacle_goes_to_the_controller_by_default(node_of):
    sent = sent_by_tick(node_of(PATROL, 20.0))
    obstacles = sent["/opti_pessi/obstacles"]
    assert obstacles.header.frame_id == "odom"
    assert [obstacle.type for obstacle in obstacles.obstacles] == [scenario.Obstacle.HUMAN]
    assert START == xy(obstacles.obstacles[0].position)
    assert "/scenario_person/pose" not in sent


def test_person_gets_the_obstacle_position_and_the_controller_nothing(node_of):
    node = node_of(PATROL, 20.0, person=True)
    sent = sent_by_tick(node)
    assert START == xy(sent["/scenario_person/pose"].position)
    assert "/opti_pessi/obstacles" not in sent
    # Not even a silent publisher: the people pipeline must be the only one the controller can hear there
    assert "/opti_pessi/obstacles" not in [publisher.topic_name for publisher in node.publishers]


@pytest.mark.parametrize("person", [False, True])
def test_goal_and_true_obstacle_marker_are_published_in_both_modes(node_of, person):
    sent = sent_by_tick(node_of(PATROL, 20.0, person=person))
    assert GOAL == xy(sent["/opti_pessi/goal"].markers[0].pose.position)
    assert START == xy(sent["/opti_pessi/scenario_obstacles"].markers[0].pose.position)


def test_person_walks_in_simulation_time(node_of):
    assert node_of(PATROL, 20.0, person=True).get_parameter("use_sim_time").value is True
    assert node_of(PATROL, 20.0).get_parameter("use_sim_time").value is False


def test_time_gazebo_ran_before_the_first_tick_does_not_move_the_person(node_of):
    # On simulation time the clock reads 0 until the first /clock message, then the time Gazebo has already run
    node = node_of(PATROL, 20.0, person=True)
    at(node, 35.0)
    assert START == xy(sent_by_tick(node)["/scenario_person/pose"].position)
    at(node, 36.0)
    after_one_second = (START[0] + DIRECTION[0] * SPEED, START[1] + DIRECTION[1] * SPEED)
    assert after_one_second == xy(sent_by_tick(node)["/scenario_person/pose"].position)


def test_with_the_person_the_scenario_is_in_the_world_and_the_goal_goes_to_the_controller_in_odom(node_of):
    # The estimator has drifted: the world (FAST-LIO's lio_map) is 1 m ahead of odom and turned by 90 deg
    node = node_of(PATROL, 20.0, person=True)
    tell_tf(node, "odom", "lio_map", 1.0, 0.0, math.pi / 2.0)
    sent = sent_by_tick(node)
    assert START == xy(sent["/scenario_person/pose"].position)   # Gazebo is the world: as the scenario says
    goal, disk = sent["/opti_pessi/goal"].markers[0], sent["/opti_pessi/scenario_obstacles"].markers[0]
    assert goal.header.frame_id == disk.header.frame_id == "odom"
    assert (1.0 - GOAL[1], GOAL[0]) == xy(goal.pose.position)     # turned by 90 deg, then 1 m along x
    assert (1.0 - START[1], START[0]) == xy(disk.pose.position)


def test_without_the_person_odom_is_the_frame_of_the_scenario_whatever_tf_says(node_of):
    node = node_of(PATROL, 20.0)
    tell_tf(node, "odom", "lio_map", 1.0, 0.0, math.pi / 2.0)
    sent = sent_by_tick(node)
    assert GOAL == xy(sent["/opti_pessi/goal"].markers[0].pose.position)
    assert START == xy(sent["/opti_pessi/obstacles"].obstacles[0].position)


@pytest.mark.parametrize("person, towards", [(False, -1.0), (True, 1.0)])
def test_antagonist_heads_for_the_robot_where_the_scenario_is(node_of, person, towards):
    # The estimator has the robot 5 m to one side of the obstacle, FAST-LIO 5 m to the other: the person in Gazebo
    # must go for the second
    _, start, speed, _ = layout(ANTAGONIST)
    node = node_of(ANTAGONIST, 20.0, person=person)
    tell_tf(node, "odom", "base", start[0], start[1] - 5.0)
    tell_tf(node, "lio_map", "lio_base", start[0], start[1] + 5.0)
    at(node, 35.0)
    sent_by_tick(node)
    at(node, 36.0)
    sent = sent_by_tick(node)
    position = sent["/scenario_person/pose"].position if person else sent["/opti_pessi/obstacles"].obstacles[0].position
    assert (start[0], start[1] + towards * speed) == xy(position)


def test_antagonist_person_uses_the_estimator_until_fast_lio_runs(node_of):
    _, start, speed, _ = layout(ANTAGONIST)
    node = node_of(ANTAGONIST, 20.0, person=True)
    tell_tf(node, "odom", "base", start[0], start[1] - 5.0)
    at(node, 35.0)
    sent_by_tick(node)
    at(node, 36.0)
    assert (start[0], start[1] - speed) == xy(sent_by_tick(node)["/scenario_person/pose"].position)


@pytest.mark.parametrize("argv, expected", [
    (["scenario", "2"], (2, 20.0, False)),
    (["scenario", "3", "50"], (3, 50.0, False)),
    (["scenario", "2", "--person"], (2, 20.0, True)),
    (["scenario", "--person", "4", "10"], (4, 10.0, True)),
    (["scenario"], None),
    (["scenario", "5"], None),
    (["scenario", "--person"], None),
])
def test_command_line(argv, expected):
    assert scenario.parse_arguments(argv) == expected
