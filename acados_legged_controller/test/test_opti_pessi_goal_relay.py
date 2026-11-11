"""Which /opti_pessi/goal markers opti_pessi_goal_relay.py forwards to /goal_pose: acados walks again on every goal it
gets while standing, so the same goal must not come again, but a new position or a new yaw must."""
import importlib.util
import math
import os

import pytest
import rclpy
from visualization_msgs.msg import Marker, MarkerArray

os.environ["ROS_DOMAIN_ID"] = "93"  # the node publishes goals: never on the domain of a robot

_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'scripts', 'opti_pessi_goal_relay.py')
_spec = importlib.util.spec_from_file_location('opti_pessi_goal_relay', _path)
relay = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(relay)


@pytest.fixture
def node():
    rclpy.init()
    n = relay.OptiPessiGoalRelay()
    n.sent = []
    n.publisher.publish = n.sent.append
    yield n
    n.destroy_node()
    rclpy.try_shutdown()


def goal(n, x, y, yaw, frame='odom'):
    marker = Marker(action=Marker.ADD, type=Marker.ARROW)
    marker.header.frame_id = frame
    marker.pose.position.x, marker.pose.position.y = float(x), float(y)
    marker.pose.orientation.z, marker.pose.orientation.w = math.sin(0.5 * yaw), math.cos(0.5 * yaw)
    n.callback(MarkerArray(markers=[marker]))


def test_the_goal_is_forwarded_with_its_orientation(node):
    goal(node, 1.0, 2.0, 0.8)

    assert len(node.sent) == 1
    sent = node.sent[0]
    assert sent.header.frame_id == 'odom'
    assert (sent.pose.position.x, sent.pose.position.y) == (1.0, 2.0)
    assert (sent.pose.orientation.z, sent.pose.orientation.w) == pytest.approx((math.sin(0.4), math.cos(0.4)))


def test_the_same_goal_again_is_not_forwarded(node):
    goal(node, 1.0, 2.0, 0.8)
    goal(node, 1.01, 2.0, 0.82)  # 1 cm and 0.02 rad: noise of a standing robot

    assert len(node.sent) == 1


def test_a_new_yaw_at_the_same_place_is_forwarded(node):
    goal(node, 1.0, 2.0, 0.8)
    goal(node, 1.0, 2.0, 0.9)

    assert len(node.sent) == 2


def test_a_yaw_change_across_pi_is_measured_the_short_way(node):
    goal(node, 1.0, 2.0, math.pi - 0.01)
    goal(node, 1.0, 2.0, -math.pi + 0.01)  # 0.02 rad apart

    assert len(node.sent) == 1


def test_a_new_position_is_forwarded(node):
    goal(node, 1.0, 2.0, 0.8)
    goal(node, 1.05, 2.0, 0.8)

    assert len(node.sent) == 2


def test_a_goal_not_in_odom_is_dropped(node):
    goal(node, 1.0, 2.0, 0.8, frame='map')

    assert node.sent == []
