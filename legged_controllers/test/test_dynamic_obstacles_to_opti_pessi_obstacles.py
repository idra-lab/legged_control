"""What dynamic_obstacles_to_opti_pessi_obstacles.py hands to the controller for the people of people_perception."""
import importlib.util
import math
import os

import pytest
from geometry_msgs.msg import TransformStamped
from legged_controllers.msg import Obstacle
from people_perception_msgs.msg import DynamicObstacle, DynamicObstacleArray

_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'scripts',
                     'dynamic_obstacles_to_opti_pessi_obstacles.py')
_spec = importlib.util.spec_from_file_location('dynamic_obstacles_to_opti_pessi_obstacles', _path)
relay = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(relay)

PEDESTRIAN = 7  # autoware_perception_msgs/ObjectClassification
CAR = 1


def people(*positions, frame='camera_init', label=PEDESTRIAN):
    """The people of one scan as obstacle_adapter publishes them."""
    msg = DynamicObstacleArray()
    msg.header.frame_id = frame
    msg.header.stamp.sec = 10
    msg.header.stamp.nanosec = 699999999
    for x, y, z in positions:
        person = DynamicObstacle(label=label, radius=0.35, max_speed=1.5)
        person.position.x, person.position.y, person.position.z = float(x), float(y), float(z)
        msg.obstacles.append(person)
    return msg


def odom_from_camera_init(x, y, z, yaw):
    tf = TransformStamped()
    tf.header.frame_id = 'odom'
    tf.child_frame_id = 'camera_init'
    tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = x, y, z
    tf.transform.rotation.z = math.sin(yaw / 2)
    tf.transform.rotation.w = math.cos(yaw / 2)
    return tf


def test_people_come_out_as_humans_in_odom_at_the_stamp_of_their_scan():
    msg = people((3.0, 0.0, 0.4), (0.0, -2.0, 0.4))
    out, unknown = relay.to_opti_pessi_obstacles(msg, odom_from_camera_init(1.0, 2.0, 0.5, math.pi / 2), 'odom')

    assert out.header.frame_id == 'odom'  # the controller ignores any other frame
    assert out.header.stamp == msg.header.stamp
    assert unknown == []
    assert [obstacle.type for obstacle in out.obstacles] == [Obstacle.HUMAN, Obstacle.HUMAN]
    # camera_init is turned a quarter left in odom and sits at (1, 2, 0.5)
    first, second = (obstacle.position for obstacle in out.obstacles)
    assert (first.x, first.y, first.z) == pytest.approx((1.0, 5.0, 0.9))
    assert (second.x, second.y, second.z) == pytest.approx((3.0, 2.0, 0.9))


def test_nobody_gives_an_empty_array_which_clears_the_obstacles_of_the_controller():
    out, unknown = relay.to_opti_pessi_obstacles(people(), odom_from_camera_init(1.0, 2.0, 0.5, 0.3), 'odom')
    assert out.header.frame_id == 'odom'
    assert list(out.obstacles) == []
    assert unknown == []


def test_obstacles_already_in_the_frame_of_the_controller_are_not_moved():
    out, _ = relay.to_opti_pessi_obstacles(people((3.0, 1.0, 0.4), frame='odom'), None, 'odom')
    assert len(out.obstacles) == 1
    position = out.obstacles[0].position
    assert (position.x, position.y, position.z) == (3.0, 1.0, 0.4)


def test_a_label_the_controller_has_no_type_for_is_left_out_and_reported():
    msg = people((3.0, 0.0, 0.4))
    msg.obstacles.append(DynamicObstacle(label=CAR))
    out, unknown = relay.to_opti_pessi_obstacles(msg, odom_from_camera_init(0.0, 0.0, 0.0, 0.0), 'odom')
    assert [obstacle.type for obstacle in out.obstacles] == [Obstacle.HUMAN]
    assert unknown == [CAR]
