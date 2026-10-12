"""Which /opti_pessi/goal tare_path_to_opti_pessi_goal.py picks when far_planner gives it a route (/viz_path_topic)."""
import importlib.util
import os

import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import Point, PointStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker

os.environ["ROS_DOMAIN_ID"] = "93"  # the node publishes goals: never on the domain of a robot

_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'scripts', 'tare_path_to_opti_pessi_goal.py')
_spec = importlib.util.spec_from_file_location('tare_path_to_opti_pessi_goal', _path)
bridge = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(bridge)

LOOKAHEAD = 2.0
CHORD = 0.2


def route(*xy):
    return np.array([(x, y, 0.0) for x, y in xy])


def pursue(path, robot, lookahead=LOOKAHEAD):
    return tuple(np.round(bridge.pursue(path, np.array(robot, dtype=float), lookahead, CHORD)[:2], 3))


# ---------------------------------------------------------------- the point of the route to walk to

def test_on_a_straight_route_the_goal_is_lookahead_ahead_on_it():
    assert pursue(route((0, 0), (10, 0)), (1.0, 0.0)) == (3.0, 0.0)


def test_the_goal_is_measured_from_where_the_robot_is_along_the_route_not_from_its_start():
    # far_planner's route starts where the robot was when it planned; the robot has walked on, a little to the side
    assert pursue(route((0, 0), (10, 0)), (4.0, 0.3)) == (6.0, 0.0)


def test_the_goal_never_goes_past_the_end_of_the_route():
    assert pursue(route((0, 0), (1.2, 0)), (0.0, 0.0)) == (1.2, 0.0)
    assert pursue(route((0, 0), (1.2, 0)), (1.2, 0.0)) == (1.2, 0.0)


def test_before_a_corner_the_goal_stops_at_the_corner_instead_of_cutting_it():
    # the route turns left round an obstacle corner at (3, 0): the straight line to a point past it would cut inside
    path = route((0, 0), (3, 0), (3, 5))
    assert pursue(path, (2.0, 0.0)) == (3.0, 0.0)


def test_at_the_corner_the_goal_moves_on_along_the_next_stretch():
    path = route((0, 0), (3, 0), (3, 5))
    assert pursue(path, (3.0, 0.0)) == (3.0, 2.0)


def test_a_gentle_bend_is_cut_as_far_as_max_chord_deviation_allows():
    path = route((0, 0), (1, 0), (3, 0.2))  # the vertex (1, 0) lies 0.05 m off the line robot -> (2, 0.1)
    assert pursue(path, (0.0, 0.0)) == pytest.approx((1.995, 0.1), abs=0.01)


def test_a_route_that_comes_back_past_the_robot_is_followed_from_its_start():
    # out and back round the end of a wall: the return stretch passes 0.1 m from the robot too
    path = route((0, 0), (4, 0), (4, 0.1), (0, 0.1))
    assert pursue(path, (0.0, 0.0)) == (2.0, 0.0)


# ---------------------------------------------------------------- the node

class Bridge:
    """The node with everything already in odom (no TF needed) and its goals caught."""

    def __init__(self, far_path=True):
        args = ['--ros-args', '-p', f'lookahead_distance:={LOOKAHEAD}']
        if far_path:
            args += ['-p', 'route_topic:=/viz_path_topic']
        rclpy.init(args=args)
        self.node = bridge.TareGoalBridge()
        self.goals = []
        self.node.goal_pub.publish = lambda msg: self.goals.append(msg.markers[0])

    def put(self, key, msg):
        self.node.inputs[key] = (msg, self.node.get_clock().now())

    def robot(self, x, y):
        odom = Odometry()
        odom.header.frame_id = 'odom'
        odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = float(x), float(y), 0.4
        self.put('/odom', odom)

    def way_point(self, x, y):
        msg = PointStamped()
        msg.header.frame_id = 'odom'
        msg.point.x, msg.point.y = float(x), float(y)
        self.put('/way_point', msg)

    def tare_path(self, *xy):
        path = Path()
        path.header.frame_id = 'odom'
        for x, y in xy:
            pose = PoseStamped()
            pose.pose.position.x, pose.pose.position.y = float(x), float(y)
            path.poses.append(pose)
        self.put('/exploration_path', path)

    def far_route(self, *xy):
        marker = Marker(type=Marker.LINE_STRIP, points=[Point(x=float(x), y=float(y)) for x, y in xy])
        marker.header.frame_id = 'odom'
        self.put('/viz_path_topic', marker)

    def obstacles(self, *xy):
        self.node.inputs['/terrain_map'] = (('odom', np.array([(x, y, 0.5) for x, y in xy], dtype=float).reshape(-1, 3)),
                                           self.node.get_clock().now())

    def goal(self):
        """Goal (x, y) and where it comes from, after one tick."""
        self.node.published_goal = None  # publish whatever the tick decides
        self.node.tick()
        marker = self.goals[-1]
        source = {tuple(c): s for s, c in bridge.COLORS.items()}[(marker.color.r, marker.color.g, marker.color.b)]
        return (round(marker.pose.position.x, 2), round(marker.pose.position.y, 2)), source


@pytest.fixture
def far():
    made = []

    def make(**kwargs):
        made.append(Bridge(**kwargs))
        return made[0]

    yield make
    for b in made:
        b.node.destroy_node()
    rclpy.try_shutdown()


def test_the_goal_is_on_far_planners_route_not_towards_its_way_point(far):
    b = far()
    b.robot(0.0, 0.0)
    b.far_route((0, 0), (10, 0))
    b.way_point(2.0, 1.5)  # far_planner's carrot: 2.5 m from the robot, pushed off the route
    b.obstacles()

    assert b.goal() == ((2.0, 0.0), 'path')


def test_an_obstacle_on_the_way_to_the_goal_shortens_the_step_along_the_route(far):
    b = far()
    b.robot(0.0, 0.0)
    b.far_route((0, 0), (3, 0), (3, 5))
    b.obstacles((1.6, 0.1))  # somebody standing on the route

    position, source = b.goal()
    assert source == 'path'
    assert position == (1.0, 0.0)  # the lookahead halved: still on the route, short of the obstacle


def test_a_route_blocked_right_ahead_is_left_by_a_detour(far):
    b = far()
    b.robot(0.0, 0.0)
    b.far_route((0, 0), (10, 0))
    b.obstacles((0.5, 0.0))

    position, source = b.goal()
    assert source == 'detour'
    assert position != (0.0, 0.0)


def test_without_a_route_far_planners_way_point_is_used_as_before(far):
    b = far()
    b.robot(0.0, 0.0)
    b.far_route()  # far_planner publishes an empty route when it has none: goal reached, or no way found
    b.way_point(4.0, 0.0)
    b.obstacles()

    assert b.goal() == ((2.0, 0.0), 'midpoint')


def test_a_route_is_enough_without_any_way_point(far):
    b = far()
    b.robot(0.0, 0.0)
    b.far_route((0, 0), (0, 10))
    b.obstacles()

    assert b.goal() == ((0.0, 2.0), 'path')


def test_without_route_topic_the_route_is_ignored(far):
    b = far(far_path=False)  # the TARE exploration: /way_point and TARE's paths, as before
    b.robot(0.0, 0.0)
    b.far_route((0, 0), (0, 10))
    b.way_point(4.0, 0.0)
    b.obstacles()

    assert b.goal() == ((2.0, 0.0), 'midpoint')


def test_with_tare_the_path_is_still_taken_only_when_the_straight_way_to_the_way_point_is_blocked(far):
    b = far(far_path=False)
    b.robot(0.0, 0.0)
    b.tare_path((0, 0), (0, 1.5), (4, 1.5), (4, 0))  # round something standing between the robot and the waypoint
    b.way_point(4.0, 0.0)
    b.obstacles()
    assert b.goal() == ((2.0, 0.0), 'midpoint')

    b.obstacles((2.0, 0.0))
    assert b.goal() == ((0.0, 1.5), 'path')  # the first corner of the path: no cutting it
