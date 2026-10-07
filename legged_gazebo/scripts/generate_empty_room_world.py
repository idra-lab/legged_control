#!/usr/bin/env python3
"""
generate_empty_room_world.py

Writes worlds/empty_room.world: a square room with nothing in it but a few packages on the floor and a few people
walking around, and the script with which people_perception fake_people_detector follows those people
(people_perception config/fake_people_empty_room_actors.yaml). Both come from the same paths: edit and run this
script rather than one of the two files.

    python3 generate_empty_room_world.py [--seed N] [--people N] [--packages N]

The people are scripted Gazebo actors, as in warehouse.world: Gazebo has no actor that wanders by itself, so "randomly"
is a random closed walk for each person, drawn here once (same seed, same world) and repeated by Gazebo every PERIOD.

How a walk is made: random corners across the room, a smooth closed curve through them, then WAYPOINTS points at
equal distance along that curve, one every WAYPOINT_DT. Gazebo moves an actor on a Catmull-Rom spline through its
waypoints (gz-math Spline, see people_perception scripts/fake_people_detector.py), so close and evenly spaced
waypoints give a steady pace on a smooth path. A walk is kept only if the path Gazebo will follow stays clear of the
walls, of the packages, of where the robot spawns and, at every instant, of the people already placed: every walk
lasts PERIOD, so two people who do not meet in the first lap never meet.
"""

import argparse
import math
from pathlib import Path

import numpy as np

ROOM = 30.0              # [m] side of the room, wall to wall
WALL_HEIGHT = 3.0        # [m]
WALL_THICKNESS = 0.2     # [m]
SPAWN = (0.0, 0.0)       # [m] default spawn of the robot in empty_room_launch.xml, facing +x
AIRY_AHEAD = 0.2835      # [m] the Airy is this far ahead of the base (robot.xacro): camera_init of FAST-LIO is there

PACKAGE_SIZE = ((0.4, 0.9), (0.3, 0.7), (0.25, 0.6))  # [m] ranges of length, width, height
PACKAGE_WALL_CLEARANCE = 2.5    # [m] centre of a package to a wall: room to walk around it
PACKAGE_SPAWN_CLEARANCE = 3.0   # [m] centre of a package to the spawn
PACKAGE_SPACING = 4.0           # [m] between the centres of two packages

PERIOD = 120.0           # [s] every walk ends where it started after this time
WAYPOINT_DT = 1.5        # [s] between two waypoints; a multiple of 1 ms, Gazebo keeps the times as integer ms
WAYPOINTS = round(PERIOD / WAYPOINT_DT)
CORNERS = 8              # random corners of a walk
CORNER_REACH = 2.0       # [m] corners no closer than this to a wall
LEG = (6.0, 16.0)        # [m] distance between two corners
MAX_CORNER_TURN = math.radians(120.0)
TURNS_EACH_WAY = 3       # corners turning left, and right: a walk that wanders, not a lap around the room
MEAN_SPEED = (0.6, 0.95)  # [m/s] OptiPessiController takes a person to move at 1.0 m/s at most (task.info)
MAX_SPEED = 1.0          # [m/s] at any instant
MAX_TURN_RATE = 1.0      # [rad/s] at any instant
WALL_CLEARANCE = 1.0     # [m] path of a person to a wall
PACKAGE_CLEARANCE = 0.8  # [m] path of a person to the side of a package
SPAWN_CLEARANCE = 2.0    # [m] path of a person to the spawn
PEOPLE_CLEARANCE = 1.5   # [m] between two people at the same instant
CHECK_DT = 0.05          # [s] step at which a walk is checked

PERSON_SIZE = (0.6, 0.6, 1.8)   # [m] box of a person for the detector
ACTOR_Z = 1.0            # [m] puts the hips of walk.dae at walking height
SENSOR_HEIGHT = 0.47     # [m] the Airy above the floor when the robot stands


def spline(points, samples):
    """
    `samples` positions per waypoint interval on the path of an actor whose waypoints are `points` (first = last):
    cubic Hermite segments with the tangents of gz-math Spline::RecalcTangents for a closed spline, tension 0.
    The last point is left out: row k is the position at k / samples intervals.
    """
    tangents = np.empty_like(points)
    tangents[1:-1] = (points[2:] - points[:-2]) / 2.0
    tangents[0] = tangents[-1] = (points[1] - points[-2]) / 2.0
    u = np.linspace(0.0, 1.0, samples, endpoint=False)[None, :, None]
    path = ((2 * u**3 - 3 * u**2 + 1) * points[:-1, None] + (u**3 - 2 * u**2 + u) * tangents[:-1, None] +
            (-2 * u**3 + 3 * u**2) * points[1:, None] + (u**3 - u**2) * tangents[1:, None])
    return path.reshape(-1, 2)


def wrap(angle):
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


def distance_to_package(points, package):
    """Distance [m] of each point to the footprint of the package (0 inside)."""
    x, y, yaw = package["pose"]
    cos, sin = math.cos(yaw), math.sin(yaw)
    dx, dy = points[:, 0] - x, points[:, 1] - y
    along, across = cos * dx + sin * dy, -sin * dx + cos * dy
    return np.hypot(np.maximum(np.abs(along) - package["size"][0] / 2.0, 0.0),
                    np.maximum(np.abs(across) - package["size"][1] / 2.0, 0.0))


def place_packages(rng, count):
    reach = ROOM / 2.0 - PACKAGE_WALL_CLEARANCE
    packages = []
    for _ in range(10000):
        if len(packages) == count:
            return packages
        size = [round(float(rng.uniform(low, high)), 2) for low, high in PACKAGE_SIZE]
        x, y = (round(float(v), 2) for v in rng.uniform(-reach, reach, 2))
        yaw = round(float(rng.uniform(-math.pi / 2.0, math.pi / 2.0)), 2)
        if math.hypot(x - SPAWN[0], y - SPAWN[1]) < PACKAGE_SPAWN_CLEARANCE:
            continue
        if any(math.hypot(x - p["pose"][0], y - p["pose"][1]) < PACKAGE_SPACING for p in packages):
            continue
        packages.append({"name": f"package_{len(packages) + 1}", "pose": (x, y, yaw), "size": size})
    raise RuntimeError(f"no room for {count} packages {PACKAGE_SPACING} m apart")


def random_corners(rng):
    """Endless closed polygons of CORNERS random corners: legs within LEG, turns as MAX_CORNER_TURN and TURNS_EACH_WAY."""
    reach = ROOM / 2.0 - CORNER_REACH
    batch = 20000
    while True:
        # Drawn leg after leg from a random start, then kept if they stay in the room and the leg back to the start
        # fits too: corners drawn anywhere in the room almost never do
        corners = np.empty((batch, CORNERS, 2))
        corners[:, 0] = rng.uniform(-reach, reach, (batch, 2))
        heading = rng.uniform(-np.pi, np.pi, batch)
        for i in range(1, CORNERS):
            heading = heading + rng.uniform(-MAX_CORNER_TURN, MAX_CORNER_TURN, batch)
            corners[:, i] = corners[:, i - 1] + rng.uniform(*LEG, batch)[:, None] * np.column_stack(
                [np.cos(heading), np.sin(heading)])
        legs = np.roll(corners, -1, axis=1) - corners
        length = np.linalg.norm(legs, axis=2)
        heading = np.arctan2(legs[..., 1], legs[..., 0])
        turn = wrap(heading - np.roll(heading, 1, axis=1))
        yield from corners[(np.abs(corners).max(axis=(1, 2)) <= reach) &
                           (length.min(axis=1) >= LEG[0]) & (length.max(axis=1) <= LEG[1]) &
                           (np.abs(turn).max(axis=1) <= MAX_CORNER_TURN) &
                           ((turn > 0.0).sum(axis=1) >= TURNS_EACH_WAY) & ((turn < 0.0).sum(axis=1) >= TURNS_EACH_WAY)]


def waypoints_through(corners):
    """WAYPOINTS + 1 points at equal distance along the closed curve through the corners, the last on the first."""
    curve = spline(np.vstack([corners, corners[:1]]), 200)
    curve = np.vstack([curve, curve[:1]])
    travelled = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(curve, axis=0), axis=1))])
    at = np.linspace(0.0, travelled[-1], WAYPOINTS, endpoint=False)
    points = np.column_stack([np.interp(at, travelled, curve[:, 0]), np.interp(at, travelled, curve[:, 1])])
    points = np.round(points, 3)  # as written in the two files
    return np.vstack([points, points[:1]])


def walk_is_good(path, packages, others):
    """path: the positions of a person every CHECK_DT over one PERIOD; others: those of the people already placed."""
    step = np.roll(path, -1, axis=0) - path
    speed = np.linalg.norm(step, axis=1) / CHECK_DT
    heading = np.arctan2(step[:, 1], step[:, 0])
    turn_rate = np.abs(wrap(np.roll(heading, -1) - heading)) / CHECK_DT
    return (MEAN_SPEED[0] <= speed.mean() <= MEAN_SPEED[1] and speed.max() <= MAX_SPEED and
            turn_rate.max() <= MAX_TURN_RATE and
            np.abs(path).max() <= ROOM / 2.0 - WALL_CLEARANCE and
            np.linalg.norm(path - SPAWN, axis=1).min() >= SPAWN_CLEARANCE and
            all(distance_to_package(path, package).min() >= PACKAGE_CLEARANCE for package in packages) and
            all(np.linalg.norm(path - other, axis=1).min() >= PEOPLE_CLEARANCE for other in others))


def place_people(rng, count, packages):
    people, paths = [], []
    for tried, corners in enumerate(random_corners(rng)):
        if len(people) == count:
            return people
        if tried == 20000:
            raise RuntimeError(f"no room for {count} people: {len(people)} placed")
        points = waypoints_through(corners)
        path = spline(points, round(WAYPOINT_DT / CHECK_DT))
        if not walk_is_good(path, packages, paths):
            continue
        # Facing the way the spline leaves each waypoint. Not wrapped to +-pi: Gazebo interpolates the quaternions,
        # and those of two headings on the two sides of pi have opposite signs
        tangents = np.vstack([points[1] - points[-2], points[2:] - points[:-2], points[1] - points[-2]])
        yaw = np.round(np.unwrap(np.arctan2(tangents[:, 1], tangents[:, 0])), 4)
        people.append({"name": f"person_{len(people) + 1}", "times": np.arange(WAYPOINTS + 1) * WAYPOINT_DT,
                       "points": points, "yaw": yaw,
                       "speed": float(np.linalg.norm(np.diff(np.vstack([path, path[:1]]), axis=0), axis=1).sum())
                       / PERIOD})
        paths.append(path)


def box_model(name, pose, size, color):
    x, y, z, yaw = pose
    box = f"<geometry><box><size>{size[0]:g} {size[1]:g} {size[2]:g}</size></box></geometry>"
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x:g} {y:g} {z:g} 0 0 {yaw:g}</pose>
      <link name="link">
        <collision name="collision">
          {box}
        </collision>
        <visual name="visual">
          {box}
          <material>
            <ambient>{color}</ambient>
            <diffuse>{color}</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""


def actor(person):
    waypoints = "".join(f"""
          <waypoint>
            <time>{t:g}</time>
            <pose>{x:g} {y:g} {ACTOR_Z:g} 0 0 {yaw:g}</pose>
          </waypoint>""" for t, (x, y), yaw in zip(person["times"], person["points"], person["yaw"]))
    return f"""
    <actor name="{person["name"]}">
      <skin>
        <filename>model://walking_actor/meshes/walk.dae</filename>
        <scale>1.0</scale>
      </skin>
      <animation name="walking">
        <filename>model://walking_actor/meshes/walk.dae</filename>
        <scale>1.0</scale>
        <interpolate_x>true</interpolate_x>
      </animation>
      <script>
        <loop>true</loop>
        <delay_start>0</delay_start>
        <auto_start>true</auto_start>

        <trajectory id="0" type="walking">{waypoints}
        </trajectory>
      </script>
    </actor>
"""


def world_file(packages, people, settings):
    half = ROOM / 2.0
    centre = half + WALL_THICKNESS / 2.0  # of a wall: its inner face is the side of the room
    outside = ROOM + 2.0 * WALL_THICKNESS
    walls = "".join(box_model(name, (x, y, WALL_HEIGHT / 2.0, 0.0), size, "0.85 0.85 0.85 1") for name, x, y, size in (
        ("wall_north", 0.0, centre, (outside, WALL_THICKNESS, WALL_HEIGHT)),
        ("wall_south", 0.0, -centre, (outside, WALL_THICKNESS, WALL_HEIGHT)),
        ("wall_east", centre, 0.0, (WALL_THICKNESS, ROOM, WALL_HEIGHT)),
        ("wall_west", -centre, 0.0, (WALL_THICKNESS, ROOM, WALL_HEIGHT))))
    boxes = "".join(box_model(p["name"], (p["pose"][0], p["pose"][1], p["size"][2] / 2.0, p["pose"][2]), p["size"],
                              "0.72 0.53 0.33 1") for p in packages)
    return f"""<?xml version="1.0" ?>
<!-- Written by legged_gazebo/scripts/generate_empty_room_world.py ({settings}):
     change that script and run it again rather than editing this file. The same run writes people_perception
     config/fake_people_empty_room_actors.yaml, the people of this world for the fake people detector.

     A {ROOM:g} x {ROOM:g} m room centred on the world origin (walls {WALL_HEIGHT:g} m high, no ceiling), {len(packages)} static packages on the
     floor and {len(people)} people. Each person repeats a random closed walk of {PERIOD:g} s, clear of the walls, of the packages,
     of the other people and of the world origin, where empty_room_launch.xml spawns the robot. -->
<sdf version="1.7">
  <world name="empty_room">
    <physics name="default_physics" default="true" type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <!-- Drives <sensor type="imu"> (the IMU of the lidar); the base IMU is read by LeggedHWSim -->
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.3 0.7 0.9 1</background>
      <shadows>0</shadows>
      <grid>1</grid>
    </scene>

    <!-- Gives the faces of the boxes different shades; no shadows -->
    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- Contact as in the other worlds; the visual is the floor of the room only -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>100</mu>
                <mu2>50</mu2>
              </ode>
            </friction>
            <contact>
              <ode>
                <soft_cfm>0</soft_cfm>
                <soft_erp>0.2</soft_erp>
                <kp>1e+13</kp>
                <kd>1</kd>
                <max_vel>0.01</max_vel>
                <min_depth>0.001</min_depth>
              </ode>
            </contact>
          </surface>
        </collision>
        <visual name="visual">
          <cast_shadows>false</cast_shadows>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>{outside:g} {outside:g}</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <!-- Walls: the inner faces are at +-{half:g} m -->
{walls}
    <!-- Packages -->
{boxes}
    <!-- People. Actors are animated visuals only (no collision): cameras and the lidar see them, the robot does not
         bump into them. The yaw of a waypoint is the direction of the walk there. -->
{"".join(actor(person) for person in people)}
  </world>
</sdf>
"""


def people_script(people, settings):
    entries = "".join(f"""  - name: {person["name"]}              # {person["speed"]:.2f} m/s
    size: [{", ".join(f"{v:g}" for v in PERSON_SIZE)}]
    floor_z: {-SENSOR_HEIGHT:g}
    loop: true
    delay_start: 0.0
    interpolation: gazebo
    waypoints:                  # [t, x, y] in the world
""" + "".join(f"      - [{t:g}, {x:g}, {y:g}]\n" for t, (x, y) in zip(person["times"], person["points"]))
                      for person in people)
    return f"""# Script for fake_people_detector that follows the {len(people)} walking actors of legged_gazebo/worlds/empty_room.world.
# Written with that world by legged_gazebo/scripts/generate_empty_room_world.py ({settings}):
# change that script and run it again rather than editing this file.
#
# The waypoints are those of the world file, in world coordinates. interpolation: gazebo moves them as Gazebo
# Harmonic moves a scripted actor (see scripts/fake_people_detector.py): script time is the simulation time, which is
# the stamp of the scans.
#
# origin is where the world origin is in camera_init. It holds only if
#   - the robot is at the default spawn of empty_room_launch.xml (x:={SPAWN[0]:g} y:={SPAWN[1]:g}, facing +x), and
#   - FAST-LIO is started with the robot standing there (camera_init is where the Airy is at that moment), and
#   - Gazebo has run since the world was loaded (the actors move with the simulation time).
# Spawned or started elsewhere, change origin: [x, y, yaw] of the world origin seen from camera_init.
frame: camera_init
origin: [{-(SPAWN[0] + AIRY_AHEAD):g}, {0.0 - SPAWN[1]:g}, 0.0]      # the Airy is {AIRY_AHEAD:g} m ahead of the base, no turn
start_time: 0.0
people:
{entries}"""


def main():
    package = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description="Write worlds/empty_room.world and the fake people script of it")
    parser.add_argument("--seed", type=int, default=3, help="Of the random packages and walks")
    parser.add_argument("--people", type=int, default=4)
    parser.add_argument("--packages", type=int, default=7)
    parser.add_argument("--world", type=Path, default=package / "worlds" / "empty_room.world")
    parser.add_argument("--people-script", type=Path, default=package.parents[1] / "people_perception" /
                        "people_perception" / "config" / "fake_people_empty_room_actors.yaml")
    args = parser.parse_args()

    rng = np.random.default_rng(args.seed)
    packages = place_packages(rng, args.packages)
    people = place_people(rng, args.people, packages)

    settings = f"seed {args.seed}, {args.people} people, {args.packages} packages"  # no "--": it goes in an XML comment
    args.world.write_text(world_file(packages, people, settings))
    args.people_script.write_text(people_script(people, settings))
    for p in packages:
        print(f"{p['name']}: {p['size'][0]:g} x {p['size'][1]:g} x {p['size'][2]:g} m at ({p['pose'][0]:g}, "
              f"{p['pose'][1]:g})")
    for person in people:
        print(f"{person['name']}: {person['speed'] * PERIOD:.0f} m in {PERIOD:g} s, {person['speed']:.2f} m/s")
    print(f"wrote {args.world}\nwrote {args.people_script}")


if __name__ == "__main__":
    main()
