#!/usr/bin/env python3
"""empty_world.launch.py — ROS 2 porting of empty_world.launch

Launches a Gazebo simulation with an empty world and spawns the Unitree robot.
Uses gazebo_ros ROS 2 API and xacro Python API to generate the robot URDF.

Key design decisions:
- URDF is generated synchronously at launch-time (before any node starts) via
  OpaqueFunction + subprocess, so robot_description is always populated.
- spawn_entity is wrapped in a polling loop that waits up to 120s for the
  /spawn_entity service to appear, working around slow gzserver startups.
"""
import os
import re
import subprocess
import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    robot_type_str = context.perform_substitution(LaunchConfiguration('robot_type'))

    legged_unitree_description_dir = get_package_share_directory('legged_unitree_description')
    legged_controllers_dir = get_package_share_directory('legged_controllers')

    # Ensure target temp directory exists
    os.makedirs('/tmp/legged_control', exist_ok=True)
    urdf_path = f'/tmp/legged_control/{robot_type_str}.urdf'
    xacro_path = os.path.join(legged_unitree_description_dir, 'urdf', 'robot.xacro')

    # Generate temporary controllers.yaml for Gazebo containing robot-specific
    # paths (taskFile, urdfFile, etc.) used by the legged_controller plugin.
    controllers_yaml_src = os.path.join(legged_controllers_dir, 'config', 'controllers.yaml')
    print(f"[Launch] Loading source controllers configuration: {controllers_yaml_src}")
    with open(controllers_yaml_src, 'r') as f:
        config = yaml.safe_load(f)

    if 'legged_controller' not in config:
        config['legged_controller'] = {}
    if 'ros__parameters' not in config['legged_controller']:
        config['legged_controller']['ros__parameters'] = {}

    config['legged_controller']['ros__parameters'].update({
        'urdfFile': urdf_path,
        'taskFile': os.path.join(legged_controllers_dir, 'config', robot_type_str, 'task.info'),
        'referenceFile': os.path.join(legged_controllers_dir, 'config', robot_type_str, 'reference.info'),
        'gaitCommandFile': os.path.join(legged_controllers_dir, 'config', robot_type_str, 'gait.info'),
    })

    controllers_yaml_dest = f'/tmp/legged_control/controllers_{robot_type_str}.yaml'
    print(f"[Launch] Generating temporary controllers YAML file: {controllers_yaml_dest}")
    with open(controllers_yaml_dest, 'w') as f:
        yaml.safe_dump(config, f)

    # Generate the URDF synchronously (xacro → URDF file + in-memory string)
    print(f"[Launch] Generating URDF file: {urdf_path}")
    subprocess.run([
        'ros2', 'run', 'xacro', 'xacro',
        xacro_path,
        f'robot_type:={robot_type_str}',
        f'controllers_yaml:={controllers_yaml_dest}',
        '-o', urdf_path
    ], check=True)

    with open(urdf_path, 'r') as f:
        robot_description_xml = f.read()

    # Strip XML comments to prevent gazebo_ros2_control parser errors caused by '--' in '-->'
    robot_description_xml = re.sub(r'<!--.*?-->', '', robot_description_xml, flags=re.DOTALL)

    # Overwrite the URDF file with the cleaned XML so all nodes (including OCS2) read the comment-free version
    with open(urdf_path, 'w') as f:
        f.write(robot_description_xml)

    # robot_state_publisher publishes /robot_description for spawn_entity and TF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_xml}],
    )

    # spawn_entity wrapper: polls /spawn_entity until Gazebo is ready (up to
    # 120s) before invoking spawn_entity.py.  This avoids the hard 30s timeout
    # that was causing failures when gzserver took longer to start.
    spawn_script = "\n".join([
        "import subprocess, sys, time",
        "print('[spawn_wait] Waiting for /spawn_entity service (up to 120 s)...')",
        "for i in range(120):",
        "    r = subprocess.run(['ros2', 'service', 'list'], capture_output=True, text=True)",
        "    if '/spawn_entity' in r.stdout:",
        "        print('[spawn_wait] /spawn_entity found — spawning robot!')",
        "        break",
        "    time.sleep(1)",
        "else:",
        "    print('[spawn_wait] ERROR: /spawn_entity did not appear after 120 s', file=sys.stderr)",
        "    sys.exit(1)",
        "subprocess.run([",
        "    'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',",
        "    '-z', '0.5',",
        "    '-topic', 'robot_description',",
        f"    '-entity', '{robot_type_str}',",
        "], check=True)",
    ])

    spawn_entity_action = ExecuteProcess(
        cmd=['python3', '-c', spawn_script],
        output='screen',
        name='spawn_urdf',
    )

    return [
        robot_state_publisher_node,
        spawn_entity_action,
    ]


def generate_launch_description():
    legged_gazebo_dir = get_package_share_directory('legged_gazebo')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')

    robot_type_arg = DeclareLaunchArgument(
        'robot_type',
        default_value='aliengo',
        description='Robot type: [a1, aliengo, go1, laikago]'
    )

    # Gazebo — verbose=true so we can see plugin loading in the terminal
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': os.path.join(legged_gazebo_dir, 'worlds', 'empty_world.world'),
            'verbose': 'true',
        }.items(),
    )

    return LaunchDescription([
        robot_type_arg,
        gazebo_launch,
        OpaqueFunction(function=launch_setup),
    ])
