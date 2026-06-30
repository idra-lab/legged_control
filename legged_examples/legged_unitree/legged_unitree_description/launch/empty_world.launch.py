#!/usr/bin/env python3
"""empty_world.launch.py — ROS 2 launch for Gazebo Harmonic

Launches Gazebo Harmonic (gz sim) with an empty world and spawns the Unitree
robot using the ros_gz_sim 'create' node (replaces the old gazebo_ros
spawn_entity.py).

Key design decisions:
- URDF is generated synchronously at launch-time (before any node starts) via
  OpaqueFunction + subprocess, so robot_description is always populated.
- Model spawning is delegated to the ros_gz_sim 'create' node which reads the
  robot_description topic published by robot_state_publisher.
"""
import os
import re
import subprocess
import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    # Prepend /opt/openrobots/lib so that nodes (and pluginlib dynamic loaders) can link to pinocchio
    os.environ['LD_LIBRARY_PATH'] = f"/opt/openrobots/lib:{os.environ.get('LD_LIBRARY_PATH', '')}"

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
        parameters=[{
            'robot_description': robot_description_xml,
            'use_sim_time': True,
        }],
    )

    # Bridge Gazebo clock to ROS 2 /clock so all nodes using use_sim_time
    # receive a valid simulation clock source.
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    # Spawn robot using ros_gz_sim 'create' node (Gazebo Harmonic equivalent
    # of the old gazebo_ros spawn_entity.py).
    spawn_entity_action = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', robot_type_str,
            '-topic', 'robot_description',
            '-z', '0.5',
        ],
        output='screen',
        name='spawn_urdf',
    )

    # Determine if Gazebo should start with GUI or headless (server-only)
    gui_val = context.perform_substitution(LaunchConfiguration('gui'))
    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')
    world_file = os.path.join(get_package_share_directory('legged_gazebo'), 'worlds', 'empty_world.world')

    if gui_val.lower() == 'false':
        gz_args = f'-v 4 -r -s {world_file}'
    else:
        gz_args = f'-v 4 -r {world_file}'

    # Gazebo Harmonic — use gz_sim.launch.py from ros_gz_sim.
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': gz_args,
            'on_exit_shutdown': 'true',
        }.items(),
    )

    return [
        gazebo_launch,
        clock_bridge,
        robot_state_publisher_node,
        spawn_entity_action,
    ]


def generate_launch_description():
    robot_type_arg = DeclareLaunchArgument(
        'robot_type',
        default_value='aliengo',
        description='Robot type: [a1, aliengo, go1, laikago]'
    )

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Start Gazebo client (GUI)'
    )

    return LaunchDescription([
        robot_type_arg,
        gui_arg,
        OpaqueFunction(function=launch_setup),
    ])
