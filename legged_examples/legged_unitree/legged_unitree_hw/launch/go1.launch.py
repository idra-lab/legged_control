#!/usr/bin/env python3
"""go1.launch.py — ROS 2 porting of go1.launch

In ROS 2, legged_unitree_hw is a hardware_interface plugin (not a standalone
executable). It is loaded via ros2_control_node with the robot URDF that
contains the <ros2_control> tag describing the HardwareInterface.

This launcher:
  1. Starts ros2_control_node with the Go1 URDF and hardware config YAML
  2. Spawns all required controllers
  (Uses Unitree SDK 3.8.0 — different from A1/Aliengo)
"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    legged_unitree_hw_dir = get_package_share_directory('legged_unitree_hw')
    legged_controllers_dir = get_package_share_directory('legged_controllers')

    robot_type = 'go1'

    hw_config_yaml = os.path.join(legged_unitree_hw_dir, 'config', f'{robot_type}.yaml')
    controllers_yaml = os.path.join(legged_controllers_dir, 'config', 'controllers.yaml')
    urdf_file = f'/tmp/legged_control/{robot_type}.urdf'

    # ros2_control_node loads the hardware plugin via the URDF's <ros2_control> tag.
    # The Unitree SDK 3.8.0 plugin is used for Go1.
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='controller_manager',
        output='screen',
        parameters=[
            hw_config_yaml,
            controllers_yaml,
            {'robot_description': open(urdf_file).read() if os.path.exists(urdf_file) else ''},
        ],
        remappings=[
            ('~/robot_description', '/robot_description'),
        ]
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    imu_sensor_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['imu_sensor_broadcaster'],
        output='screen',
    )

    legged_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['legged_controller'],
        output='screen',
    )

    return LaunchDescription([
        ros2_control_node,
        joint_state_broadcaster_spawner,
        imu_sensor_broadcaster_spawner,
        legged_controller_spawner,
    ])
