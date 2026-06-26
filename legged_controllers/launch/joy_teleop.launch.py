#!/usr/bin/env python3
"""joy_teleop.launch.py — ROS 2 porting of joy_teleop.launch"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    legged_controllers_dir = get_package_share_directory('legged_controllers')

    joy_dev_arg = DeclareLaunchArgument(
        'joy_dev',
        default_value='/dev/input/js0',
        description='Joystick device'
    )
    teleop_config_arg = DeclareLaunchArgument(
        'teleop_config',
        default_value=os.path.join(legged_controllers_dir, 'config', 'joy.yaml'),
        description='Teleop YAML config file'
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'dev': LaunchConfiguration('joy_dev'),
            'deadzone': 1e-3,
            'autorepeat_rate': 10.0,
            'coalesce_interval': 0.05,
        }]
    )

    joy_teleop_node = Node(
        package='joy_teleop',
        executable='joy_teleop',
        name='joy_teleop',
        output='screen',
        parameters=[LaunchConfiguration('teleop_config')]
    )

    return LaunchDescription([
        joy_dev_arg,
        teleop_config_arg,
        joy_node,
        joy_teleop_node,
    ])
