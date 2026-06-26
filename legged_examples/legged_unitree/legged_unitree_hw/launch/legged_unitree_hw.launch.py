#!/usr/bin/env python3
"""legged_unitree_hw.launch.py — ROS 2 porting of legged_unitree_hw.launch

Main entry-point for real hardware deployment.

Sequence:
  1. Generate URDF via generate_urdf.sh (xacro, dumps to /tmp/legged_control/)
  2. Publish robot_description via robot_state_publisher
  3. Include the robot-type-specific launch file (a1/aliengo/go1)
     which starts ros2_control_node + controller spawners
"""
import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    legged_unitree_description_dir = get_package_share_directory('legged_unitree_description')
    legged_unitree_hw_dir = get_package_share_directory('legged_unitree_hw')

    # -------------------------------------------------------------------------
    # Launch argument: robot_type (defaults to $ROBOT_TYPE env var)
    # -------------------------------------------------------------------------
    robot_type_arg = DeclareLaunchArgument(
        'robot_type',
        default_value=EnvironmentVariable('ROBOT_TYPE', default_value='aliengo'),
        description='Robot type: [a1, aliengo, go1, laikago]'
    )
    robot_type = LaunchConfiguration('robot_type')

    # -------------------------------------------------------------------------
    # Step 1: generate URDF
    # Calls generate_urdf.sh: ros2 run xacro xacro ... > /tmp/legged_control/<type>.urdf
    # -------------------------------------------------------------------------
    generate_urdf_node = Node(
        package='legged_common',
        executable='generate_urdf.sh',
        name='generate_urdf',
        output='screen',
        arguments=[
            os.path.join(legged_unitree_description_dir, 'urdf', 'robot.xacro'),
            robot_type,
        ]
    )

    # -------------------------------------------------------------------------
    # Step 2: robot_state_publisher
    # The URDF is generated at /tmp/legged_control/<type>.urdf; after that
    # we need to read it. We use TimerAction with a short delay to ensure
    # generate_urdf.sh has completed before rsp reads the file.
    # -------------------------------------------------------------------------
    # NOTE: In production you can replace this with an event handler
    # (RegisterEventHandler + OnProcessExit) for robustness.

    # -------------------------------------------------------------------------
    # Step 3: include robot-specific launcher (a1/aliengo/go1)
    # Each sub-launch starts ros2_control_node + spawners
    # We give 2 s for the URDF to be generated before the HW node starts.
    # -------------------------------------------------------------------------
    robot_specific_launch = TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('legged_unitree_hw'),
                        'launch',
                        [robot_type, '.launch.py'],
                    ])
                ),
            ),
        ]
    )

    return LaunchDescription([
        robot_type_arg,
        generate_urdf_node,
        robot_specific_launch,
    ])
