#!/usr/bin/env python3
"""load_controller.launch.py — ROS 2 porting of load_controller.launch

Launches:
  - controller_manager spawners (joint_state_broadcaster, legged_controller,
    imu_sensor_broadcaster)
  - legged_robot_gait_command or legged_robot_gait_joy_command (optional joy)
  - legged_target_trajectories_publisher
  - kill_node_sigint_service (when joy is used)
  - joy_node + joy_teleop (when joy is used)
  - rviz2
  - multiplot (optional)
"""
import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    legged_controllers_dir = get_package_share_directory('legged_controllers')

    # -------------------------------------------------------------------------
    # Launch arguments
    # -------------------------------------------------------------------------
    robot_type_arg = DeclareLaunchArgument(
        'robot_type',
        default_value='aliengo',
        description='Robot type: [a1, aliengo, go1, laikago]'
    )
    joy_arg = DeclareLaunchArgument(
        'joy',
        default_value='false',
        description='Enable joystick teleoperation'
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2'
    )
    description_name_arg = DeclareLaunchArgument(
        'description_name',
        default_value='legged_robot_description',
        description='Robot description parameter name'
    )
    multiplot_arg = DeclareLaunchArgument(
        'multiplot',
        default_value='false',
        description='Enable multiplot'
    )
    joy_dev_arg = DeclareLaunchArgument(
        'joy_dev',
        default_value='/dev/input/js0',
        description='Joystick device'
    )

    robot_type = LaunchConfiguration('robot_type')
    joy = LaunchConfiguration('joy')
    rviz = LaunchConfiguration('rviz')
    multiplot = LaunchConfiguration('multiplot')
    joy_dev = LaunchConfiguration('joy_dev')

    # -------------------------------------------------------------------------
    # Config paths — evaluated at runtime via PathJoinSubstitution
    # -------------------------------------------------------------------------
    task_file = PathJoinSubstitution(
        [legged_controllers_dir, 'config', robot_type, 'task.info']
    )
    reference_file = PathJoinSubstitution(
        [legged_controllers_dir, 'config', robot_type, 'reference.info']
    )
    gait_command_file = PathJoinSubstitution(
        [legged_controllers_dir, 'config', robot_type, 'gait.info']
    )
    controllers_yaml = os.path.join(legged_controllers_dir, 'config', 'controllers.yaml')
    joy_yaml = os.path.join(legged_controllers_dir, 'config', 'joy.yaml')
    rviz_config = os.path.join(legged_controllers_dir, 'config', 'config.rviz')

    # -------------------------------------------------------------------------
    # Controller spawners
    # -------------------------------------------------------------------------
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    imu_sensor_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['imu_sensor_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    legged_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['legged_controller', '--controller-manager', '/controller_manager'],
        output='screen',
        parameters=[{
            'urdfFile': PathJoinSubstitution(['/tmp/legged_control/', [robot_type, '.urdf']]),
            'taskFile': task_file,
            'referenceFile': reference_file,
            'gaitCommandFile': gait_command_file,
        }]
    )

    # -------------------------------------------------------------------------
    # Gait command nodes (keyboard vs joystick)
    # -------------------------------------------------------------------------
    gait_command_node = Node(
        package='ocs2_legged_robot_ros',
        executable='legged_robot_gait_command',
        name='legged_robot_gait_command',
        output='screen',
        condition=UnlessCondition(joy),
        parameters=[{'gaitCommandFile': gait_command_file}],
    )

    gait_joy_command_node = Node(
        package='ocs2_legged_robot_ros',
        executable='legged_robot_gait_joy_command',
        name='legged_robot_gait_joy_command',
        output='screen',
        condition=IfCondition(joy),
        parameters=[{'gaitCommandFile': gait_command_file}],
    )

    # -------------------------------------------------------------------------
    # Target trajectories publisher
    # -------------------------------------------------------------------------
    target_trajectories_publisher = Node(
        package='legged_controllers',
        executable='legged_target_trajectories_publisher',
        name='legged_robot_target',
        output='screen',
        parameters=[{
            'taskFile': task_file,
            'referenceFile': reference_file,
        }],
    )

    # -------------------------------------------------------------------------
    # Joy nodes (only when joy=true)
    # -------------------------------------------------------------------------
    kill_node_service = Node(
        package='legged_controllers',
        executable='kill_node_sigint_service.py',
        name='kill_node_sigint',
        output='screen',
        condition=IfCondition(joy),
        parameters=[{'allow_kill': True}],
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        condition=IfCondition(joy),
        parameters=[{
            'dev': joy_dev,
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
        condition=IfCondition(joy),
        parameters=[joy_yaml],
    )

    # -------------------------------------------------------------------------
    # RViz2
    # -------------------------------------------------------------------------
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(rviz),
    )

    # -------------------------------------------------------------------------
    # Multiplot (optional)
    # -------------------------------------------------------------------------
    ocs2_legged_robot_ros_dir = get_package_share_directory('ocs2_legged_robot_ros')
    multiplot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ocs2_legged_robot_ros_dir, 'launch', 'multiplot.launch.py')
        ),
        condition=IfCondition(multiplot),
    )

    return LaunchDescription([
        robot_type_arg,
        joy_arg,
        rviz_arg,
        description_name_arg,
        multiplot_arg,
        joy_dev_arg,
        # Spawners
        joint_state_broadcaster_spawner,
        imu_sensor_broadcaster_spawner,
        legged_controller_spawner,
        # Gait
        gait_command_node,
        gait_joy_command_node,
        # Targets
        target_trajectories_publisher,
        # Joy
        kill_node_service,
        joy_node,
        joy_teleop_node,
        # Viz
        rviz_node,
        multiplot_launch,
    ])
