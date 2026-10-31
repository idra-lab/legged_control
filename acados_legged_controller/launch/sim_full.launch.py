import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('acados_legged_controller')

    # Gazebo, robot_state_publisher and spawn of the robot
    sim = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(os.path.join(share, 'launch', 'sim_launch.xml')))

    # Broadcasters first: the controller needs the imu and joint interfaces
    broadcasters = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=['joint_state_broadcaster', 'imu_sensor_broadcaster',
                   '--controller-manager-timeout', '120'])

    # No --inactive: load, configure and activate. on_init is slow, hence the long timeout
    controller = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=['acados_legged_controller', '--service-call-timeout', '300.0'])

    # The estimator publishes /odom but not the TF odom -> base
    odom_to_tf = Node(
        package='acados_legged_controller', executable='odom_to_tf.py', output='screen',
        parameters=[{'use_sim_time': True}])

    rviz = Node(
        package='rviz2', executable='rviz2', output='screen',
        arguments=['-d', os.path.join(share, 'config', 'sim.rviz')],
        parameters=[{'use_sim_time': True}])

    return LaunchDescription([
        sim,
        broadcasters,
        odom_to_tf,
        rviz,
        RegisterEventHandler(OnProcessExit(target_action=broadcasters, on_exit=[controller])),
    ])
