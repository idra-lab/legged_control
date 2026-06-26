from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # ===================================================================
        # PIPELINE COMPLETA: Opti-Pessi MPC → WBC → Isaac Sim
        # ===================================================================

        # 1. Bridge di comunicazione con Isaac Sim
        #    Isaac Sim ↔ ROS 2: /isaac_joint_states → /lowstate, /lowcmd → /isaac_joint_commands
        Node(
            package='legged_opti_pessi_mpc',
            executable='isaac_go2_bridge.py',
            name='isaac_go2_bridge',
            output='screen'
        ),

        # 2. State Estimator: /lowstate → /odom
        #    Converte IMU + giunti in odometria per alimentare l'MPC
        # Node(
        #     package='opti_pessi_control',
        #     executable='state_estimator_node',
        #     name='state_estimator',
        #     output='screen'
        # ),

        # 3. Opti-Pessi MPC: /odom → /legged_robot_mpc_policy
        #    Solutore SQP centroidale che genera la traiettoria ottimale
        Node(
            package='legged_opti_pessi_mpc',
            executable='opti_pessi_mpc_node',
            name='opti_pessi_mpc_node',
            output='screen'
        ),

        # 4. Whole-Body Controller: /legged_robot_mpc_policy + /lowstate → /lowcmd
        #    Converte la traiettoria centroidale in comandi giunto-livello via IK
        Node(
            package='legged_opti_pessi_mpc',
            executable='wbc_controller_node',
            name='wbc_controller',
            output='screen',
            parameters=[{
                'kp': 35.0,
                'kd': 1.5,
                'swing_height': 0.06,
            }]
        ),

        # 5. MPC Interactive GUI
        Node(
            package='legged_opti_pessi_mpc',
            executable='mpc_interactor_gui.py',
            name='mpc_interactive_gui',
            output='screen'
        ),
    ])