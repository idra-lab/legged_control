#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from unitree_go.msg import LowState, LowCmd, MotorState, MotorCmd
from geometry_msgs.msg import Point
from std_msgs.msg import String

class NetworkJointRemapNode(Node):
    def __init__(self):
        super().__init__('network_joint_remap_node')

        # 1. Channels communicating with the remote Isaac Sim PC
        self.isaac_sub = self.create_subscription(JointState, '/isaac_joint_states', self.isaac_telemetry_callback, 10)
        self.isaac_pub = self.create_publisher(JointState, '/isaac_joint_commands', 10)
        self.isaac_pub_walk = self.create_publisher(JointState, '/isaac_joint_commands_walk', 10)

        # 2. Channels communicating locally with your compiled C++ binaries
        self.unitree_pub = self.create_publisher(LowState, '/lowstate', 10)
        self.unitree_sub = self.create_subscription(LowCmd, '/lowcmd', self.unitree_command_callback, 10)

        # Index Map Matrix: Converts Isaac Type-grouping to Unitree Leg-grouping
        # Isaac: [FL_hip(0), FR_hip(1), RL_hip(2), RR_hip(3), FL_thigh(4), FR_thigh(5)...]
        self.unitree_to_isaac_mapping = [0, 4, 8, 1, 5, 9, 2, 6, 10, 3, 7, 11]
        
        # Hardcoded ordered string list matching Isaac Sim's structural requirements
        self.isaac_joint_names = [
            "FL_hip_joint", "FR_hip_joint", "RL_hip_joint", "RR_hip_joint",
            "FL_thigh_joint", "FR_thigh_joint", "RL_thigh_joint", "RR_thigh_joint",
            "FL_calf_joint", "FR_calf_joint", "RL_calf_joint", "RR_calf_joint"
        ]

        self.get_logger().info("Network ROS 2 Index Remapper Started successfully.")

        self.unitree_state_velocities = [0.0] * 12
        self.unitree_state_positions = [0.0] * 12
        
        # Real-time joint gain tuning
        self.kp_kd_sub = self.create_subscription(Point, '/joint_kp_kd', self.kp_kd_callback, 10)
        self.kp = 35.0
        self.kd = 1.5

        # Exponential moving average filter for joint velocities (reduces D-term vibration)
        self.dq_filtered = [0.0] * 12
        self.filter_alpha = 0.3  # filter coefficient (0.0 < alpha <= 1.0)

        self.robot_mode_sub = self.create_subscription(String, '/robot_mode', self.robot_mode_callback, 10)
        self.robot_mode = ""

    def kp_kd_callback(self, msg: Point):
        self.kp = msg.x
        self.kd = msg.y

    def robot_mode_callback(self, msg: String):
        self.robot_mode = msg.data

    def isaac_telemetry_callback(self, isaac_msg: JointState):
        """ Receives type-grouped arrays from remote PC and builds leg-grouped Unitree packets """
        unitree_state = LowState()
        
        # Initialize the lowstate structural array
        for _ in range(20):
            unitree_state.motor_state.append(MotorState())

        # Map joint names to their index in isaac_msg
        name_to_idx = {name: idx for idx, name in enumerate(isaac_msg.name)}

        # Remap values dynamically by name lookup to be robust against publisher joint ordering
        for u_idx, i_idx in enumerate(self.unitree_to_isaac_mapping):
            joint_name = self.isaac_joint_names[i_idx]
            if joint_name in name_to_idx:
                msg_idx = name_to_idx[joint_name]
                if msg_idx < len(isaac_msg.position):
                    unitree_state.motor_state[u_idx].q = isaac_msg.position[msg_idx]
                if msg_idx < len(isaac_msg.velocity):
                    unitree_state.motor_state[u_idx].dq = isaac_msg.velocity[msg_idx]

        # Publish locally for your workspace binaries
        self.unitree_pub.publish(unitree_state)

        self.unitree_state_velocities = [motor.dq for motor in unitree_state.motor_state]
        self.unitree_state_positions = [motor.q for motor in unitree_state.motor_state]

    def unitree_command_callback(self, unitree_cmd: LowCmd):
        isaac_cmd = JointState()
        isaac_cmd.header.stamp = self.get_clock().now().to_msg()
        isaac_cmd.name = self.isaac_joint_names
        isaac_cmd.position = [0.0] * 12
        isaac_cmd.velocity = [0.0] * 12
        isaac_cmd.effort = [0.0] * 12

        for u_idx, i_idx in enumerate(self.unitree_to_isaac_mapping):
            motor = unitree_cmd.motor_cmd[u_idx]
            isaac_cmd.position[i_idx] = motor.q
            isaac_cmd.velocity[i_idx] = motor.dq
            isaac_cmd.effort[i_idx] = motor.tau

        if self.robot_mode == "walk":
            self.isaac_pub_walk.publish(isaac_cmd)
        else: 
            self.isaac_pub.publish(isaac_cmd)


def main(args=None):
    rclpy.init(args=args)
    node = NetworkJointRemapNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()