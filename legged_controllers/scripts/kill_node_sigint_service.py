#!/usr/bin/env python3
"""
kill_node_sigint_service.py (ROS 2 Python)

Service: /kill_node_sigint
Service type: legged_controllers/srv/KillNode

Behavior:
 - Listens on the service /kill_node_sigint.
 - When a request is received, scans local processes (/proc) or runs system tools
   to find matching process(es) for the requested node name.
 - Sends SIGINT to the identified PID(s).
 - Requires parameter 'allow_kill' == True to actually send the signal.
"""

import os
import signal
import sys
import rclpy
from rclpy.node import Node
from legged_controllers.srv import KillNode

class KillNodeSigintService(Node):
    def __init__(self):
        super().__init__('kill_node_sigint_service')
        self.declare_parameter('allow_kill', False)
        self.allow_kill = self.get_parameter('allow_kill').get_parameter_value().bool_value
        
        if not self.allow_kill:
            self.get_logger().warn("KILLING IS DISABLED by default. Start with parameter 'allow_kill':=true to enable.")
            
        self.srv = self.create_service(KillNode, '/kill_node_sigint', self.handle_kill_request)
        self.get_logger().info("Starting kill_node_sigint_service")

    def handle_kill_request(self, request, response):
        # Refresh parameter value in case it changed dynamically
        self.allow_kill = self.get_parameter('allow_kill').get_parameter_value().bool_value
        if not self.allow_kill:
            msg = "Killing disabled. Set parameter 'allow_kill' to true to enable."
            self.get_logger().warn(msg)
            response.success = False
            response.message = msg
            return response

        node_name = request.node_name
        if not node_name:
            msg = "Empty node name in request."
            self.get_logger().warn(msg)
            response.success = False
            response.message = msg
            return response

        # Normalize node name by stripping leading/trailing slashes
        clean_node_name = node_name.strip('/')
        self.get_logger().info(f"Received request to kill node: {node_name} (clean: {clean_node_name})")

        # Find matching PIDs
        pids = self.find_node_pids(clean_node_name)
        if not pids:
            # Try to see if we can find processes containing the node name as a substring/executable
            # (e.g. if the node is launched via an executable named clean_node_name)
            pids = self.find_pids_by_executable_name(clean_node_name)

        if not pids:
            msg = f"Could not determine local PID for node: {node_name}"
            self.get_logger().warn(msg)
            response.success = False
            response.message = msg
            return response

        # Filter out our own PID
        my_pid = os.getpid()
        pids = [p for p in pids if p != my_pid]

        if not pids:
            msg = f"No external processes found matching node: {node_name} (only self-process found)"
            self.get_logger().warn(msg)
            response.success = False
            response.message = msg
            return response

        # Send SIGINT to all matched PIDs
        success_count = 0
        errors = []
        for pid in pids:
            try:
                self.get_logger().info(f"Sending SIGINT to PID {pid} (node {node_name})")
                os.kill(pid, signal.SIGINT)
                success_count += 1
            except OSError as e:
                err_msg = f"Failed to send SIGINT to PID {pid}: {e}"
                self.get_logger().error(err_msg)
                errors.append(err_msg)

        if success_count > 0:
            msg = f"Sent SIGINT to {success_count} process(es) matching node {node_name}."
            if errors:
                msg += f" Failures: {'; '.join(errors)}"
            self.get_logger().info(msg)
            response.success = True
            response.message = msg
        else:
            msg = f"Failed to send SIGINT to any processes: {'; '.join(errors)}"
            self.get_logger().error(msg)
            response.success = False
            response.message = msg

        return response

    def find_node_pids(self, node_name):
        """
        Scan /proc to find processes that contain the ROS 2 node remapping argument
        __node:=node_name or __node:=/node_name
        """
        pids = []
        # Remapping targets to search for in command lines
        targets = [f"__node:={node_name}", f"__node:=/{node_name}"]
        try:
            for pid_dir in os.listdir('/proc'):
                if pid_dir.isdigit():
                    pid = int(pid_dir)
                    try:
                        with open(os.path.join('/proc', pid_dir, 'cmdline'), 'r') as f:
                            cmdline = f.read()
                            # cmdline is null-terminated strings
                            args = cmdline.split('\x00')
                            for arg in args:
                                if any(t == arg or arg.endswith(f"/{t}") for t in targets):
                                    pids.append(pid)
                                    break
                    except (IOError, OSError, ValueError):
                        continue
        except Exception as e:
            self.get_logger().error(f"Error scanning /proc: {e}")
        return pids

    def find_pids_by_executable_name(self, name):
        """
        Fallback scanner: check if executable name matches or contains 'name'
        """
        pids = []
        try:
            for pid_dir in os.listdir('/proc'):
                if pid_dir.isdigit():
                    pid = int(pid_dir)
                    try:
                        # Check /proc/<pid>/comm or command line executable path
                        with open(os.path.join('/proc', pid_dir, 'comm'), 'r') as f:
                            comm = f.read().strip()
                            if comm == name:
                                pids.append(pid)
                                continue
                        with open(os.path.join('/proc', pid_dir, 'cmdline'), 'r') as f:
                            cmdline = f.read()
                            args = cmdline.split('\x00')
                            if args and args[0]:
                                exe_name = os.path.basename(args[0])
                                if exe_name == name or exe_name == f"{name}_node":
                                    pids.append(pid)
                    except (IOError, OSError):
                        continue
        except Exception as e:
            self.get_logger().error(f"Error scanning /proc for executable name: {e}")
        return pids

def main(args=None):
    rclpy.init(args=args)
    node = KillNodeSigintService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
