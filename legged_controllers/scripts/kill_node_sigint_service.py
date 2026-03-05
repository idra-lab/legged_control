#!/usr/bin/env python3
"""
kill_node_sigint_service.py (robust XML-RPC PID lookup)

Service: /kill_node_sigint
Service type: legged_controllers/KillNode

Behavior:
 - Uses ROS Master XML-RPC to lookup node URI (master.lookupNode).
 - Calls the node's Slave XML-RPC API method getPid(caller_id) to obtain PID.
 - If PID is on local host, sends SIGINT to the PID.
 - Requires private param ~allow_kill == true to actually send the signal.
 - Falls back to parsing 'rosnode info' only if XML-RPC calls fail.
"""

from __future__ import print_function
import os
import signal
import socket
import errno
import rospy
import subprocess
import shlex

# Python2/3 xmlrpc client compatibility
try:
    from xmlrpc.client import ServerProxy, ProtocolError, Fault
except Exception:
    from xmlrpclib import ServerProxy, ProtocolError, Fault

from legged_controllers.srv import KillNode, KillNodeResponse

ALLOW_KILL_PARAM = "allow_kill"

def get_master_proxy():
    """Return an xmlrpc ServerProxy to the ROS master."""
    master_uri = os.environ.get('ROS_MASTER_URI')
    if not master_uri:
        raise RuntimeError("ROS_MASTER_URI not set in environment")
    return ServerProxy(master_uri)

def lookup_node_uri(node_name):
    """
    Use master.lookupNode(caller_id, node_name) to get node URI.
    Returns (success_bool, node_uri_or_errmsg)
    """
    try:
        master = get_master_proxy()
        caller_id = rospy.get_name()
        code, msg, val = master.lookupNode(caller_id, node_name)
        if code != 1:
            return False, "master.lookupNode failed: code={} msg={}".format(code, msg)
        return True, val
    except Exception as e:
        return False, "lookupNode exception: {}".format(e)

def get_pid_via_xmlrpc(node_uri):
    """
    Call the node's Slave API getPid(caller_id) to retrieve PID.
    Returns (success_bool, pid_or_errmsg)
    """
    try:
        node_api = ServerProxy(node_uri)
        caller_id = rospy.get_name()
        # rosnode's code calls node.getPid(ID)
        code, msg, val = node_api.getPid(caller_id)
        if code != 1:
            return False, "node.getPid returned code={} msg={}".format(code, msg)
        # val should be an integer pid
        try:
            pid = int(val)
            return True, pid
        except Exception:
            return False, "node.getPid returned non-integer pid: {}".format(val)
    except ProtocolError as e:
        return False, "ProtocolError contacting node API {}: {}".format(node_uri, e)
    except socket.error as e:
        return False, "socket error contacting node API {}: {}".format(node_uri, e)
    except Fault as e:
        return False, "XML-RPC fault from {}: {}".format(node_uri, e)
    except Exception as e:
        return False, "exception contacting node API {}: {}".format(node_uri, e)

def run_cmd_capture_stdout(cmd):
    """Run shell command and return (success_bool, stdout_str)."""
    try:
        out = subprocess.check_output(shlex.split(cmd), stderr=subprocess.STDOUT)
        return True, out.decode('utf-8', errors='replace')
    except subprocess.CalledProcessError as e:
        return False, e.output.decode('utf-8', errors='replace')
    except OSError as e:
        return False, "OS error running '{}': {}".format(cmd, e)

import re
def parse_rosnode_info_fallback(info_text):
    """Fallback parser (regex) to extract Pid and Node URI from 'rosnode info' output."""
    # Look for "Pid: <number>"
    pid_match = re.search(r'^\s*Pid:\s*([0-9]+)\s*$', info_text, re.MULTILINE)
    node_uri_match = re.search(r'^\s*Node URI:\s*(\S+)\s*$', info_text, re.MULTILINE)
    pid = -1
    node_uri = ''
    if pid_match:
        try:
            pid = int(pid_match.group(1))
        except Exception:
            pid = -1
    if node_uri_match:
        node_uri = node_uri_match.group(1)
    return pid, node_uri

def parse_host_from_uri(uri):
    """Extract host from http://host:port/..."""
    try:
        if "://" in uri:
            rest = uri.split("://", 1)[1]
            host_port = rest.split("/", 1)[0]
            host = host_port.split(":", 1)[0]
            return host
    except Exception:
        pass
    return ""

def get_local_addresses():
    """Return set of local IP addresses for heuristic local-check."""
    addrs = set()
    try:
        local_name = socket.gethostname()
        try:
            _, _, host_addrs = socket.gethostbyname_ex(local_name)
            for a in host_addrs:
                addrs.add(a)
        except Exception:
            pass
        addrs.add("127.0.0.1")
        addrs.add("::1")
        try:
            _, _, localhost_addrs = socket.gethostbyname_ex("localhost")
            for a in localhost_addrs:
                addrs.add(a)
        except Exception:
            pass
    except Exception:
        pass
    return addrs

def host_is_local(host):
    """Heuristic: True if host is localhost/127.0.0.1 or resolves to a local address."""
    if not host:
        return False
    host_lower = host.lower()
    if host_lower in ("localhost", "127.0.0.1", "0.0.0.0", "::1"):
        return True
    local_addrs = get_local_addresses()
    try:
        _, _, host_addrs = socket.gethostbyname_ex(host)
        for a in host_addrs:
            if a in local_addrs:
                return True
    except Exception:
        try:
            if host == socket.gethostname():
                return True
        except Exception:
            pass
    return False

def handle_kill_request(req):
    pnh = rospy.get_param
    allow_kill = rospy.get_param("~" + ALLOW_KILL_PARAM, True)
    if not allow_kill:
        msg = "Killing disabled. Set private param '~{}' to true to enable.".format(ALLOW_KILL_PARAM)
        rospy.logwarn(msg)
        return KillNodeResponse(success=False, message=msg)

    node = req.node_name or ""
    if node == "":
        msg = "Empty node name in request."
        rospy.logwarn(msg)
        return KillNodeResponse(success=False, message=msg)
    if not node.startswith("/"):
        node = "/" + node

    # 1) Try XML-RPC master.lookupNode -> node.getPid(caller_id)
    ok, node_uri_or_err = lookup_node_uri(node)
    if not ok:
        rospy.logwarn("lookup_node_uri failed: %s -- falling back to 'rosnode info' parsing", node_uri_or_err)
    else:
        node_uri = node_uri_or_err
        pid_ok, pid_or_err = get_pid_via_xmlrpc(node_uri)
        if pid_ok:
            pid = pid_or_err
            rospy.loginfo("Got PID %s for node %s via XML-RPC (URI=%s)", pid, node, node_uri)
        else:
            rospy.logwarn("get_pid_via_xmlrpc failed: %s -- will try rosnode info fallback", pid_or_err)
            pid = -1

    # 2) Fallback: parse rosnode info if we didn't get a pid
    if 'pid' not in locals() or pid <= 0:
        # run rosnode info and parse with regex
        cmd = "rosnode info '{}'".format(node)
        ok_cmd, out = run_cmd_capture_stdout(cmd)
        if not ok_cmd:
            msg = "Failed to run '{}': {}".format(cmd, out)
            rospy.logerr(msg)
            return KillNodeResponse(success=False, message=msg)
        pid, node_uri = parse_rosnode_info_fallback(out)
        rospy.loginfo("Fallback parsed pid=%s node_uri=%s", pid, node_uri)

    if not node_uri:
        msg = "Could not determine Node URI for {}.".format(node)
        rospy.logwarn(msg)
        return KillNodeResponse(success=False, message=msg)

    if pid <= 0:
        msg = "Could not determine PID for {} (node URI {}).".format(node, node_uri)
        rospy.logwarn(msg)
        return KillNodeResponse(success=False, message=msg)

    host = parse_host_from_uri(node_uri)
    if not host_is_local(host):
        msg = ("Node appears to run on remote host '{}'. PID={}. This service only signals local processes. "
               "SSH to the host or use XML-RPC shutdown. Node URI: {}").format(host, pid, node_uri)
        rospy.logwarn(msg)
        return KillNodeResponse(success=False, message=msg)

    # Send SIGINT
    try:
        os.kill(int(pid), signal.SIGINT)
    except OSError as e:
        msg = "Failed to send SIGINT to PID {}: errno={} ({})".format(pid, e.errno, e.strerror)
        rospy.logerr(msg)
        return KillNodeResponse(success=False, message=msg)

    msg = "Sent SIGINT to PID {} (node {})".format(pid, node)
    rospy.loginfo(msg)
    return KillNodeResponse(success=True, message=msg)

def main():
    rospy.init_node('kill_node_sigint_service')
    rospy.loginfo("Starting kill_node_sigint_service")
    allow_kill = rospy.get_param("~" + ALLOW_KILL_PARAM, False)
    if not allow_kill:
        rospy.logwarn("KILLING IS DISABLED by default. Start with _allow_kill:=true to enable.")
    rospy.Service('/kill_node_sigint', KillNode, handle_kill_request)
    rospy.spin()

if __name__ == "__main__":
    main()
