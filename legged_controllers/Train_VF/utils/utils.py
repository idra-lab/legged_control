import torch
import yaml
import rospy
from gazebo_msgs.srv import ApplyBodyWrench
import rospkg
import roslaunch
import numpy as np

def load_config(file_path):
    """ Function to load YAML configuration """
    with open(file_path, 'r') as file:
        return yaml.safe_load(file)

def swap_legs(array):
    """
    Swap the front and rear legs of the array based on predefined indices.
    
    The swap logic is fixed:
    """
    array_copy = array.copy()  # Make a copy to avoid modifying the original array
    
    #array_copy[0:3] = array[6:9]
    #array_copy[3:6] = array[9:12]
    #array_copy[6:9] = array[0:3]
    #array_copy[9:12] = array[3:6] 

    #[FL, RL, FR, RR] to [FL, FR, RL, RR]
    
    array_copy[0:3] = array[0:3]
    array_copy[3:6] = array[6:9]
    array_copy[6:9] = array[3:6]
    array_copy[9:12] = array[9:12] 
    return array_copy

# Quaternion rotation helper
def quat_rotate_inverse(q, v):
    shape = q.shape
    q_w = q[:, -1]
    q_vec = q[:, :3]
    a = v * (2.0 * q_w ** 2 - 1.0).unsqueeze(-1)
    b = torch.cross(q_vec, v, dim=-1) * q_w.unsqueeze(-1) * 2.0
    c = q_vec * torch.bmm(q_vec.view(shape[0], 1, 3), v.view(shape[0], 3, 1)).squeeze(-1) * 2.0
    return a - b + c

def applyForce(Fx, Fy, Fz, Mx, My, Mz, duration, link_name="base"):
    from geometry_msgs.msg import Wrench, Point

    wrench = Wrench()
    wrench.force.x = Fx
    wrench.force.y = Fy
    wrench.force.z = Fz
    wrench.torque.x = Mx
    wrench.torque.y = My
    wrench.torque.z = Mz
    reference_frame = "world"  # you can apply forces only in this frame because this service is buggy, it will ignore any other frame
    reference_point = Point(x=0, y=0, z=0)
    apply_body_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
    try:
        apply_body_wrench(body_name="aliengo::"+link_name, reference_frame=reference_frame,
                                reference_point=reference_point, wrench=wrench, duration=rospy.Duration(duration))
    except:
        pass


def launchFileNode(package,launch_file, additional_args=None):
    launch_file = rospkg.RosPack().get_path(package) + '/launch/'+launch_file
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    cli_args = [launch_file]
    if additional_args is not None:
        cli_args.extend(additional_args)
    roslaunch_args = cli_args[1:]
    roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
    return roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)

def quaternion_to_euler_deg(quat):
    x = quat[0]
    y = quat[1] 
    z = quat[2] 
    w = quat[3]

    # Roll (x-axis rotation)
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2)) * (180 / np.pi)

    # Pitch (y-axis rotation)
    pitch = np.arcsin(2 * (w * y - z * x)) * (180 / np.pi)

    # Yaw (z-axis rotation)
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2)) * (180 / np.pi)

    return roll, pitch, yaw