
import numpy as np
import time
import os
from tqdm import tqdm
from termcolor import colored
import rospy

import copy
import torch
from gazebo_msgs.srv import *
from gazebo_msgs.msg import ModelState
from controller_manager_msgs.srv import SwitchController, LoadController, UnloadController
from std_srvs.srv import Empty
import roslaunch
import rospkg
import sys

full_path = os.path.realpath(__file__)
sys.path.append(os.path.dirname(full_path) + '/../Train_VF/utils')

import publish_subscribe as publish_subscribe
from rl_controller import RlVelocityController
from utils import *
from value_function_manager import ValueFunctionManager



class TestManager():
    def __init__(self, use_nn=False):
        # -------------------------------
        # Simulation Thresholds and Constants
        # -------------------------------
        self.INCLINATION_THRESHOLD = 30.0  # degrees - max allowed inclination before considering robot as fallen
        self.FALL_HEIGHT_THRESHOLD = 0.2   # meters - min allowed height before considering robot as fallen
        self.CP_SAFE_RADIUS = 0.05         # meters - acceptable radius to consider CP successful
        self.G = 9.81                      # gravitational constant
        self.policy_frequency = 50         # Hz
        self.vf_frequency = 100        # Hz
        self.dt = 0.002
        self.decimation = (1 / self.dt) * (1 / self.policy_frequency)
        self.decimation_vf = (1 / self.dt) * (1 / self.vf_frequency)
        self.grav_tens = torch.tensor([[0., 0., -1.]], device='cuda:0', dtype=torch.double)

        self.sim_time = 0

        # Values to reset robot state
        self.initial_pose = np.array([0, 0, 0.45, 0, 0, 0, 1])
        self.joint_names = ['LF_HAA', 'LF_HFE', 'LF_KFE', 
                            'LH_HAA', 'LH_HFE', 'LH_KFE', 
                            'RF_HAA', 'RF_HFE', 'RF_KFE', 
                            'RH_HAA', 'RH_HFE', 'RH_KFE']
        self.joint_positions = [-0.1, 0.62, -1.24,
                                -0.1, 0.62, -1.24,
                                 0.1, 0.62, -1.24,
                                 0.1, 0.62, -1.24]
            
        self.use_nn = use_nn
        self.init_ros()

        full_path = os.path.realpath(__file__)
        config_path = os.path.dirname(full_path) + '/../Train_VF/utils/config.yaml'
        self.config = load_config(config_path)

        self.force_time = self.config['settings']['tests']['force_time']
        self.use_backup = self.config['settings']['tests']['use_backup']
        self.vf_additional_term = self.config['settings']['tests']['additional_term_vf']
        self.threshold = self.config['settings']['tests']['threshold_vf']
        self.Fx = 0
        self.Fy = 0
        self.Fz = 0
        self.iter_push = 0
        self.last_i = 0
        
        
        # Backup policy
        self.backup_policy = RlVelocityController('aliengo', self.dt, use_nn_se=True)
        self.kp_backup = self.backup_policy.kp[0]
        self.kd_backup = self.backup_policy.kd[0]
        self.backup_policy.velocity_cmd = np.zeros(3)
        self.ffw_torques = np.zeros(12)

        self.vf = ValueFunctionManager(use_nn=True, stop=False)

    def init_ros(self):
        # ROS
        # Launch nodes
        self.launch_world = launchFileNode('legged_unitree_description','empty_world.launch')
        self.launch_world.start()
        time.sleep(1)

        if self.use_nn:
            nn_arg = 'nn:=true'
        else:
            nn_arg = 'nn:=false'
        self.launch_controller = launchFileNode('legged_controllers', 'load_controller.launch', additional_args=['joy:=true', nn_arg, 'mps:=true'])
        self.launch_controller.start()

        # Subscribe to messages
        rospy.init_node('communicate_aliengo')
        
        self.pubSub = publish_subscribe.PubSub()
        self.pubSub.init_publishers()
        self.pubSub.init_subscribers()
        self.rate_ros = rospy.Rate(1/self.dt)  # 500 Hz for dt = 0.002

    def deregister_node(self):
        self.launch_world.shutdown()
        self.launch_controller.shutdown()
   
    def call_service(self, ns, cls, **kwargs):
        rospy.wait_for_service(ns)
        service = rospy.ServiceProxy(ns, cls)
        response = service(**kwargs)

    def reset(self):
        reset_iter = 1
        while(np.linalg.norm(self.pubSub.pose - self.initial_pose) > 0.05*2 or np.linalg.norm(self.pubSub.twist) > 0.05*2 or
              np.linalg.norm(self.pubSub.joint_pos - self.joint_positions) > 0.05*2):
            if reset_iter > 10:
                self.deregister_node()
                self.init_ros()
                reset_iter = 1
            rospy.wait_for_service('/controller_manager/switch_controller')
            self.call_service("/controller_manager/switch_controller", SwitchController,
            start_controllers=[""],
            stop_controllers=["controllers/mps_controller",'controllers/joint_state_controller', 'controllers/imu_sensor_controller'],
            strictness=1, start_asap=False, timeout=0.0)
            
            rospy.wait_for_service('/controller_manager/unload_controller')
            unload_srv = rospy.ServiceProxy('/controller_manager/unload_controller', UnloadController)
            
            # Chiamare il servizio
            resp = unload_srv("controllers/mps_controller")


            rospy.wait_for_service('/gazebo/set_model_configuration')
            rospy.wait_for_service('/gazebo/unpause_physics')
            
            pause_physics_client = rospy.ServiceProxy('/gazebo/pause_physics',Empty)  
            unpause_physics_client = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
            
            pause_physics_client()
            rospy.wait_for_service('/gazebo/set_model_state')
            reset_world = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            model_state = ModelState()
            model_state.model_name = 'aliengo'
            model_state.pose.position.x = self.initial_pose[0]
            model_state.pose.position.y = self.initial_pose[1]
            model_state.pose.position.z = self.initial_pose[2]

            model_state.pose.orientation.x = self.initial_pose[3]
            model_state.pose.orientation.y = self.initial_pose[4]
            model_state.pose.orientation.z = self.initial_pose[5]
            model_state.pose.orientation.w = self.initial_pose[6]

            model_state.twist.linear.x = 0
            model_state.twist.linear.y = 0
            model_state.twist.linear.z = 0

            model_state.twist.angular.x = 0
            model_state.twist.angular.y = 0
            model_state.twist.angular.z = 0
            
            req = SetModelStateRequest()
            req.model_state = model_state
            reset_world(req)

            rospy.wait_for_service('/gazebo/set_model_configuration')
            time.sleep(1)
            
            set_model_configuration = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)

            req_config = SetModelConfigurationRequest()
            req_config.model_name ='aliengo'
            req_config.urdf_param_name = 'robot_description'
            req_config.joint_names = self.joint_names
            req_config.joint_positions = self.joint_positions

            resp = set_model_configuration(req_config)

            time.sleep(1)
            
            rospy.wait_for_service('/controller_manager/switch_controller')
            rospy.wait_for_service('/controller_manager/load_controller')
            rospy.wait_for_service('/controller_manager/switch_controller')
            rospy.wait_for_service('/gazebo/apply_body_wrench')
            unpause_physics_client()
            load_srv = rospy.ServiceProxy('/controller_manager/load_controller', LoadController)
            
            # Chiamare il servizio
            resp = load_srv("controllers/mps_controller")
            self.call_service("/controller_manager/switch_controller", SwitchController,
                start_controllers=["controllers/mps_controller",'controllers/joint_state_controller', 'controllers/imu_sensor_controller'],
                stop_controllers=[""],
                strictness=1, start_asap=False, timeout=0.0)
            
        
            self.sim_time = 0
            self.pubSub.publish_button([2])
            time.sleep(2)
            reset_iter += 1

    # -------------------------------
    # Capture Point Computation
    # -------------------------------
    def compute_capture_point(self, pos, vel, height):
        tc = np.sqrt(height / self.G)  # time constant based on height
        return pos + vel * tc

    # -------------------------------
    # Fall Condition Checker
    # -------------------------------
    def check_fallen(self, base_z_position, inclination_deg):
        return inclination_deg > self.INCLINATION_THRESHOLD or base_z_position < self.FALL_HEIGHT_THRESHOLD

    def check_fall_cp(self, data_new):
        # -------------------------------
        # Check Falling
        # -------------------------------
        inclination_after = 2 * np.arcsin(np.sqrt(data_new[4][0] ** 2 + data_new[4][1] ** 2)) * (180 / np.pi)
        
        if  self.check_fallen(data_new[0][2], inclination_after):
            self.fallen_flag = 1
            self.capture_flag = 0

        # -------------------------------
        # Check com inside safe radius of capture point
        # -------------------------------
        base_pos_xy = copy.copy(data_new[0][:2])
        base_vel_xy = copy.copy(data_new[1][:2])
        z_after = copy.copy(data_new[0][2])
        cp = self.compute_capture_point(base_pos_xy, base_vel_xy, z_after)
        cp_local = cp - base_pos_xy

        #when self.capture_flag = 1 it means we are stable
        if np.linalg.norm(cp_local) < self.CP_SAFE_RADIUS and self.fallen_flag == 0:
            self.capture_flag = 1
        


    # -------------------------------
    # Main Function: Single Simulation Episode
    # -------------------------------
    def run_single_simulation(self, max_steps=2000, warmup_time=1.0):
        #init vars
        self.fallen_flag = 0
        self.capture_flag = 0
        self.last_i = 0
        self.first_time = True
        self.isrec = True
        self.backup_used = False
        decimation_counter_vf = 0

        self.warmup_time = warmup_time
        velocity_cmd = np.array([0.1, 0, 0])
        #debug
        #generate push instant
        self.warmup_steps = int(warmup_time / self.dt)
        push_instant = self.warmup_steps + (self.iter_push*5)

        
        self.pubSub.publish_button([3]) # Trot
        time.sleep(1)
        for self.step in range(max_steps):
            # Update messages
            data_new = [self.pubSub.pose, self.pubSub.twist, self.pubSub.joint_pos, self.pubSub.joint_vel, self.pubSub.imu_quat, self.pubSub.imu_ang_vel, self.pubSub.imu_lin_acc]

            self.check_fall_cp(data_new)
            if self.capture_flag:
                self.last_i = int(self.step/5) + 1
             
            # Apply external push
            if self.step == push_instant:
                applyForce(self.Fx, self.Fy, self.Fz, 0, 0, 0, self.force_time)

            if self.isrec:
                cmd_vel = np.array([velocity_cmd[0], velocity_cmd[1], 0, 0, 0, 0, velocity_cmd[2]])
                self.pubSub.publish_vel(cmd_vel)
                self.pubSub.publish_backup(np.zeros(12),np.zeros(12),np.zeros(12))

                if self.use_nn:

                    
                    body_ang_vel = copy.copy(data_new[5])
                    proj_gravity = quat_rotate_inverse(
                        torch.tensor(data_new[4], device='cuda:0', dtype=torch.double).unsqueeze(0),
                        #torch.tensor(data_new[0][3:], device='cuda:0', dtype=torch.double).unsqueeze(0),
                        self.grav_tens
                    )[0].cpu().numpy()
                    if self.use_backup and (self.step*self.dt > 0.5) and (decimation_counter_vf % self.decimation_vf) == 0:
                        self.isrec, V_safe = self.vf.computeValueFnc(body_ang_vel, proj_gravity, joint_pos=data_new[2], joint_vel=data_new[3], threshold=self.threshold, vf_additional_term = self.vf_additional_term)

                    qDes_no = self.backup_policy.action(data_new[6], None, body_ang_vel, proj_gravity, data_new[2], data_new[3], policy_type="safe")
    
            else:
                self.backup_used = True
                body_ang_vel = copy.copy(data_new[5])
                proj_gravity = quat_rotate_inverse(
                    torch.tensor(data_new[4], device='cuda:0', dtype=torch.double).unsqueeze(0),
                    #torch.tensor(data_new[0][3:], device='cuda:0', dtype=torch.double).unsqueeze(0),
                    self.grav_tens
                )[0].cpu().numpy()
                qDes = self.backup_policy.action(data_new[6], None, body_ang_vel, proj_gravity, data_new[2], data_new[3], policy_type="safe")
                self.pubSub.publish_is_rec(False)
                
                self.pubSub.publish_backup(qDes,np.zeros(12),self.ffw_torques)
            #self.pubSub.publish_button(2) # Stance

            self.rate_ros.sleep()
            self.sim_time = np.round(self.sim_time + self.dt, 4)  # np.array([self.loop_time]), 3)

        return data_new

    # -------------------------------
    # Run a Batch of Simulations and Save Results
    # -------------------------------
    def run_simulations(self):
        full_path = os.path.realpath(__file__)
        folder_results = os.path.dirname(full_path) + self.config['settings']['paths']['results_folder'] + '/'

        test_num_last = self.config['settings']['tests']['last_test']
        dir_path = os.path.dirname(full_path) + self.config['settings']['paths']['force_directions']
        iter_path = os.path.dirname(full_path) + self.config['settings']['paths']['force_iterations']
        force_mag = np.array(self.config['settings']['tests']['force_mag'])

        max_steps = int(self.config['settings']['tests']['test_time']/self.dt)
        
        
        
        self.reset()
        #for i in tqdm(range(n_episodes)):
        # Each test lasts 20 seconds 
        for i in force_mag:
            test_force = str(i)
            test_num = 1   

            dir_file = open(dir_path)
            dir_text = dir_file.readline().rstrip().split(',')
            while dir_text[0] != '':

                data_save = {        'data_sim': [], 'save_fall': [], 
                          'save_backup': [], 'save_stop': [], 
                     'save_stop_backup': []}
                # Change force 
                
                j = np.array([float(dir_text[0]),float(dir_text[1]),float(dir_text[2])])
                applied_force = j * i
                self.Fx = applied_force[0]
                self.Fy = applied_force[1]
                self.Fz = applied_force[2]

                iter_file = open(iter_path)
                iter_text = iter_file.readline().rstrip().split(',')
                while iter_text[0] != '':
                    self.iter_push = int(iter_text[0])
                    if  test_num > test_num_last:
                        data_final = self.run_single_simulation(max_steps=max_steps, warmup_time=4.0)
                        # -------------------------------
                        # Reset robot in Gazebo
                        # -------------------------------
                        self.reset()
                        
                        data_save['data_sim'].append([iter_text,           # 0
                                                      i,                   # 1
                                                      j,                   # 2
                                                      test_num,            # 3
                                                      data_final[0],       # 4
                                                      data_final[1],       # 5
                                                      data_final[2],       # 6
                                                      data_final[3],       # 7
                                                      data_final[4],       # 8
                                                      data_final[5],       # 9
                                                      data_final[6],       # 10
                                                      self.last_i])        # 11
                        
                        
                        if self.fallen_flag:
                            data_save['save_fall'].append([i, test_num])

                        if not self.backup_used:
                            data_save['save_backup'].append([i, test_num])

                        if not self.capture_flag:
                            data_save['save_stop'].append([i, test_num])
                        
                        if not self.backup_used and not self.capture_flag:
                            data_save['save_stop_backup'].append([i, test_num])
                        
                        if self.use_nn:
                            self.backup_policy.prev_action = np.zeros(12)
                            self.backup_policy.decimation_counter = 0
                            self.backup_policy.history_buffer = np.zeros((1, 3, 48))
                            self.pubSub.publish_is_rec(True)
                            self.pubSub.publish_backup(np.zeros(12),np.zeros(12),np.zeros(12))
                        time.sleep(2)
                    
                    test_num += 1
                    iter_text = iter_file.readline().rstrip().split(',')
                iter_file.close()

                # Save data
                path_save = folder_results + test_force
                save_data_tests(path_save, test_force, data_save)

                dir_text = dir_file.readline().rstrip().split(',')
            dir_file.close()
            test_num_last = 0


        self.deregister_node()

        