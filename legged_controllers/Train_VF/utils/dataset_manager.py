import numpy as np
import time
import os
from tqdm import tqdm
from termcolor import colored
import rospy
import utils.publish_subscribe as publish_subscribe
import copy
import torch
from gazebo_msgs.srv import *
from gazebo_msgs.msg import ModelState
from controller_manager_msgs.srv import SwitchController, ReloadControllerLibraries, LoadController, UnloadController
from std_srvs.srv import Empty
import roslaunch
import rospkg
from utils.backup import BackupPolicy
from utils.rl_controller import RlVelocityController
from utils.utils import *
from controller_manager import controller_manager_interface



class DatasetManager():
    def __init__(self, use_nn=False, backup_trot=True):
        # -------------------------------
        # Simulation Thresholds and Constants
        # -------------------------------
        self.INCLINATION_THRESHOLD = 30.0  # degrees - max allowed inclination before considering robot as fallen
        self.FALL_HEIGHT_THRESHOLD = 0.2   # meters - min allowed height before considering robot as fallen
        self.CP_SAFE_RADIUS = 0.05         # meters - acceptable radius to consider CP successful
        self.G = 9.81                      # gravitational constant
        self.policy_frequency = 50 #Hz
        self.dt = 0.002
        self.decimation = (1 / self.dt) * (1 / self.policy_frequency)
        self.grav_tens = torch.tensor([[0., 0., -1.]], device='cuda:0', dtype=torch.double)

        self.sim_time = 0
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
        self.backup_trot = backup_trot
        self.init_ros()

        full_path = os.path.realpath(__file__)
        config_path = os.path.dirname(full_path) + '/config.yaml'
        self.config = load_config(config_path)
        if self.backup_trot:
            self.backup_policy = BackupPolicy(self.config)
            self.running_mean_backup = copy.copy(self.backup_policy.actor_network.running_mean_std.running_mean)
            self.running_var_backup = copy.copy(self.backup_policy.actor_network.running_mean_std.running_var)
            self.count_backup = copy.copy(self.backup_policy.actor_network.running_mean_std.count)
            self.backup_policy.decimation = self.decimation
            self.kp_backup = np.array(self.config['robot']['kp'])
            self.kd_backup = np.array(self.config['robot']['kd'])
            self.backup_policy.commands = np.array(self.config['robot']['cmd_backup'])

            self.ffw_torques = np.array([1.6, 0.0, 0.0,      # LF 
                                    1.6, 0.0, 0.0,     # LH 
                                    -1.6, 0.0, 0.0,      # RF
                                    -1.6, 0.0, 0.0])*1  # RH
            
        else:
            self.backup_policy = RlVelocityController('aliengo', self.dt, use_nn_se=True)
            self.kp_backup = self.backup_policy.kp[0]
            self.kd_backup = self.backup_policy.kd[0]
            self.backup_policy.velocity_cmd = np.zeros(3)
            self.ffw_torques = np.zeros(12)

    def init_ros(self):
        # ROS
        

        self.launch_world = launchFileNode('legged_unitree_description','empty_world.launch')
        self.launch_world.start()
        time.sleep(1)

        if self.use_nn:
            nn_arg = 'nn:=true'
        else:
            nn_arg = 'nn:=false'
        self.launch_controller = launchFileNode('legged_controllers', 'load_controller.launch', additional_args=['joy:=true', nn_arg, 'mps:=true'])
        self.launch_controller.start()

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
        #time.sleep(2)
        reset_iter = 1
        '''data_new = [self.pubSub.pose, self.pubSub.twist, self.pubSub.joint_pos, self.pubSub.joint_vel, self.pubSub.imu_quat, self.pubSub.imu_ang_vel, self.pubSub.imu_lin_acc]
            
        print('resquat', data_new[4])
        print('resxyz', data_new[0][:3])'''
        while(np.linalg.norm(self.pubSub.pose - self.initial_pose) > 0.05*2 or np.linalg.norm(self.pubSub.twist) > 0.05*2 or
              np.linalg.norm(self.pubSub.joint_pos - self.joint_positions) > 0.05*2):
            
            print(reset_iter)
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

    def check_termination(self, data_new):
        # -------------------------------
        # Check Falling
        # -------------------------------
        inclination_after = 2 * np.arcsin(np.sqrt(data_new[4][0] ** 2 + data_new[4][1] ** 2)) * (180 / np.pi)
        
        #roll, pitch, yaw = quaternion_to_euler_deg(data_new[4])
        #inclination_after = max(roll, pitch)
        if  self.check_fallen(data_new[0][2], inclination_after):
            self.fallen_flag = 1
            '''print('fall')
            print('inclination',inclination_after)
            print('quat', data_new[4])
            print('xyz', data_new[0][:3])'''

        # -------------------------------
        # Check com inside safe radius of capture point
        # -------------------------------
        base_pos_xy = copy.copy(data_new[0][:2])
        base_vel_xy = copy.copy(data_new[1][:2])
        z_after = copy.copy(data_new[0][2])
        cp = self.compute_capture_point(base_pos_xy, base_vel_xy, z_after)
        cp_local = cp - base_pos_xy

        #when self.capture_flag = 1 it means we are stable
        if np.linalg.norm(cp_local) < self.CP_SAFE_RADIUS and self.step >= self.warmup_steps and self.fallen_flag == 0:
            self.capture_flag = 1
            '''print('capture')
            print('inclination',inclination_after)
            print('quat', data_new[4])
            print('xyz', data_new[0][:3])'''

        return self.capture_flag #self.fallen_flag#or 

    def store_observations(self, data_new):
        # -------------------------------
        # Save Observation Transition for VF Learning
        # -------------------------------
        if self.step % self.decimation == 0 :
            body_ang_vel = copy.copy(data_new[5])
            proj_gravity = quat_rotate_inverse(
                torch.tensor(data_new[4], device='cuda:0', dtype=torch.double).unsqueeze(0),
                self.grav_tens
            )[0].cpu().numpy()
            # dimension of observation vector = 3+3+12+12=30
            self.obs_tp1_ = np.concatenate((
                proj_gravity.astype(np.float32),
                body_ang_vel.astype(np.float32),
                data_new[2].astype(np.float32),
                data_new[3].astype(np.float32)
            ))
            # dimension of full_obs vector = 30+30+1+1=62
            full_obs = np.concatenate([self.obs_t_, self.obs_tp1_, [float(self.fallen_flag), float(self.capture_flag)]])
            #print('store',self.capture_flag)
            # collect data only for backup policy which is active after warmup (the = ensures at least one sample is collected before break)
            if self.step>=self.warmup_steps:
                #print(full_obs)
                self.observations.append(full_obs)

        # store last observation
            self.obs_t_ = self.obs_tp1_


    # -------------------------------
    # Main Function: Single Simulation Episode
    # -------------------------------
    def run_single_simulation(self, max_steps=2000, noise_std=1.0, warmup_time=1.0):
        #init vars
        self.fallen_flag = 0
        self.capture_flag = 0
        self.first_time = True
        self.observations = []
        self.obs_t_ = []
        self.obs_tp1_ = []

        #reset robot
        self.warmup_time = warmup_time
        random_cmd = np.array([ 
            np.random.uniform(-0.1, 0.1),  # (-0.5, 1.0),#vx
            np.random.uniform(-0.1, 0.1),  # vy
            np.random.uniform(-0.1, 0.1) #(-0.4, 0.4)  # yaw_rate
        ])
        #debug
        #actor_network.velocity_cmd = np.array([0.5, 0.0, 0.0])
        print(f"Nominal policy velocity command {random_cmd}")
        #generate push instant
        low = warmup_time - 0.5
        high = warmup_time
        n_steps = int((high - low) / self.dt)
        self.warmup_steps = int(warmup_time / self.dt)
        push_instant = self.warmup_steps - 5#round(low + np.random.randint(0, n_steps + 1) * self.dt,3)

        #print('initialB', self.quadruped.baseTwistW, self.quadruped.basePoseW, self.quadruped.q)
        #while np.linalg.norm(self.quadruped.q - self.quadruped.qj_0.copy()) > 0.001:
        #    self.quadruped.updateKinematics()

        #while np.linalg.norm(self.quadruped.baseTwistW - np.array([0, 0, 0., 0., 0., 0.])) > 0.01:
        #    self.quadruped.updateKinematics()
        #print('initialC', self.quadruped.baseTwistW,self.quadruped.basePoseW, self.quadruped.q)
        

        # -------------------------------
        # Reset robot in Gazebo
        # -------------------------------
        #self.reset()
        data_new = [self.pubSub.pose, self.pubSub.twist, self.pubSub.joint_pos, self.pubSub.joint_vel, self.pubSub.imu_quat, self.pubSub.imu_ang_vel, self.pubSub.imu_lin_acc]
        print('self.pubSub.joint_pos',data_new[2])
        print('self.pubSub.joint_vel',data_new[3])

        self.pubSub.publish_button([3]) # Trot
        time.sleep(1)
        for self.step in range(max_steps):
            # Update messages
            data_new = [self.pubSub.pose, self.pubSub.twist, self.pubSub.joint_pos, self.pubSub.joint_vel, self.pubSub.imu_quat, self.pubSub.imu_ang_vel, self.pubSub.imu_lin_acc]
          #  print('self.pubSub.joint_pos',data_new[2])
          #  print('self.pubSub.joint_vel',data_new[3])
            terminate = self.check_termination(data_new)
            self.store_observations(data_new)
            if self.step > self.warmup_steps:
                # Stop if CP was reached successfully
                if terminate and self.step % self.decimation == 0:
                    #print('terminate', self.capture_flag)
                    print(f"Termination at {self.sim_time}")
                    break
            # -------------------------------
            # Observation and Action Computation
            # -------------------------------
            # Prepare observation

            # Inference (get new action)
            # Generate random initial command (velocity and yaw) to explore nominal policy states
            if self.step >= self.warmup_steps - 1:
                torque_noise = np.random.normal(0, noise_std, size=12)
            else:
                torque_noise = np.zeros(12)
  
            if self.step <= self.warmup_steps: #nominal policy
                if np.mod(self.sim_time, 0.5) == 0:
                    print(colored(f"TIME: {self.sim_time}", "blue"))
                
                # Apply external push randomly before warmup ends
                if self.step == push_instant:
                    #[self.pubSub.pose, self.pubSub.twist, self.pubSub.joint_pos, self.pubSub.joint_vel]
                    #apply as a twisch change
                    vx = np.random.uniform(-3, 3) #(-2.0, 2.0)#+ self.quadruped.baseTwistW[0]
                    vy = np.random.uniform(-3, 3) #(-2.0, 2.0) #+ self.quadruped.baseTwistW[1]
                    #debug makes it fall
                    # vx = -1.645
                    # vy = -1.239
                    print(f"Pushing robot at {push_instant} with twist  vx: {vx}, vy: {vy}")
                    push_vel = copy.copy(data_new[1])
                    push_vel[0] =  vx #+ push_vel[0]
                    push_vel[1] =  vy #+ push_vel[1]
                    #print(data_new[0], push_vel)
                    self.pubSub.publish_state(data_new[0], push_vel)
                cmd_vel = np.array([random_cmd[0], random_cmd[1], 0, 0, 0, 0, random_cmd[2]])
                self.pubSub.publish_vel(cmd_vel)
                self.pubSub.publish_backup(np.zeros(12),np.zeros(12),torque_noise)

                if not self.backup_trot and self.use_nn:
                    body_ang_vel = copy.copy(data_new[5])
                    proj_gravity = quat_rotate_inverse(
                        torch.tensor(data_new[4], device='cuda:0', dtype=torch.double).unsqueeze(0),
                        #torch.tensor(data_new[0][3:], device='cuda:0', dtype=torch.double).unsqueeze(0),
                        self.grav_tens
                    )[0].cpu().numpy()
                    qDes_no = self.backup_policy.action(data_new[6], None, body_ang_vel, proj_gravity, data_new[2], data_new[3], policy_type="safe")
    
            else:#switch to backup policy
                if np.mod(self.sim_time, 0.5) == 0:
                    print(colored(f"TIME: {self.sim_time}", "red"))
                if not self.use_nn:
                    cmd_vel = np.array([0, 0, 0, 0, 0, 0, 0.])
                    self.pubSub.publish_vel(cmd_vel)
                else:
                    if self.backup_trot:
                        qDes = self.backup_policy.compute_actions(data_new[4], data_new[5], data_new[2], data_new[3])
                    else:
                        body_ang_vel = copy.copy(data_new[5])
                        proj_gravity = quat_rotate_inverse(
                            torch.tensor(data_new[4], device='cuda:0', dtype=torch.double).unsqueeze(0),
                            #torch.tensor(data_new[0][3:], device='cuda:0', dtype=torch.double).unsqueeze(0),
                            self.grav_tens
                        )[0].cpu().numpy()
                        qDes = self.backup_policy.action(data_new[6], None, body_ang_vel, proj_gravity, data_new[2], data_new[3], policy_type="safe")
                    self.pubSub.publish_is_rec(False)
                    
                    self.pubSub.publish_backup(qDes,np.zeros(12),self.ffw_torques+torque_noise)
                #self.pubSub.publish_button(2) # Stance

            # Add noise to simulate real-world actuation
            #self.quadruped.tau_ffwd = np.zeros(12)

            #if self.sim_time >= self.warmup_time:
            #    noise = np.random.normal(0, noise_std, size=self.quadruped.tau_ffwd.shape)
            #    self.quadruped.tau_ffwd += noise

            self.rate_ros.sleep()
            self.sim_time = np.round(self.sim_time + self.dt, 4)  # np.array([self.loop_time]), 3)
        #print('final',data_new[0], push_vel)

        '''print('finquat', data_new[4])
        print('finxyz', data_new[0][:3])'''
        return np.array(self.observations), self.fallen_flag, self.capture_flag

    # -------------------------------
    # Run a Batch of Simulations and Save Results
    # -------------------------------
    def run_batch_simulations(self, n_episodes=100, save_path_relative="results", noise_std=1.0, seed=None):
        full_path = os.path.realpath(__file__)
        save_path = os.path.dirname(full_path) + "/../" + save_path_relative
        os.makedirs(save_path, exist_ok=True)
        all_obs = []
        stats = []
        max_len = 0

        if seed is not None:
            np.random.seed(seed)

        print("Running batch simulations...")
        self.reset()
        #for i in tqdm(range(n_episodes)):
        for i in (range(n_episodes)):
            print(colored(f"Simulation {i}-----------------------", "blue"))
           
            

            obs, fallen, captured = self.run_single_simulation(noise_std=noise_std,max_steps=3500, warmup_time=4.0)
            # -------------------------------
            # Reset robot in Gazebo
            # -------------------------------
            self.reset()
            print(colored(f"Fallen {fallen}, Captured {captured}", "green"))
            #print('obs', obs)
            for j in obs:
                if len(j) == 32:
                    print(obs)
                    exit()
            all_obs.append(obs)

            #debug
            #print(obs.shape)
            stats.append((fallen, captured, len(obs)))
            max_len = max(max_len, len(obs))

            if self.use_nn:
                if self.backup_trot:
                    self.backup_policy.actor_network.running_mean_std.running_mean = self.running_mean_backup
                    self.backup_policy.actor_network.running_mean_std.running_var = self.running_var_backup
                    self.backup_policy.actor_network.running_mean_std.count = self.count_backup
                    self.backup_policy.decimation_counter = 0
                    self.backup_policy.prev_actions = np.zeros(12)
                    self.backup_policy.qDes = self.backup_policy.q_def
                else:
                    self.backup_policy.prev_action = np.zeros(12)
                    self.backup_policy.decimation_counter = 0
                    self.backup_policy.history_buffer = np.zeros((1, 3, 48))
                self.pubSub.publish_is_rec(True)
                self.pubSub.publish_backup(np.zeros(12),np.zeros(12),np.zeros(12))
            time.sleep(2)

        # Pad observation arrays to same length in case of early termination (com comverget to cop)
        obs_dim = all_obs[0].shape[1]
        padded_obs = np.zeros((n_episodes, max_len, obs_dim), dtype=np.float32)

        for i, episode in enumerate(all_obs):
            padded_obs[i, :len(episode), :] = episode

        stats = np.array(stats, dtype=int)

        np.save(os.path.join(save_path, "observations_rl_controller_100_high_pushes_original_radius_test.npy"), padded_obs)

        print(f"Episodi completati: {n_episodes}")
        print(f"Caduti: {np.sum(stats[:, 0])}, CP raggiunto: {np.sum(stats[:, 1])}")
        print(f"Dati salvati in: {save_path}")
        print(f"Shape of observations: {padded_obs.shape}")
        self.deregister_node()