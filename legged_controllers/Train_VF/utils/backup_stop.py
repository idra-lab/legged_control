import torch
import torch.nn as nn
import numpy as np
from collections import OrderedDict
import os

# Backup policy
class BackupStop(nn.Module):
    def __init__(self, config, freq=50):
        """ Neural network to stop the robot using only sensor data """
        super(BackupStop, self).__init__()
        
        # Data to preprocess state before sending it to the network
        self.device = config['networks']['device']
        self.threshold = float(config['stop']['scaling']['clip_threshold'])
        self.joint_def = torch.tensor(config['stop']['scaling']['default_joint_angles'], device=torch.device(self.device), dtype=torch.float64)
        self.scaling_factor = float(config['stop']['scaling']['factor'])
        epsilon = float(config['stop']['scaling']['epsilon'])
        self.qDes = self.orderBackup(self.joint_def)
        dt = config['simulation']['timestep_mps']
        self.decimation = (1/dt)*(1/freq)
        self.decimation_counter = 0

        self.last_action = np.zeros(12)
        backup_name = config['stop']['policy']
        self.mean = torch.tensor(config['stop']['scaling']['running_mean' + backup_name[1:]], device=torch.device(self.device), dtype=torch.float64)
        running_variance = torch.tensor(config['stop']['scaling']['running_variance' + backup_name[1:]], device=torch.device(self.device), dtype=torch.float64)
        self.scale = torch.sqrt(running_variance.float()) + epsilon
        
        # Change directory keys
        full_path = os.path.realpath(__file__)
        PATH = os.path.dirname(full_path) + "/../" + config['networks']['paths']['stop' + backup_name[1:]]
        old_keys = config['stop']['dictionary']['old_keys']
        new_keys = config['stop']['dictionary']['new_keys']
        if '1' == backup_name[1:]:
            dict_policy = torch.load(PATH, map_location=torch.device(self.device), weights_only=True)['policy']
        else:
            dict_policy = torch.load(PATH, map_location=torch.device(self.device), weights_only=True)
        policy_dict = self.labels_policy_dict(dict_policy, old_keys, new_keys)
        #self.load_state_dict(policy_dict)

        # Define network architechture
        self.layers = nn.Sequential(
            nn.Linear(42, 256),
            nn.ELU(),
            nn.Linear(256, 256),
            nn.ELU(),
            nn.Linear(256, 128),
            nn.ELU(),
            nn.Linear(128, 12)
        ).to(self.device)

        self.load_state_dict(policy_dict)

    def forward(self, x):
        """ Neural network inference """
        return self.layers(x)
    
    def labels_policy_dict(self, old_state_dict, old_keys, new_keys):
        """ Rename the dictionary keys for the required ones """
        new_policy_dict = OrderedDict()
        new_dict = 0
        for k, v in old_state_dict.items():
            if k in old_keys:
                name = new_keys[new_dict]
                new_policy_dict[name] = v
                new_dict += 1
        return new_policy_dict

    def computeBackup(self, q_muj, v_muj, imu):
        """ Scale input vector before inference and compute the desired joint positions,
            the network receives the following:
            * Velocity_command   -->   torch.tensor([0.0, 0.0, 0.0])
            * IMU (gravity is considered)
            * Joint_Position
            * Joint_Velocity
            * Last_action """
        if self.decimation_counter == 0:
            imu[0] = imu[0]
            imu[1] = imu[1]
            imu[2] = imu[2]
            # Network inputs
            vel_comm = np.zeros(3)
            pos_order = self.orderData(q_muj)
            vel_order = self.orderData(v_muj)
            state_order = np.concatenate((vel_comm, imu, pos_order, vel_order, self.last_action))
            state_torch = torch.from_numpy(state_order)
            state_torch = state_torch.to(self.device, torch.float32)

            # Offset joint positions as required by the network
            state_torch[6:18] = state_torch[6:18] - self.joint_def

            # Input scaling
            scaled_state = torch.clamp((state_torch - self.mean.float()) / self.scale,
                        min=-self.threshold, max=self.threshold)
                        
            # Inference
            new_action = self.forward(scaled_state) 

            # Compute desired joint positions
            pos_backup_order = self.orderBackup((new_action * self.scaling_factor) + self.joint_def)
            self.qDes = pos_backup_order
            self.last_action = new_action.detach().cpu().numpy()
        self.decimation_counter = (self.decimation_counter+1) % self.decimation
        
        return self.qDes

    def orderData(self, data):
        """ Change order of joint data from the one given by Pinocchio 
            to the one considered by the network and vice versa"""
        """Policy order:
        0'FL_hip_joint', 1'FR_hip_joint', 2'RL_hip_joint', 3'RR_hip_joint', 
        4'FL_thigh_joint', 5'FR_thigh_joint', 6'RL_thigh_joint', 7'RR_thigh_joint',
        8'FL_calf_joint', 9'FR_calf_joint', 10'RL_calf_joint', 11'RR_calf_joint'"""
        order = [0, 6, 3, 9, 1, 7, 4, 10, 2, 8, 5, 11]
        order_final = np.zeros(12)
        for i in range(12):
            order_final[i] = data[order[i]]
        return order_final
    
    def orderBackup(self, action):
        order_action = [0, 4, 8, 2, 6, 10, 1, 5, 9, 3, 7, 11]
        order_final = np.zeros(12)
        for i in range(12):
            order_final[i] = action[order_action[i]]
        return order_final