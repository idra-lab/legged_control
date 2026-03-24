import yaml
import torch
import torch.nn as nn
import numpy as np
from rl_games.algos_torch.running_mean_std import RunningMeanStd
from utils.utils import*
import os

# Function to load YAML configuration
def load_config(file_path):
    with open(file_path, 'r') as file:
        return yaml.safe_load(file)


# Actor Network Class
class ActorNetwork(nn.Module):
    def __init__(self, input_dim, action_dim, mlp_units=[512, 256, 128], activation=nn.ELU):
        super(ActorNetwork, self).__init__()
        layers = []
        prev_dim = input_dim
        for unit in mlp_units:
            layers.append(nn.Linear(prev_dim, unit))
            layers.append(activation())
            prev_dim = unit
        self.actor_mlp = nn.Sequential(*layers)
        self.mu = nn.Linear(mlp_units[-1], action_dim)
        self.running_mean_std = RunningMeanStd((input_dim,))
        

    def forward(self, x):
        features = self.actor_mlp(x)
        mu = self.mu(features)
        return mu

    def norm_obs(self, observation):
        with torch.no_grad():
            return self.running_mean_std(observation)

class BackupPolicy:
    def __init__(self, config, freq=50):
        
        input_dim = 45
        action_dim = 12
        self.device = config['networks']['device']
        self.scaling_factors = config['scaling']
        self.scale = config['scaling']['q_des']
        self.q_def = config['robot']['default_joint_angles']
        self.qDes = self.q_def
        self.prev_actions = np.zeros(12)
        dt = config['simulation']['timestep_mps']
        self.decimation = (1/dt)*(1/freq)
        self.decimation_counter = 0
        self.actor_network = ActorNetwork(input_dim=input_dim, action_dim=action_dim)

        full_path = os.path.realpath(__file__)
        policy_path = os.path.dirname(full_path) + "/../" + config['networks']['paths']['backup']
 
        state_dict = torch.load(policy_path, map_location=torch.device(config['networks']['device']), weights_only=False)['model']
        actor_state_dict = {k.replace('a2c_network.', ''): v for k, v in state_dict.items()
                            if k.startswith('a2c_network.actor_mlp') or k.startswith('a2c_network.mu') or k.startswith('running_mean_std.running_mean') or k.startswith('running_mean_std.running_var') or k.startswith('running_mean_std.count')}
        self.actor_network.load_state_dict(actor_state_dict)
        self.commands = np.zeros(3)
        self.actor_network.eval()

    def compute_actions(self, imu_quat, imu_gyro, qpos, qvel):
        """
        Compute the observation vector from the robot's state.
        """
        if self.decimation_counter == 0:
            body_quat = np.array([imu_quat[0], imu_quat[1], imu_quat[2], imu_quat[3]])
            body_vel = np.array([imu_gyro[0], imu_gyro[1], imu_gyro[2]])
        
            joint_angles = swap_legs(qpos)
            joint_velocities = swap_legs(qvel)

            # Gravity vector in body frame
            gravity_body = quat_rotate_inverse(
                torch.tensor(body_quat, device=self.device, dtype=torch.double).unsqueeze(0),
                torch.tensor([[0.0, 0.0, -1.0]], device=self.device, dtype=torch.double)
            )
            prev_actions = swap_legs(self.prev_actions)

            # Scale observations
            scaled_body_vel = body_vel * self.scaling_factors['body_ang_vel']
            scaled_commands = self.commands[:2] * self.scaling_factors['commands']
            scaled_commands = np.append(scaled_commands, self.commands[2] * self.scaling_factors['body_ang_vel'])
            scaled_gravity_body = gravity_body[0].cpu() * self.scaling_factors['gravity_body']
            scaled_joint_angles = np.array(joint_angles) * self.scaling_factors['joint_angles']
            scaled_joint_velocities = np.array(joint_velocities) * self.scaling_factors['joint_velocities']
            scaled_actions = prev_actions * self.scaling_factors['actions']
            
            obs = np.concatenate((scaled_body_vel, scaled_commands, scaled_gravity_body, scaled_joint_angles, scaled_joint_velocities, scaled_actions))

            obs_tensor = torch.tensor(obs, dtype=torch.float32)
            obs_normalized = self.actor_network.norm_obs(obs_tensor)
            with torch.no_grad():
                new_actions1 = self.actor_network(obs_normalized).numpy()

            current_actions = swap_legs(new_actions1)

            self.prev_actions = current_actions
            
            self.qDes = self.scale * current_actions + np.array(self.q_def)

            # Clip the joint angles to the joint limits
            for i in range(4):
                self.qDes[i*3] = np.clip(self.qDes[i*3], -1.22, 1.22) # Hip joint
                self.qDes[i*3+1] = np.clip(self.qDes[i*3+1], 0.0, 1.8) # Thigh joint
                self.qDes[i*3+2] = np.clip(self.qDes[i*3+2], -2.78, -0.65) # Calf joint
                
        self.decimation_counter = (self.decimation_counter+1) % self.decimation
        return self.qDes