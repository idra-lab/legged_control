#!/usr/bin/env python3
"""
Nodo ROS 2 per l'inferenza della Policy CAT (Unitree Go2).
Sottoscrive il topic custom /mpc/output, estrae i parametri del MPC e del Footstep,
esegue l'inferenza sulla rete neurale e pubblica su /lowcmd.
"""

import rclpy
from rclpy.node import Node
import torch
import numpy as np

# Import dei messaggi standard e del modulo geometrico custom dell'utente
from geometry_msgs.msg import Point, Vector3
try:
    from legged_opti_pessi_mpc.msg import MpcOutput, FootstepCommand
except ImportError:
    # Se i messaggi non sono ancora compilati nell'ambiente locale,
    # creiamo una classe di mockup al volo per non far fallire l'interprete Python
    print("Messaggi legged_opti_pessi_mpc non trovati. Uso classi di mockup.")
    class FootstepCommand:
        def __init__(self):
            self.p0_next = Point()
            self.p1_next = Point()
            self.duration = 0.25
            self.alpha = 0.0
            self.beta = 0.0
            self.gamma = 0.0
    class MpcOutput:
        def __init__(self):
            self.immediate_command = FootstepCommand()
            self.com_velocity_optimistic = [Vector3()]
            self.com_trajectory_optimistic = [Point()]

from unitree_go.msg import LowState, LowCmd, MotorState, MotorCmd

class MpcToLowCmdNode(Node):
    def __init__(self):
        super().__init__('mpc_to_lowcmd_node')

        # --- 1. PARAMETRI E STRUTTURE DATI ---
        self.declare_parameter('model_path', '/Isaaclab/logs/go2_cat/model_final.pt')
        self.declare_parameter('device', 'cuda' if torch.cuda.is_available() else 'cpu')
        
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.device = torch.device(self.get_parameter('device').get_parameter_value().string_value)
        
        self.action_scale = 0.3
        self.num_actions = 12
        self.num_observations = 34

        # Posizioni nominali di riposo dei giunti del Unitree Go2
        self.default_joint_pos = torch.tensor(
            [0.0, 0.9, -1.8,  0.0, 0.9, -1.8,  0.0, 0.9, -1.8,  0.0, 0.9, -1.8],
            device=self.device
        )

        # Buffer dello stato stimato del robot (da aggiornare in anello chiuso tramite /joint_states)
        self.current_joint_pos = self.default_joint_pos.clone()
        self.current_joint_vel = torch.zeros(12, device=self.device)
        self.base_ang_vel = torch.zeros(3, device=self.device)  # Da IMU / low_state

        # Vettori di stato estratti dinamicamente dal messaggio custom dell'MPC
        self.base_lin_vel = torch.zeros(3, device=self.device)
        self.cmd_lin_vel = torch.zeros(2, device=self.device)
        self.cmd_yaw_vel = torch.zeros(1, device=self.device)
        self.cmd_contact_duration = torch.tensor([0.25], device=self.device)
        
        # Variabili adimensionali e target geometrici del FootstepCommand
        self.mpc_params = torch.zeros(3, device=self.device) # alpha, beta, gamma

        # --- 2. RETE NEURALE MLP ---
        self.actor_net = torch.nn.Sequential(
            torch.nn.Linear(self.num_observations, 512),
            torch.nn.ELU(),
            torch.nn.Linear(512, 256),
            torch.nn.ELU(),
            torch.nn.Linear(256, 128),
            torch.nn.ELU(),
            torch.nn.Linear(128, self.num_actions)
        ).to(self.device)

        try:
            checkpoint = torch.load(model_path, map_location=self.device)
            if 'model_state_dict' in checkpoint:
                self.actor_net.load_state_dict(checkpoint['model_state_dict'])
            else:
                self.actor_net.load_state_dict(checkpoint)
            self.actor_net.eval()
            self.get_logger().info(f"Rete caricata con successo da {model_path}")
        except Exception as e:
            self.get_logger().error(f"Impossibile caricare il file .pt: {str(e)}")

        # --- 3. SOTTOSCRIZIONI E PUBBLICAZIONI ---
        # Sottoscrive il topic /mpc/output usando la classe MpcOutput generata dal tuo pacchetto
        self.mpc_sub = self.create_subscription(
            MpcOutput,
            '/mpc/output',
            self.mpc_output_callback,
            10
        )
        
        # Sottoscrizione allo stato dei motori /lowstate da Isaac Sim (via bridge)
        self.lowstate_sub = self.create_subscription(
            LowState,
            '/lowstate',
            self.lowstate_callback,
            10
        )
        
        self.lowcmd_pub = self.create_publisher(LowCmd, '/lowcmd', 10)

        # Loop deterministico di inferenza a 50Hz
        self.timer = self.create_timer(0.02, self.inference_loop)

    def mpc_output_callback(self, msg):
        """Callback per il parsing asincrono del messaggio /mpc/output dell'utente."""
        # 1. Estrazione della velocità lineare ottimistica stimata dall'MPC
        if len(msg.com_velocity_optimistic) > 0:
            self.base_lin_vel[0] = msg.com_velocity_optimistic[0].x
            self.base_lin_vel[1] = msg.com_velocity_optimistic[0].y
            self.base_lin_vel[2] = msg.com_velocity_optimistic[0].z

        # 2. Parsing dell'immediate_command (FootstepCommand)
        footstep = msg.immediate_command
        self.cmd_contact_duration[0] = float(footstep.duration)
        
        # Salviamo i parametri adimensionali alpha, beta, gamma
        self.mpc_params[0] = float(footstep.alpha)
        self.mpc_params[1] = float(footstep.beta)
        self.mpc_params[2] = float(footstep.gamma)

        # Mappiamo le coordinate planari di riferimento per i comandi x, y della policy
        if len(msg.com_trajectory_optimistic) > 0:
            self.cmd_lin_vel[0] = msg.com_trajectory_optimistic[0].x
            self.cmd_lin_vel[1] = msg.com_trajectory_optimistic[0].y

    def lowstate_callback(self, msg: LowState):
        """Riceve lo stato fisico dei motori da Isaac Sim via /lowstate."""
        # 1. Aggiorna posizioni e velocità dei giunti (primi 12 motori)
        for i in range(12):
            self.current_joint_pos[i] = float(msg.motor_state[i].q)
            self.current_joint_vel[i] = float(msg.motor_state[i].dq)
            
        # 2. Aggiorna le velocità angolari della base (dall'IMU nel LowState)
        self.base_ang_vel[0] = float(msg.imu_state.gyroscope[0])
        self.base_ang_vel[1] = float(msg.imu_state.gyroscope[1])
        self.base_ang_vel[2] = float(msg.imu_state.gyroscope[2])

    def inference_loop(self):
        """Costruisce l'osservazione ed esegue il pass avanti a 50Hz."""
        with torch.no_grad():
            # Ricostruzione esatta delle 34 dimensioni richieste dal modello PPO
            obs_tensor = torch.cat([
                self.base_lin_vel,                  # [3] Velocità lineare ottimistica dell'MPC
                self.base_ang_vel,                  # [3] Velocità angolare stimata
                self.current_joint_pos - self.default_joint_pos, # [12] Errore giunti
                self.current_joint_vel,             # [12] Velocità giunti
                self.cmd_lin_vel,                   # [2] Obiettivo planare
                self.cmd_yaw_vel,                   # [1] Yaw target
                self.cmd_contact_duration           # [1] Variabile delta (duration) del paper
            ], dim=-1).unsqueeze(0)                 # Shape finale: [1, 34]

            # Infezione deterministica
            raw_action = self.actor_net(obs_tensor).squeeze(0)
            clean_action = torch.clamp(raw_action, -1.2, 1.2)
            
            # Denormalizzazione cinematica per calcolare i radianti dei motori
            target_positions = (self.action_scale * clean_action + self.default_joint_pos).cpu().numpy()

            # Pubblicazione sul topic /lowcmd
            self.publish_low_cmd(target_positions)

    def publish_low_cmd(self, target_positions):
        """Invia i pacchetti ai motori PD tramite la struttura LowCmd di Unitree."""
        # Se usiamo la classe mock std_msgs/Float32MultiArray
        if not hasattr(LowCmd, 'motor_cmd'):
            msg = LowCmd()
            msg.data = target_positions.tolist()
            self.lowcmd_pub.publish(msg)
            return

        msg = LowCmd()
        kp_stance = 35.0
        kd_stance = 1.0

        # Inizializza l'array motor_cmd di 20 elementi (richiesto da ROS 2 Python per array a dimensione fissa)
        for _ in range(20):
            msg.motor_cmd.append(MotorCmd())

        # Configura i primi 12 motori (actuated joints del Go2)
        for i in range(12):
            msg.motor_cmd[i].q = float(target_positions[i])
            msg.motor_cmd[i].dq = 0.0
            msg.motor_cmd[i].kp = kp_stance
            msg.motor_cmd[i].kd = kd_stance
            msg.motor_cmd[i].tau = 0.0
            
        self.lowcmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MpcToLowCmdNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()