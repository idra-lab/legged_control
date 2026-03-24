from utils.value_function_manager import ValueFunctionManager
import utils.publish_subscribe as publish_subscribe
import rospy
import torch
import numpy as np
import time
from utils.backup import BackupPolicy
from utils.utils import load_config, quat_rotate_inverse, applyForce
import copy
import os



if __name__ == '__main__':
    full_path = os.path.realpath(__file__)
    config_path = os.path.dirname(full_path) + '/utils/config.yaml'
    config = load_config(config_path)
    backup_policy = BackupPolicy(config)
   # example_inputs = torch.randn(45)
   # onnx_program = torch.onnx.export(backup_policy.actor_network, example_inputs, dynamo=True)
   # onnx_program.save("backup.onnx")
   # exit()
    running_mean_backup = copy.copy(backup_policy.actor_network.running_mean_std.running_mean)
    running_var_backup = copy.copy(backup_policy.actor_network.running_mean_std.running_var)
    count_backup = copy.copy(backup_policy.actor_network.running_mean_std.count)

    kp_backup = config['robot']['kp']
    kd_backup = config['robot']['kd']
    backup_policy.commands = np.array(config['robot']['cmd_backup'])
    
    dt = config['simulation']['timestep_mps']

    
    # Initialize ROS
    rospy.init_node('communicate_aliengo')
    pubSub = publish_subscribe.PubSub()
    pubSub.init_publishers()
    pubSub.init_subscribers()

    # Frequency to compute value function
    rate_ros = rospy.Rate(1/dt)  # Compute value function every 10 ms

    

    grav_tens = torch.tensor([[0., 0., -1.]], device='cuda:0', dtype=torch.double)

    sim = True
    sim_push = True
    use_backup = True
    sim_time = 0
    time_rec = 0
    counter = 0
    prev_rec = True
    manual_switch = False
    manual_count = 0
    isrec = True
    use_nn = False
    stop = False
    stop_count = 0

    # Load value function
    vf = ValueFunctionManager(use_nn)
    if use_nn:
        threshold = 0.7
    else:
        threshold = 0.5
    pubSub.publish_is_rec(isrec)
    if sim:
        pubSub.publish_button([0])
        time.sleep(2)
        pubSub.publish_button([3])
        time.sleep(2)

         # Wait for subscribers
    #while pubSub.is_rec_pub.get_num_connections() < 1:# or pubSub.odom_data_pub.get_num_connections() < 1 or pubSub.joint_state_pub.get_num_connections() < 1:
            #pass
     #       print('no subscriber')
    
    while not rospy.is_shutdown():
        prev_rec = isrec
        if (sim_time > 0.5) and (sim_time - time_rec) % 2. == 0 and isrec and sim_push and sim and not stop:
            applyForce(0, 50*counter, 0, 0, 0, 0, 0.25)
            print(50*counter)
            counter += 1
            if manual_switch and counter > 1:
                isrec = False
        if manual_switch and not isrec:
            manual_count += 1
            if manual_count == 200:
                isrec = True
                manual_count = 0
                time_rec = sim_time

        # Read new data
        data_new = [pubSub.pose, pubSub.twist, pubSub.joint_pos, pubSub.joint_vel, pubSub.imu_quat, pubSub.imu_ang_vel, pubSub.imu_lin_acc]
        body_ang_vel = data_new[5]
        #body_ang_vel = data_new[1][3:]
        proj_gravity = quat_rotate_inverse(
            torch.tensor(data_new[4], device='cuda:0', dtype=torch.double).unsqueeze(0),
            #torch.tensor(data_new[0][3:], device='cuda:0', dtype=torch.double).unsqueeze(0),
            grav_tens
        )[0].cpu().numpy()
        if not manual_switch and (sim_time > 0.5):
            isrec, V_safe = vf.computeValueFnc(body_ang_vel, proj_gravity, joint_pos=data_new[2], joint_vel=data_new[3], threshold=threshold, vf_additional_term = 0.0)
            #print(V_safe)
        
        #isrec = False
        if not prev_rec and isrec and use_backup and not use_nn:
            print('STOP')
            stop = True
            pubSub.publish_button([2])
        if stop:
            stop_count += 1
        
            
        if not use_backup and not isrec:
            isrec = True
        elif use_backup and not isrec and not use_nn and not stop:
            pubSub.publish_button([4,5])
        elif sim and not stop:
            pubSub.publish_button([4])
        if not prev_rec and isrec and use_backup and use_nn:
            time_rec = sim_time
            backup_policy.actor_network.running_mean_std.running_mean = running_mean_backup
            backup_policy.actor_network.running_mean_std.running_var = running_var_backup
            backup_policy.actor_network.running_mean_std.count = count_backup
            backup_policy.decimation_counter = 0
            backup_policy.prev_actions = np.zeros(12)
            backup_policy.qDes = backup_policy.q_def
            pubSub.publish_is_rec(isrec)
        
        if not isrec and use_backup and use_nn:
            qDes = backup_policy.compute_actions(data_new[4], data_new[5], data_new[2], data_new[3])
            #qDes = backup_policy.compute_actions(data_new[0][3:], data_new[1][3:], data_new[2], data_new[3])
            pubSub.publish_is_rec(isrec)
            pubSub.publish_backup(qDes,np.zeros(12),np.zeros(12), kp_backup, kd_backup)
            
        #pubSub.publish_is_rec(False)
        if stop_count == 1000:
            stop_count = 0
            stop = False
            pubSub.publish_button([3])
            time_rec = sim_time
        rate_ros.sleep()
        sim_time = np.round(sim_time + dt, 4)