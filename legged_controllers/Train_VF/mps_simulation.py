from utils.value_function_manager import ValueFunctionManager
import utils.publish_subscribe as publish_subscribe
import rospy
import torch
import numpy as np
import time
from utils.backup import BackupPolicy
from utils.rl_controller import RlVelocityController
from utils.utils import load_config, quat_rotate_inverse, applyForce
import copy
import os
from utils.backup_stop import BackupStop



if __name__ == '__main__':
    full_path = os.path.realpath(__file__)
    config_path = os.path.dirname(full_path) + '/utils/config.yaml'
    config = load_config(config_path)

    dt = config['simulation']['timestep_mps']
    sim_time = 0
    time_rec = 0
    counter = 0
    manual_count = 0
    stop_count = 0
    decimation = 5
    decimation_counter = 0

    sim = False
    sim_push = False
    use_backup = True
    use_joy = True
    prev_rec = True
    manual_switch = False
    isrec = True
    use_nn = True
    stop = False
    stop_backup = False
    only_backup = False
    backup_trot = False
    push_once = False

    if backup_trot:
        backup_policy = BackupPolicy(config)
        running_mean_backup = copy.copy(backup_policy.actor_network.running_mean_std.running_mean)
        running_var_backup = copy.copy(backup_policy.actor_network.running_mean_std.running_var)
        count_backup = copy.copy(backup_policy.actor_network.running_mean_std.count)

        kp_backup = config['robot']['kp']
        kd_backup = config['robot']['kd']
        backup_policy.commands = np.array(config['robot']['cmd_backup'])
    else:
        if stop_backup:
            backup_policy = BackupStop(config)
            backup_policy.last_action= np.zeros(12)
            kp_backup = config['stop']['robot']['Kp_b']
            kd_backup = config['stop']['robot']['Kd_b']
        else:
            backup_policy = RlVelocityController('aliengo', dt, use_nn_se=True)
            kp_backup = backup_policy.kp[0]
            kd_backup = backup_policy.kd[0]
            backup_policy.velocity_cmd = np.zeros(3)
    #backup_policy.commands = np.array([-0.25, 0., 0.])
    
    

    
    # Initialize ROS
    rospy.init_node('communicate_aliengo')
    pubSub = publish_subscribe.PubSub()
    pubSub.init_publishers()
    pubSub.init_subscribers()

    # Frequency to compute value function
    rate_ros = rospy.Rate(1/dt)  # Loop function every 2 ms

    

    grav_tens = torch.tensor([[0., 0., -1.]], device='cuda:0', dtype=torch.double)

    

    if backup_trot:
        ffw_torques = np.array([1.6, 0.0, 0.0,      # LF 
                                1.6, 0.0, 0.0,     # LH 
                                -1.6, 0.0, 0.0,      # RF
                                -1.6, 0.0, 0.0])*1  # RH
    else:
        ffw_torques = np.zeros(12)
    pubSub.publish_backup(np.zeros(12),np.zeros(12),np.zeros(12), 0, 0)
    # Load value function
    vf = ValueFunctionManager(use_nn, stop=False)
    if use_nn:
        if stop:
            threshold = 0
        else:
            threshold = 0.5#5#7
    else:
        threshold = 0.5
    # Run value function to compile it
    

    pubSub.publish_is_rec(isrec)
    if sim and not use_joy:
        pubSub.publish_button([0])
        #time.sleep(2)
        pubSub.publish_button([3])
        time.sleep(2)

         # Wait for subscribers
    #while pubSub.is_rec_pub.get_num_connections() < 1:# or pubSub.odom_data_pub.get_num_connections() < 1 or pubSub.joint_state_pub.get_num_connections() < 1:
            #pass
     #       print('no subscriber')

    
    while not rospy.is_shutdown():
        prev_rec = isrec
        if (sim_time > 0.5) and (sim_time - time_rec) % 2. == 0 and isrec and sim_push and sim and not stop:
            if not push_once or counter == 0:
                #applyForce(0, 50*counter, 0, 0, 0, 0, 0.25)
                #print(50*counter)
                force_push = 100
                applyForce(0, force_push, 0, 0, 0, 0, 0.25)
                print(force_push)
                counter += 1
                if manual_switch and counter > 1:
                    isrec = False
        if manual_switch and not isrec:
            manual_count += 1
            if manual_count == 2000:
                isrec = True
                manual_count = 0
                time_rec = sim_time

        # Read new data
        
        data_new = [pubSub.pose, pubSub.twist, pubSub.joint_pos, pubSub.joint_vel, pubSub.imu_quat, pubSub.imu_ang_vel, pubSub.imu_lin_acc]
        #print(pubSub.imu_lin_acc)
        #exit()
        body_ang_vel = data_new[5]
        #body_ang_vel = data_new[1][3:]
        proj_gravity = quat_rotate_inverse(
            torch.tensor(data_new[4], device='cuda:0', dtype=torch.double).unsqueeze(0),
            #torch.tensor(data_new[0][3:], device='cuda:0', dtype=torch.double).unsqueeze(0),
            grav_tens
        )[0].cpu().numpy()
        if not manual_switch and (sim_time > 0.5) and (decimation_counter % decimation)==0:
            isrec2, V_safe = vf.computeValueFnc(body_ang_vel, proj_gravity, joint_pos=data_new[2], joint_vel=data_new[3], threshold=threshold, vf_additional_term = 0.0)
            #print(V_safe)
        #isrec =True
        if only_backup:
            isrec = False
        if not prev_rec and isrec and use_backup and not use_nn and not use_joy:
            print('STOP')
            stop = True
            pubSub.publish_button([2])
        if stop:
            stop_count += 1
        
            
        if not use_backup and not isrec:
            isrec = True
        elif use_backup and not isrec and not use_nn and not stop and not use_joy:
            pubSub.publish_button_no_joy([4,5])
        if sim and not stop and not use_joy and isrec:
            pubSub.publish_button_no_joy([4])
        if not prev_rec and isrec and use_backup and use_nn:
            pubSub.publish_backup(np.zeros(12),np.zeros(12),np.zeros(12), 0, 0)
            time_rec = sim_time
            if backup_trot:
                backup_policy.actor_network.running_mean_std.running_mean = running_mean_backup
                backup_policy.actor_network.running_mean_std.running_var = running_var_backup
                backup_policy.actor_network.running_mean_std.count = count_backup
                backup_policy.decimation_counter = 0
                backup_policy.prev_actions = np.zeros(12)
                backup_policy.qDes = backup_policy.q_def
            else:
                if stop_backup:
                    backup_policy.last_action= np.zeros(12)
                    backup_policy.decimation_counter = 0
                else:
                    backup_policy.prev_action = np.zeros(12)
                    backup_policy.decimation_counter = 0
                    #backup_policy.history_buffer = np.zeros((1, 3, 48))
            pubSub.publish_is_rec(isrec)
        
        if isrec and use_backup and use_nn and not stop_backup:
            qDes_no = backup_policy.action(data_new[6], None, body_ang_vel, proj_gravity, data_new[2], data_new[3], policy_type="safe")
            pubSub.publish_backup(qDes_no,np.zeros(12),ffw_torques, kp_backup, kd_backup)
        if not isrec and use_backup and use_nn:
            if backup_trot:
                qDes = backup_policy.compute_actions(data_new[4], data_new[5], data_new[2], data_new[3])
                #qDes = backup_policy.compute_actions(data_new[0][3:], data_new[1][3:], data_new[2], data_new[3])
            else:
                if stop_backup:
                    qDes = backup_policy.computeBackup(data_new[2], data_new[3], data_new[6])
                else:
                    qDes = backup_policy.action(data_new[6], None, body_ang_vel, proj_gravity, data_new[2], data_new[3], policy_type="safe")
            pubSub.publish_is_rec(isrec)
            pubSub.publish_backup(qDes,np.zeros(12),ffw_torques, kp_backup, kd_backup)
            
        #pubSub.publish_is_rec(False)
        if stop_count == 1000:
            stop_count = 0
            stop = False
            pubSub.publish_button([3])
            time_rec = sim_time

        decimation_counter += 1
        
        sim_time = np.round(sim_time + dt, 4)
        rate_ros.sleep()