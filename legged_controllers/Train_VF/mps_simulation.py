from utils.value_function_manager import ValueFunctionManager
import utils.publish_subscribe as publish_subscribe
import rospy
import torch
import numpy as np
import time
from utils.backup import BackupPolicy
from utils.rl_controller import RlVelocityController
from utils.utils import load_config, quat_rotate_inverse, applyForce, launchFileNode
import copy
import os
from utils.backup_stop import BackupStop
from gazebo_msgs.srv import *
from std_srvs.srv import Empty
from gazebo_msgs.msg import ModelState
from controller_manager_msgs.srv import SwitchController, LoadController, UnloadController

np.set_printoptions(linewidth=100000)

def reset():
        initial_pose = np.array([0, 0, 0.45, 0, 0, 0, 1])

        joint_names = ['LF_HAA', 'LF_HFE', 'LF_KFE', 
                        'LH_HAA', 'LH_HFE', 'LH_KFE', 
                        'RF_HAA', 'RF_HFE', 'RF_KFE', 
                        'RH_HAA', 'RH_HFE', 'RH_KFE']
        joint_positions = [ -0.1, 0.62, -1.24,
                            -0.1, 0.62, -1.24,
                             0.1, 0.62, -1.24,
                             0.1, 0.62, -1.24]
        rospy.wait_for_service('/controller_manager/switch_controller')
        call_service("/controller_manager/switch_controller", SwitchController,
        start_controllers=[""],
        stop_controllers=["controllers/mps_controller",'controllers/joint_state_controller', 'controllers/imu_sensor_controller'],
        strictness=1, start_asap=False, timeout=0.0)

        rospy.wait_for_service('/controller_manager/unload_controller')
        unload_srv = rospy.ServiceProxy('/controller_manager/unload_controller', UnloadController)
        
        # Chiamare il servizio
        resp = unload_srv("controllers/mps_controller")
        #time.sleep(2)
        #rospy.wait_for_service('/gazebo/reset_simulation')
        
        #time.sleep(1)
        #reset_service = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        #reset_service()


        rospy.wait_for_service('/gazebo/set_model_configuration')
        rospy.wait_for_service('/gazebo/unpause_physics')
        
        #time.sleep(1)
        pause_physics_client = rospy.ServiceProxy('/gazebo/pause_physics',Empty)  
        unpause_physics_client = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        
        pause_physics_client()
        rospy.wait_for_service('/gazebo/set_model_state')
        #get_physics_client = rospy.ServiceProxy('/gazebo/get_physics_properties', GetPhysicsProperties)
        
        #set_physics_client = rospy.ServiceProxy('/gazebo/set_physics_properties', SetPhysicsProperties)
  

 
        # get actual configs
        '''physics_props = get_physics_client()  
        req_reset_world = SetModelStateRequest()       

        req_reset_gravity = SetPhysicsPropertiesRequest()
        #ode config
        req_reset_gravity.time_step = physics_props.time_step
        req_reset_gravity.max_update_rate = physics_props.max_update_rate           
        req_reset_gravity.ode_config =physics_props.ode_config
        req_reset_gravity.gravity =  physics_props.gravity
        req_reset_gravity.gravity.z =  -9.81
        set_physics_client(req_reset_gravity)'''
        reset_world = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        model_state = ModelState()
        model_state.model_name = 'aliengo'
        model_state.pose.position.x = initial_pose[0]
        model_state.pose.position.y = initial_pose[1]
        model_state.pose.position.z = initial_pose[2]

        model_state.pose.orientation.x = initial_pose[3]
        model_state.pose.orientation.y = initial_pose[4]
        model_state.pose.orientation.z = initial_pose[5]
        model_state.pose.orientation.w = initial_pose[6]

        model_state.twist.linear.x = 0
        model_state.twist.linear.y = 0
        model_state.twist.linear.z = 0

        model_state.twist.angular.x = 0
        model_state.twist.angular.y = 0
        model_state.twist.angular.z = 0

        req = SetModelStateRequest()
        req.model_state = model_state
        #time.sleep(1)
        reset_world(req)

        rospy.wait_for_service('/gazebo/set_model_configuration')
        time.sleep(1)

        set_model_configuration = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)
        #time.sleep(1)
        req_config = SetModelConfigurationRequest()
        req_config.model_name ='aliengo'
        req_config.urdf_param_name = 'robot_description'
        req_config.joint_names = joint_names
        req_config.joint_positions = joint_positions

        resp = set_model_configuration(req_config)

        time.sleep(1.)
        
        


        rospy.wait_for_service('/controller_manager/switch_controller')
        rospy.wait_for_service('/controller_manager/load_controller')
        rospy.wait_for_service('/controller_manager/switch_controller')
        unpause_physics_client()

        
        load_srv = rospy.ServiceProxy('/controller_manager/load_controller', LoadController)
        
        # Chiamare il servizio
        resp = load_srv("controllers/mps_controller")
        call_service("/controller_manager/switch_controller", SwitchController,
            start_controllers=["controllers/mps_controller",'controllers/joint_state_controller', 'controllers/imu_sensor_controller'],
            stop_controllers=[""],
            strictness=1, start_asap=False, timeout=0.0)
        
    
        pubSub.publish_button([2])
        time.sleep(2)

        
        

def call_service(ns, cls, **kwargs):
        rospy.wait_for_service(ns)
        service = rospy.ServiceProxy(ns, cls)
        response = service(**kwargs)

if __name__ == '__main__':
    import os
    os.system('pkill rosmaster')
    os.system('pkill gzserver')

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

    sim = True
    sim_push = False
    use_backup = True
    use_joy = False
    prev_rec = True
    manual_switch = False
    isrec = True
    use_nn = True
    stop = False
    stop_backup = False
    only_backup = False
    backup_trot = False
    push_once = False

    grav_tens = torch.tensor([[0., 0., -1.]], device='cuda:0', dtype=torch.double)

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
    

    

    launch_world = launchFileNode('legged_unitree_description','empty_world.launch', additional_args=['use_sim_time:=true', 'gz_gui:=true'])
    launch_world.start()
    time.sleep(1)
    if use_nn:
        nn_arg = 'nn:=true'
    else:
        nn_arg = 'nn:=false'
    launch_controller = launchFileNode('legged_controllers', 'load_controller.launch', additional_args=['joy:=true', nn_arg, 'mps:=true', 'joy_msg:=false'])
    launch_controller.start()
    #time.sleep(2)

    # Initialize ROS
    rospy.init_node('communicate_aliengo')
    
    pubSub = publish_subscribe.PubSub()
    pubSub.init_publishers()
    pubSub.init_subscribers()

    rate_ros = rospy.Rate(1/dt)  # 500 Hz for dt = 0.002

    if backup_trot:
        ffw_torques = np.array([1.6, 0.0, 0.0,      # LF 
                                1.6, 0.0, 0.0,     # LH 
                                -1.6, 0.0, 0.0,      # RF
                                -1.6, 0.0, 0.0])*1  # RH
    else:
        ffw_torques = np.zeros(12)
    pubSub.publish_backup(np.zeros(12),np.zeros(12),np.zeros(12))
    # Load value function
    vf = ValueFunctionManager(use_nn, stop=False)
    if use_nn:
        if stop:
            threshold = 0
        else:
            threshold = 0.6#5#7
    else:
        threshold = 0.5
    

    pubSub.publish_is_rec(isrec)

    if sim:
        reset()
    
    while not rospy.is_shutdown():
        if sim and not use_joy:
            #pubSub.publish_button([0])
            #time.sleep(2)
            pubSub.publish_button([3])
            #time.sleep(2)
        #if sim_time == 10:
        #    pubSub.publish_world_control(True)
        #    pubSub.publish_reset(True)
        prev_rec = isrec
        #if sim_time == 1:
        #    print(data_new)
        if (sim_time > 0.5) and (sim_time - time_rec) % 2. == 0 and isrec and sim_push and sim and not stop:
            if not push_once or counter == 0:
                applyForce(0, 50*counter, 0, 0, 0, 0, 0.25)
                print(50*counter)
                #force_push = 100
                #applyForce(0, force_push, 0, 0, 0, 0, 0.25)
                #print(force_push)
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
            isrec, V_safe = vf.computeValueFnc(body_ang_vel, proj_gravity, joint_pos=data_new[2], joint_vel=data_new[3], threshold=threshold, vf_additional_term = 0.0)
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
            pubSub.publish_backup(np.zeros(12),np.zeros(12),np.zeros(12))
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
            pubSub.publish_backup(qDes_no, np.zeros(12), ffw_torques)
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
            pubSub.publish_backup(qDes,np.zeros(12),ffw_torques)
            
        #pubSub.publish_is_rec(False)
        if stop_count == 1000:
            stop_count = 0
            stop = False
            pubSub.publish_button([3])
            time_rec = sim_time

        decimation_counter += 1
        
        sim_time = np.round(sim_time + dt, 4)
        rate_ros.sleep()
