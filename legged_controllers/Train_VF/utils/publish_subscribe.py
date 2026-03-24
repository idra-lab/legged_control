import rospy
import numpy as np
from sensor_msgs.msg import Imu, JointState, Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Bool
from nav_msgs.msg import Odometry
from ocs2_msgs.msg._mode_schedule import mode_schedule
from gazebo_msgs.msg import ModelState, ModelStates
from nav_msgs.msg import Odometry
import threading
import copy

class PubSub():
    def __init__(self):
        # Publish velociy to stand up
        self.cmd_vel = Twist()
        self.mode = mode_schedule()
        self.button = Joy()
        self.state = ModelState()
        self.is_rec = Bool()
        self.joints_backup = JointState()

        self.lock_state = threading.Lock()
        self.lock_joint = threading.Lock()
        self.lock_imu = threading.Lock()

        self.pose = np.zeros(7)
        self.twist = np.zeros(6)

        self.joint_pos = np.zeros(12)
        self.joint_vel = np.zeros(12)

        self.joint_pos_backup = np.zeros(13)
        self.joint_vel_backup = np.zeros(13)
        self.joint_eff_backup = np.zeros(13)

        self.imu_quat = np.zeros(4)
        self.imu_ang_vel = np.zeros(3)
        self.imu_lin_acc = np.zeros(3)




    def callback_state(self, msg):
        with self.lock_state:
            model_name = 'aliengo' # Change this to your model's name in Gazebo
            index = msg.name.index(model_name)

            # Position [x, y, z]
            self.pose = np.array([
                msg.pose[index].position.x,
                msg.pose[index].position.y,
                msg.pose[index].position.z,
                msg.pose[index].orientation.x,
                msg.pose[index].orientation.y,
                msg.pose[index].orientation.z,
                msg.pose[index].orientation.w,
            ], dtype=np.float32)

            # Linear velocity [x, y, z]
            self.twist = np.array([
                msg.twist[index].linear.x,
                msg.twist[index].linear.y,
                msg.twist[index].linear.z,
                msg.twist[index].angular.x,
                msg.twist[index].angular.y,
                msg.twist[index].angular.z
            ], dtype=np.float32)

    def callback_imu(self, msg):
        with self.lock_imu:

            self.imu_quat = np.array([
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w
            ], dtype=np.float32)

            self.imu_ang_vel = np.array([
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z
            ], dtype=np.float32)

            self.imu_lin_acc = np.array([
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z
            ], dtype=np.float32)

    def callback_joint(self, data):
      # Data is received in the following order:
      #  0  LF_HAA
      #  1  LF_HFE
      #  2  LF_KFE
      #  3  LH_HAA
      #  4  LH_HFE
      #  5  LH_KFE
      #  6  RF_HAA
      #  7  RF_HFE
      #  8  RF_KFE
      #  9  RH_HAA
      #  10 RH_HFE
      #  11 RH_KFE

      # we convert into LH RH LF RF

        with self.lock_joint:
            order = [6, 7, 8, 0, 1, 2, 9, 10, 11, 3, 4, 5]
        
            for i in range(12):
                self.joint_pos[order[i]] = data.position[i]
                self.joint_vel[order[i]] = data.velocity[i]

    def init_subscribers(self):
        self.joint_state_sub = rospy.Subscriber('/joint_states', JointState, self.callback_joint)
        self.pose_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback_state)
        self.imu_sub = rospy.Subscriber('/base_imu', Imu, self.callback_imu)
        
    def init_publishers(self):
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=None, tcp_nodelay=True)
        self.state_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=None, tcp_nodelay=True)
        self.button_pub = rospy.Publisher('/joy', Joy, queue_size=None, tcp_nodelay=True)
        self.is_rec_pub = rospy.Publisher('/is_rec', Bool, queue_size=None, tcp_nodelay=True)
        self.joints_backup_pub = rospy.Publisher('/joints_backup', JointState, queue_size=None, tcp_nodelay=True)

    def publish_backup(self, pos, vel, eff, kp, kd):
        # Data needs to be sent in the following order:
        #  0  LF_HAA
        #  1  LF_HFE
        #  2  LF_KFE
        #  3  LH_HAA
        #  4  LH_HFE
        #  5  LH_KFE
        #  6  RF_HAA
        #  7  RF_HFE
        #  8  RF_KFE
        #  9  RH_HAA
        #  10 RH_HFE
        #  11 RH_KFE

        order = [6, 7, 8, 0, 1, 2, 9, 10, 11, 3, 4, 5]
        try:
            for i in range(12):
                self.joint_pos_backup[i] = pos[order[i]]
                self.joint_vel_backup[i] = vel[order[i]]
                self.joint_eff_backup[i] = eff[order[i]]
            
            self.joint_pos_backup[12] = kp
            self.joint_vel_backup[12] = kd
            self.joint_eff_backup[12] = 0
            
            self.joints_backup.position = self.joint_pos_backup
            self.joints_backup.velocity = self.joint_vel_backup
            self.joints_backup.effort = self.joint_eff_backup
            self.joints_backup_pub.publish(self.joints_backup)
            
        except rospy.ROSInterruptException:
            pass

    def publish_is_rec(self, is_rec):
        try:
            self.is_rec.data = is_rec
            self.is_rec_pub.publish(self.is_rec)
            
        except rospy.ROSInterruptException:
            pass

    def publish_vel(self, cmd):
        try:
            self.cmd_vel.linear.x = cmd[0]
            self.cmd_vel.linear.y = cmd[1]
            self.cmd_vel.linear.z = cmd[2]
            self.cmd_vel.angular.x = cmd[3]
            self.cmd_vel.angular.y = cmd[4]
            self.cmd_vel.angular.z = cmd[5]
            self.cmd_pub.publish(self.cmd_vel)
            
        except rospy.ROSInterruptException:
            pass

    def publish_button(self, mode):
        try:
            buttons = [0,0,0,0,0,0,0,0,0,0,0]
            for i in mode:
                buttons[i] = 1
            self.button.buttons = buttons
            self.button_pub.publish(self.button)
            
        except rospy.ROSInterruptException:
            pass

    def publish_button_no_joy(self, mode):
        try:
            buttons = [0,0,0,0,0,0,0,0,0,0,0]
            for i in mode:
                buttons[i] = 1
            if 4 in mode and 5 in mode:
                self.button.axes = [0,0,0,0,0,0,0,0]
            elif 4 in mode:
                self.button.axes = [0,0.5,0,0,0,0,0,0]
            self.button.buttons = buttons
            self.button_pub.publish(self.button)
            
        except rospy.ROSInterruptException:
            pass
        

    def publish_state(self, state_pose, state_twist):
        try:
            self.state.model_name = 'aliengo'

            self.state.pose.position.x = state_pose[0]
            self.state.pose.position.y = state_pose[1]
            self.state.pose.position.z = state_pose[2]

            self.state.pose.orientation.x = state_pose[3]
            self.state.pose.orientation.y = state_pose[4]
            self.state.pose.orientation.z = state_pose[5]
            self.state.pose.orientation.w = state_pose[6]

            self.state.twist.linear.x = state_twist[0]
            self.state.twist.linear.y = state_twist[1]
            self.state.twist.linear.z = state_twist[2]

            self.state.twist.angular.x = state_twist[3]
            self.state.twist.angular.y = state_twist[4]
            self.state.twist.angular.z = state_twist[5]
 
            self.state_pub.publish(self.state)
            
        except rospy.ROSInterruptException:
            pass