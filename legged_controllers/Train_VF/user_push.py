from utils.utils import applyForce
import rospy

if __name__ == '__main__':
     while not rospy.is_shutdown():
            user_force = input("Enter force for push: ")
            if user_force.isdecimal():
                applyForce(0, float(user_force), 0, 0, 0, 0, 0.25)
            else:
                print('Invalid force.')