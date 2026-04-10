from simulation import TestManager
import rospy

if __name__ == '__main__':
    tm = TestManager(use_nn = False)
    try:
        tm.run_simulations()

    except (rospy.ROSInterruptException, rospy.service.ServiceException):
        rospy.signal_shutdown("killed")