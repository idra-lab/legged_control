from simulation import TestManager
import rospy

if __name__ == '__main__':
    tm = TestManager(use_nn = True, only_mpc = False, only_rl = False, nom_rl = True)
    try:
        tm.run_simulations()

    except (rospy.ROSInterruptException, rospy.service.ServiceException):
        rospy.signal_shutdown("killed")