from simulation import TestManager

if __name__ == '__main__':
    tm = TestManager(use_nn = True)
    try:
        tm.run_simulations()

    except (rospy.ROSInterruptException, rospy.service.ServiceException):
        rospy.signal_shutdown("killed")