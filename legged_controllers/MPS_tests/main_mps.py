from simulation import TestManager
import rospy

if __name__ == '__main__':
    # No MPS: use_nn = False, only_mpc = True, only_rl = False, nom_rl = False
    # MPS = MPC+MPC: use_nn = True, only_mpc = True, only_rl = False, nom_rl = False
    # MPS = MPC+RL: use_nn = True, only_mpc = False, only_rl = False, nom_rl = False
    # MPS = RL+RL: use_nn = True, only_mpc = False, only_rl = True, nom_rl = False
    tm = TestManager(use_nn = True, only_mpc = False, only_rl = True, nom_rl = False, abs = True)
    try:
        tm.run_simulations()

    except (rospy.ROSInterruptException, rospy.service.ServiceException):
        rospy.signal_shutdown("killed")