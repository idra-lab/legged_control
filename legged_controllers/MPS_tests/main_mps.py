from simulation import TestManager
import rospy

if __name__ == '__main__':
    # No MPS MPC:          use_nn = False, only_mpc = True,  only_rl = False, nom_rl = False, only_nom = True,  abs_vf = False, sensor_rl=False
    # MPS = MPC+MPC:       use_nn = True,  only_mpc = True,  only_rl = False, nom_rl = False, only_nom = False, abs_vf = False, sensor_rl=False
    # MPS = MPC+RL:        use_nn = True,  only_mpc = False, only_rl = False, nom_rl = False, only_nom = False, abs_vf = False, sensor_rl=False
    # MPS = RL+RL:         use_nn = True,  only_mpc = False, only_rl = True,  nom_rl = False, only_nom = False, abs_vf = False, sensor_rl=False
    # No MPS RL:           use_nn = True,  only_mpc = False, only_rl = True,  nom_rl = False, only_nom = True,  abs_vf = False, sensor_rl=False
    # MPS ABS:             use_nn = True,  only_mpc = False, only_rl = True,  nom_rl = False, only_nom = False, abs_vf = True,  sensor_rl=False
    # MPS sensor-based:    use_nn = True,  only_mpc = False, only_rl = True,  nom_rl = False, only_nom = False, abs_vf = False, sensor_rl=True
    # No MPS sensor-based: use_nn = True,  only_mpc = False, only_rl = True,  nom_rl = False, only_nom = True,  abs_vf = False, sensor_rl=True
    tm = TestManager(      use_nn = True,  only_mpc = False, only_rl = True,  nom_rl = False, only_nom = True,  abs_vf = False, sensor_rl=True)
    try:
        tm.run_simulations()

    except (rospy.ROSInterruptException, rospy.service.ServiceException):
        rospy.signal_shutdown("killed")