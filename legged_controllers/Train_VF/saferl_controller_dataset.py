from __future__ import print_function
import rospy
from utils.dataset_manager import DatasetManager
import numpy as np
import os
import time

np.set_printoptions(threshold=np.inf, precision=5, linewidth=1000, suppress=True)


if __name__ == '__main__':
    dm = DatasetManager(use_nn = True)
    try:
        dm.run_batch_simulations(n_episodes=100, save_path_relative="observation_datasets", noise_std=10.0, seed = int(time.time()))

    except (rospy.ROSInterruptException, rospy.service.ServiceException):
        rospy.signal_shutdown("killed")

    