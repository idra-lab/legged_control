#!/usr/bin/env python3
"""Plot the .npy dumps produced by opti_pessi_interface_mpc.

Reads the same config/*.info files the C++ node reads, so there is a single source of truth for
the goal, the start pose and the horizon. Run it from the directory holding the .npy dumps:

    plot_mpc.py <taskFile> <scenarioFile>
"""
import sys
from pathlib import Path
import os

import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import plot_utils as plt_ut  # noqa: E402

from info_parser import load_info  # noqa: E402


def load_obstacles(path):
    y = np.load(path)
    return y[None, ...] if y.ndim == 2 else y


def main(argv):
    if len(argv) < 2:
        print(__doc__)
        return 1

    task = load_info("src/legged_control/opti_pessi_interface/config/task.info")
    scen = "src/legged_control/opti_pessi_interface/config/scenario_S"
    ario = argv[1]
    scenario = load_info(scen + ario + ".info")
    # scenario = load_info(argv[2])

    horizon = int(task["horizon"]["N"])
    start = np.array([float(task["initialState"]["(0,0)"]), float(task["initialState"]["(1,0)"])])
    goal = np.array([float(scenario["goal"]["(0,0)"]), float(scenario["goal"]["(1,0)"])])
    fig_name = scenario["simulation"]["figName"].strip('"')
    save_results = True
    save_dt = False
    clean_npy_files = True

    suffix = fig_name + "_opti_pessi.npy"
    X = np.load("x_quad_" + suffix)
    U = np.load("u_" + suffix)
    obstacles = load_obstacles("y_obs_" + suffix)

    dt = U[5, :]
    steps = dt.size
    num_obstacles = obstacles.shape[0]

    # Contact-phase durations.
    plt.figure()
    plt.plot(dt, "b", label="Opti-Pessi MPC")
    plt.yticks(0.1 * np.arange(6))
    plt.locator_params(axis="x", integer=True)
    plt.xlabel("Simulation step")
    plt.ylabel("Time [s]")
    plt.grid(True)
    plt.legend()
    if save_results and save_dt:
        plt_ut.saveFigure(fig_name + "dt")

    # Closed-loop trajectory, fading in with simulation time.
    plt.figure()
    plt.title("Opti-Pessi MPC, N = %d, sim_T = %.2f s" % (horizon, dt.sum()))
    fade = (1 - 0.4) / max(steps, 1)
    for i in range(steps):
        alpha = 0.2 + fade * (i + 1)
        plt.plot(X[0, i : i + 2], X[1, i : i + 2], "-c", alpha=alpha, marker=".")
        for j in range(num_obstacles):
            plt.plot(obstacles[j, 0, i : i + 2], obstacles[j, 1, i : i + 2], "-r", alpha=alpha, marker=".")
        if i == steps - 1:
            plt.plot(X[0, i : i + 2], X[1, i : i + 2], "-c", label="Opti-Pessi MPC", alpha=alpha)
            for j in range(num_obstacles):
                plt.plot(obstacles[j, 0, i : i + 2], obstacles[j, 1, i : i + 2], "-r", label="Obstacle", alpha=alpha)
    for j in range(num_obstacles):
        plt.plot(obstacles[j, 0, -1], obstacles[j, 1, -1], "-or")
        plt.plot(obstacles[j, 0, 0], obstacles[j, 1, 0], "sr")
    plt.plot(X[0, -1], X[1, -1], "-oc")
    plt.plot(start[0], start[1], "sb", label="Start")
    plt.plot(goal[0], goal[1], "vb", label="Goal")
    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.grid(True)
    plt.gca().set_aspect("equal")
    plt.legend()
    if save_results:
        plt_ut.saveFigure(fig_name + "opti_pessi")
    # plt.show()
    
    if clean_npy_files:
        x_file = "x_quad_" + suffix
        u_file = "u_" + suffix
        y_file = "y_obs_" + suffix
        npy_files_to_remove = [x_file, u_file, y_file]
        for f in npy_files_to_remove:
            if os.path.exists(f):
                os.remove(f)
                print(f"Rimosso file: {f}")

    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
