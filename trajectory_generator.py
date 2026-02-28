import numpy as np
import modern_robotics as mr

def generate_trajectory(Tse_initial, Tsc_initial, Tsc_final, Tce_grasp, Tce_standoff, k):
    """
    Generates reference trajectory for the mobile manipulator.
    """

    traj = []

    traj1 = mr.ScrewTrajectory(Tse_initial, Tsc_initial @ Tce_standoff, 4, 4*k, 3)
    traj2 = mr.ScrewTrajectory(Tsc_initial @ Tce_standoff, Tsc_initial @ Tce_grasp, 1, k, 3)
    traj3 = mr.ScrewTrajectory(Tsc_initial @ Tce_grasp, Tsc_initial @ Tce_grasp, 1, k, 3)
    traj4 = mr.ScrewTrajectory(Tsc_initial @ Tce_grasp, Tsc_initial @ Tce_standoff, 1, k, 3)
    traj5 = mr.ScrewTrajectory(Tsc_initial @ Tce_standoff, Tsc_final @ Tce_standoff, 4, 4*k, 3)
    traj6 = mr.ScrewTrajectory(Tsc_final @ Tce_standoff, Tsc_final @ Tce_grasp, 1, k, 3)
    traj7 = mr.ScrewTrajectory(Tsc_final @ Tce_grasp, Tsc_final @ Tce_grasp, 1, k, 3)
    traj8 = mr.ScrewTrajectory(Tsc_final @ Tce_grasp, Tsc_final @ Tce_standoff, 1, k, 3)

    traj = traj1 + traj2 + traj3 + traj4 + traj5 + traj6 + traj7 + traj8

    return traj
