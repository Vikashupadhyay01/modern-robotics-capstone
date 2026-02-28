import numpy as np
import modern_robotics as mr
import matplotlib.pyplot as plt
from trajectory_generator import generate_trajectory
from feedback_control import feedback_control

dt = 0.01
Kp = np.eye(6) * 5
Ki = np.eye(6) * 0.5

Tse_initial = np.eye(4)
Tsc_initial = np.array([[1,0,0,1],
                         [0,1,0,0],
                         [0,0,1,0.025],
                         [0,0,0,1]])

Tsc_final = np.array([[0,1,0,0],
                       [-1,0,0,-1],
                       [0,0,1,0.025],
                       [0,0,0,1]])

Tce_grasp = np.array([[0,0,1,0],
                       [0,1,0,0],
                       [-1,0,0,0],
                       [0,0,0,1]])

Tce_standoff = np.array([[0,0,1,0],
                          [0,1,0,0],
                          [-1,0,0,0.1],
                          [0,0,0,1]])

traj = generate_trajectory(Tse_initial, Tsc_initial, Tsc_final, Tce_grasp, Tce_standoff, 1)

integral_error = np.zeros(6)
error_log = []

for i in range(len(traj)-1):
    X = traj[i]
    Xd = traj[i]
    Xd_next = traj[i+1]

    V, Xerr, integral_error = feedback_control(X, Xd, Xd_next, Kp, Ki, dt, integral_error)

    error_log.append(Xerr)

error_log = np.array(error_log)

np.savetxt("results.csv", error_log, delimiter=",")

plt.plot(error_log)
plt.title("End Effector Tracking Error")
plt.xlabel("Time Steps")
plt.ylabel("Error")
plt.grid()
plt.savefig("error_plot.png")
plt.show()
