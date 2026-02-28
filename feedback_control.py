import numpy as np
import modern_robotics as mr

def feedback_control(X, Xd, Xd_next, Kp, Ki, dt, integral_error):
    """
    Computes feedback + feedforward control law.
    """

    Xerr = mr.se3ToVec(mr.MatrixLog6(np.dot(mr.TransInv(X), Xd)))

    integral_error += Xerr * dt

    Vd = mr.se3ToVec((1/dt) * mr.MatrixLog6(np.dot(mr.TransInv(Xd), Xd_next)))

    Ad_term = mr.Adjoint(np.dot(mr.TransInv(X), Xd))
    V = np.dot(Ad_term, Vd) + np.dot(Kp, Xerr) + np.dot(Ki, integral_error)

    return V, Xerr, integral_error
