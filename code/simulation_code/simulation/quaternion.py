import numpy as np


def quaternion_to_dcm(q):
    """Convert quaternion to direction cosine matrix."""
    q0, q1, q2, q3 = q

    dcm = np.array([
        [1 - 2*(q2**2 + q3**2), 2*(q1*q2 - q0*q3), 2*(q1*q3 + q0*q2)],
        [2*(q1*q2 + q0*q3), 1 - 2*(q1**2 + q3**2), 2*(q2*q3 - q0*q1)],
        [2*(q1*q3 - q0*q2), 2*(q2*q3 + q0*q1), 1 - 2*(q1**2 + q2**2)]
    ])
    return dcm


def quaternion_derivative(q, omega):
    """Compute quaternion derivative from angular velocity."""
    q0, q1, q2, q3 = q
    wx, wy, wz = omega

    q_dot = 0.5 * np.array([
        -q1*wx - q2*wy - q3*wz,
        q0*wx + q2*wz - q3*wy,
        q0*wy + q3*wx - q1*wz,
        q0*wz + q1*wy - q2*wx
    ])
    return q_dot
