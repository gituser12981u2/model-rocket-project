import numpy as np


def quaternion_to_dcm(q):
    """Convert quaternion to direction cosine matrix."""
    q = np.asarray(q, dtype=float)
    q0, q1, q2, q3 = q[0], q[1], q[2], q[3]  # Extract as scalars
    dcm = np.array(
        [
            [1 - 2 * (q2**2 + q3**2), 2 * (q1 * q2 - q0 * q3), 2 * (q1 * q3 + q0 * q2)],
            [2 * (q1 * q2 + q0 * q3), 1 - 2 * (q1**2 + q3**2), 2 * (q2 * q3 - q0 * q1)],
            [2 * (q1 * q3 - q0 * q2), 2 * (q2 * q3 + q0 * q1), 1 - 2 * (q1**2 + q2**2)],
        ]
    )
    return dcm


def quaternion_derivative(q, omega):
    """Compute quaternion derivative from angular velocity."""
    q = np.asarray(q, dtype=float)

    omega = np.asarray(omega, dtype=float)
    q0, q1, q2, q3 = q[0], q[1], q[2], q[3]  # Extract as scalars
    wx, wy, wz = omega[0], omega[1], omega[2]  # Extract as scalars
    q_dot = 0.5 * np.array(
        [
            -q1 * wx - q2 * wy - q3 * wz,
            q0 * wx + q2 * wz - q3 * wy,
            q0 * wy + q3 * wx - q1 * wz,
            q0 * wz + q1 * wy - q2 * wx,
        ]
    )
    return q_dot
