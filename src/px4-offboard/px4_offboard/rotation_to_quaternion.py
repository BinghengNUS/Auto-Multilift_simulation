import numpy as np


def rotation_matrix_to_quaternion(R):
    """
    Calculate the quaternion [q0, q1, q2, q3] from the rotation matrix R, where q0 = w, q1 = x, q2 = y, q3 = z.
    """
    tr = np.trace(R)

    if tr > 0:
        S = np.sqrt(1.0 + tr) * 2  # 4q0
        q0 = 0.25 * S
        q1 = (R[2, 1] - R[1, 2]) / S
        q2 = (R[0, 2] - R[2, 0]) / S
        q3 = (R[1, 0] - R[0, 1]) / S
    else:
        if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2  # 4q1
            q0 = (R[2, 1] - R[1, 2]) / S
            q1 = 0.25 * S
            q2 = (R[0, 1] + R[1, 0]) / S
            q3 = (R[0, 2] + R[2, 0]) / S
        elif R[1, 1] > R[2, 2]:
            S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2  # 4q2
            q0 = (R[0, 2] - R[2, 0]) / S
            q1 = (R[0, 1] + R[1, 0]) / S
            q2 = 0.25 * S
            q3 = (R[1, 2] + R[2, 1]) / S
        else:
            S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2  # 4q3
            q0 = (R[1, 0] - R[0, 1]) / S
            q1 = (R[0, 2] + R[2, 0]) / S
            q2 = (R[1, 2] + R[2, 1]) / S
            q3 = 0.25 * S

    return [q0, q1, q2, q3]
