import math
import numpy as np


def rotation_matrix(angle, axis):
    axis_dict = {
        'x_axis': [1, 0, 0],
        'y_axis': [0, 1, 0],
        'z_axis': [0, 0, 1]
    }
    direction = axis_dict[axis]
    sin_a = math.sin(angle)
    cos_a = math.cos(angle)

    R = np.diag([cos_a, cos_a, cos_a])
    R += np.outer(direction, direction) * (1.0 - cos_a)
    temp = []
    for i in direction:
        temp.append(float(i) * sin_a)
    direction = temp
    R += np.array([[0.0, -direction[2], direction[1]],
                   [direction[2], 0.0, -direction[0]],
                   [-direction[1], direction[0], 0.0]])
    M = np.identity(4)
    M[:3, :3] = R
    return M


def concatenate_matrices(*matrices):
    m = np.identity(4)
    for i in matrices:
        m = np.dot(m, i)
    return m


# algorithm' source:
# http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToAngle/index.htm
def rotation_matrix_to_axis_angle(rm):
    epsilon = 0.01
    epsilon_2 = 0.1
    if (
                        math.fabs(rm[0][1] - rm[1][0]) < epsilon and
                        math.fabs(rm[0][2] - rm[2][0]) < epsilon and
                    math.fabs(rm[1][2] - rm[2][1]) < epsilon
    ):
        if (
                                math.fabs(rm[0][1] + rm[1][0]) < epsilon_2 and
                                math.fabs(rm[0][2] + rm[2][0]) < epsilon_2 and
                            math.fabs(rm[1][2] + rm[2][1]) < epsilon_2 and
                    math.fabs(rm[0][0] + rm[1][1] + rm[2][2] - 3 < epsilon_2)
        ):
            # zero angle, arbitrary axis
            return 0, 1, 0, 0
        angle = math.pi
        xx = (rm[0][0] + 1) / 2
        yy = (rm[1][1] + 1) / 2
        zz = (rm[2][2] + 1) / 2
        xy = (rm[0][1] + rm[1][0]) / 4
        xz = (rm[0][2] + rm[2][0]) / 4
        yz = (rm[1][2] + rm[2][1]) / 4

        if xx > yy and xx > zz:
            if xx < epsilon:
                x = 0
                y = 0.7071
                z = 0.7071
            else:
                x = math.sqrt(xx)
                y = xy / x
                z = xz / x
        elif yy > zz:
            if yy < epsilon:
                x = 0.7071
                y = 0
                z = 0.7071
            else:
                y = math.sqrt(yy)
                x = xy / y
                z = yz / y
        else:
            if zz < epsilon:
                x = 0.7071
                y = 0.7071
                z = 0
            else:
                z = math.sqrt(zz)
                x = xz / z
                y = yz / z
        return angle, x, y, z

    # default case when all good
    angle = math.acos((rm[0][0] + rm[1][1] + rm[2][2] - 1) / 2)
    s = math.sqrt((rm[2][1] - rm[1][2]) ** 2 + (rm[0][2] - rm[2][0]) ** 2 + (rm[1][0] - rm[0][1]) ** 2)
    if math.fabs(s) < 0.001:
        s = 1
    x = (rm[2][1] - rm[1][2]) / s
    y = (rm[0][2] - rm[2][0]) / s
    z = (rm[1][0] - rm[0][1]) / s
    return angle, x, y, z


def is_rotation_matrix(r):
    rt = np.transpose(r)
    should_be_identity = np.dot(rt, r)
    identity = np.identity(3, dtype=r.dtype)
    n = np.linalg.norm(identity - should_be_identity)
    return n < 1e-6
