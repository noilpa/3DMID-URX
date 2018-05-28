import math
import numpy as np
import serial
import pprint
import math3d as m3d
from math3d import Vector
from math3d import Orientation
import urx

axis_to_number_dict = {
    'x': 0,
    'y': 1,
    'z': 2
}


def shift_point(point, shift):
    return [x + y for x, y in zip(point, shift)]


# x<->0, y<->1, z<->2
def get_plane_from_three_points(p1, p2, p3):
    v1 = m3d.Vector(vector(p1, p2))
    v2 = m3d.Vector(vector(p1, p3))
    n = v1.cross(v2)

    d = n[0] * (-p1[0]) + n[1] * (-p1[1]) + n[2] * (-p1[2])

    # plane = Ax + By + Cz + D , n = (A, B, C)
    plane = [n[0], n[1], n[2], d]

    return plane


def inverse_vector(vector):
    v = [None] * len(vector)
    for i, item in enumerate(vector):
        v[i] = item * -1
    return v


def intersection_of_three_planes(pl_1, pl_2, pl_3):
    rc = np.array([pl_1[:3], pl_2[:3], pl_3[:3]])
    rd = np.array([pl_1, pl_2, pl_3])

    # ранк - количество строк или столбцов которые нельзя выразить через другие
    rc_rank = np.linalg.matrix_rank(rc)
    rd_rank = np.linalg.matrix_rank(rd)

    if rc_rank != 3 or rd_rank != 3:
        url = r'http://www.ambrsoft.com/TrigoCalc/Plan3D/3PlanesIntersection_.htm'
        raise Exception('Planes don\'t intersect at one point. Rc={0} Rd={1}'
                        'for more information {2}'.format(rc, rd, url))
    a = rd[:, 0]
    b = rd[:, 1]
    c = rd[:, 2]
    d = rd[:, 3]
    det = np.linalg.det(np.matrix.transpose(np.array([a, b, c])))

    x = np.linalg.det(np.matrix.transpose(np.array([d, b, c]))) / det
    y = np.linalg.det(np.matrix.transpose(np.array([a, d, c]))) / det
    z = np.linalg.det(np.matrix.transpose(np.array([a, b, d]))) / det

    return [x, y, z]


def vector(p1, p2):
    return [p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]]


def orient_from_vector(vector: Vector):
    # TODO: remove hardcode vec_z = vector
    vec_z = vector
    vec_x = get_orthogonal_vector(vec_z)
    return m3d.Orientation.new_from_xz(vec_x, vec_z)


def get_orthogonal_vector(vec):
    if vec[1] != 0:
        basis = [0, -vec[2], vec[1]]
    else:
        basis = [-vec[2], 0, vec[0]]
    # else:
    #     basis = [-vec[1], vec[0], 0]
    return m3d.Vector(basis)


