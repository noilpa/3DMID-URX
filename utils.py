import math
import numpy as np
import serial


axis_to_number_dict = {
    'x': 0,
    'y': 1,
    'z': 2
}


def vec_to_axis_angle(vector):
    return euler_to_axis_angle(vec_to_euler(vector))


def vec_to_euler(vector):
    v = normalize_vector(vector[:3])
    euler = [
        math.acos(v[2]),
        math.acos((v[1]**2 + v[2]**2)**0.5),
        0
    ]
    return euler


def euler_to_axis_angle(euler: list):
    c1 = math.cos(euler[0])
    s1 = math.sin(euler[0])
    c2 = math.cos(euler[1])
    s2 = math.sin(euler[1])
    c3 = math.cos(euler[2])
    s3 = math.sin(euler[2])

    c1_c2 = c1*c2
    s1_s2 = s1*s2

    w = c1_c2 * c3 - s1_s2 * s3
    x = c1_c2 * s3 + s1_s2 * c3
    y = s1 * c2 * c3 + c1 * s2 * s3
    z = c1 * s2 * c3 - s1 * c2 * s3

    angle = 2 * math.acos(w)
    norm = x**2 + y**2 + z**2

    if norm < 0.001:
        x = 1
        y = z = 0
        return [x*angle, y*angle, z*angle]
    else:
        norm = norm**0.5

    return [x*angle/norm, y*angle/norm, z*angle/norm]


def shift_point(point, shift):
    return [x + y for x, y in zip(point, shift)]


# x<->0, y<->1, z<->2
def get_plane_from_three_points(p1, p2, p3):
    v1 = vector(p1, p2)
    v2 = vector(p1, p3)
    n = cross(v1, v2)

    d = n[0] * (-p1[0]) + n[1] * (-p1[1]) + n[2] * (-p1[2])

    # plane = Ax + By + Cz + D , n = (A, B, C)
    plane = [n[0], n[1], n[2], d]

    return plane


def vector_length(vector):
    return (vector[0] ** 2 + vector[1] ** 2 + vector[2] ** 2) ** 0.5


def normalize_vector(vector):
    length = vector_length(vector)
    if length == 1:
        return vector
    v = [None] * 3
    v[0] = vector[0] / length
    v[1] = vector[1] / length
    v[2] = vector[2] / length
    return v


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


# vec a and vec b
def cross(a, b):
    c = [None] * 3
    for i in range(3):
        c[i] = a[(i + 1) % 3] * b[(i + 2) % 3] - a[(i + 2) % 3] * b[(i + 1) % 3]
    return c


def com_ports():
    result = []
    ports = ['COM{}'.format(i) for i in range(256)]

    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except(Exception):
            pass

    return result


def test():
    pl_1 = [[8.89764, 20.00000, 100.00000], [78.89764, 20.00000, 100.00000], [69.43473, 84.00000, 100.00000]]
    pl_2 = [[23.29630, 0.00000, 80.53385], [79.29630, 0.00000, 76.53385], [24.00000, 0.00000, 46.27756]]
    pl_3 = [[0.00000, 80.00000, 77.82721], [0.00000, 32.00000, 71.82721], [0.00000, 57.89716, 30.00000]]

    points = [pl_1, pl_2, pl_3]
    planes = [None] * 3

    for i, item in enumerate(points):
        planes[i] = get_plane_from_three_points(item[0], item[1], item[2])

    origin = intersection_of_three_planes(planes[0], planes[1], planes[2])
    pprint(planes)
    pprint(origin)