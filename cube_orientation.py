import urx
import numpy as np
import math3d as m3d
import serial
from pprint import pprint
import time
import math

#
#                                              | 3-axis
#           ________                        ___v__
#         /    1    /|             1-axis x| cube | <---- 2-axis
#        /________ / |                  ___|______|___
#        |        |  |                   |          |
#        |    2   | 3|                   |   table  |
#        |        | /                    |          |
#        |________|/
#
#                                              ___#___
# значение индикатора в нулевом положении     /       \
#                                             |  000  |
#                                             \_______/
#                                                 #
#                                                 #
#                                                 #
#                                             ____Y____
#
#
#
#   Z                   Z
#   ^                   ^
#   |                   |
#   |                   |
#   |______ X           |________ Y
#    \       |           \        ^
#     \      V            \       |
#      \ Y                 \ X   /
#
#    LEFT(clock)     RIGHT(counter clock)
'''
|   get_three_points_from_robot -> 3 points
|   get_plane_from_three_points -> plane
|       stand_perpendicular
|       check_cross_direction
|   origin = intersection_of_three_planes
        shift_point(origin, cube_edge)
|   new_csys = new_from_xyp(x_vec, y_vec, origin)
|   
|   3dMid - work
|
V
'''

axis_to_number_dict = {
    'x': 0,
    'y': 1,
    'z': 2
}


class CubeOrigin:

    ROBOT = None
    TOOL_AXIS = None
    AXIS_DIRECTION = None
    COM_PORT = None
    INDICATOR_ORIGIN = None
    CUBE_EDGE = None

    def __init__(self, robot, com_port, indicator, tcp_xyz, cube_edge, axis='x'):
        """
        ==========FIRST OF ALL USE IT==========
        @robot - Robot object or ip to robot
        @axis - one of string values: x, y, -x, -y
        @com_port - Serial object or com number
        @indicator - default indicator value
        """
        if isinstance(robot, urx.Robot):
            self.ROBOT = robot
        else:
            self.ROBOT = urx.Robot(robot)

        if isinstance(com_port, serial.Serial):
            self.COM_PORT = com_port
        else:
            pass
            # self.COM_PORT = serial.Serial(com_port,
            #                               baudrate=9600,
            #                               bytesize=serial.EIGHTBITS,
            #                               stopbits=serial.STOPBITS_ONE,
            #                               parity=serial.PARITY_NONE)

        self.CUBE_EDGE = cube_edge

        # use angle if tool did not co-directed head Z-axis
        angle = [0, 0, 0]
        if axis.startswith('-'):
            self.AXIS_DIRECTION = -1
            self.TOOL_AXIS = axis[1:]
            angle[axis_to_number_dict[self.TOOL_AXIS]] = -1.57
        else:
            self.AXIS_DIRECTION = 1
            self.TOOL_AXIS = axis
            angle[axis_to_number_dict[self.TOOL_AXIS]] = 1.57

        tcp = tuple(tcp_xyz[:3] + angle)
        # self.ROBOT.set_tcp(tcp)
        # self.ROBOT.set_tcp(tuple(tcp_xyz))
        # default indicator value
        self.INDICATOR_ORIGIN = indicator

    def get_three_points_from_robot(self):
        input('Set first point on plane and press Enter')
        p1 = self.correct_point()
        input('Set second point on plane and press Enter')
        p2 = self.correct_point()
        input('Set third point on plane and press Enter')
        p3 = self.correct_point()

        return [p1, p2, p3]

    def get_indicator_data(self):
        if self.COM_PORT.is_open:
            try:
                self.COM_PORT.write(b'1\r\n')
                time.sleep(0.1)
                indicator = self.COM_PORT.read_all().decode('utf-8')[3:]
                print(indicator)
                data = float(indicator)
                return data
            except Exception as e:
                print(e)
                return None
        else:
            raise Exception('Serial port is unreachable')

    def stand_perpendicular(self, plane):
        point = self.ROBOT.getl()
        vector = plane[:3]
        pose = point[:3] + euler_to_axis_angle(vec_to_euler(vector))
        self.ROBOT.movel(pose, vel=0.1)

    def check_cross_direction(self, vector):
        pose = [0, 0, 0]
        v = normalize_vector(vector)
        # self.ROBOT.set_orientation(v)
        self.stand_perpendicular(vector)
        start_val = self.get_indicator_data()

        pose[axis_to_number_dict[self.TOOL_AXIS]] = self.AXIS_DIRECTION * 0.005
        self.ROBOT.translate_tool(pose)

        finish_val = self.get_indicator_data()

        if start_val >= finish_val:
            return inverse_vector(vector)
        else:
            return vector

    def correct_point(self):
        epsilon = 0.00005
        pose = [0, 0, 0]
        ind_data = self.get_indicator_data() / 1000
        delta = self.INDICATOR_ORIGIN - ind_data

        if abs(delta) > epsilon:
            pose[axis_to_number_dict[self.TOOL_AXIS]] = self.AXIS_DIRECTION * delta
            self.ROBOT.translate_tool(pose)

        point = self.ROBOT.getl()[:3]

        return point

########################################################################################################################


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


if __name__ == '__main__':
    rob = urx.Robot('192.168.0.104')
    # rob = urx.Robot('192.168.88.31')
    edge = 112.71
    y = -50/1000
    x = 0
    # 8.1 + 25.1
    z = 32.5/1000
    com = 'COM3'

    co = CubeOrigin(robot=rob, com_port=com, cube_edge=edge, indicator=0, tcp_xyz=[x, y, z, 0, 0, 0], axis='x')
    # print(co.get_indicator_data())
    # print(co.get_three_points_from_robot())
    points = [[-0.5397534658123233, -0.07316723152414491, -0.21604788134441955],
              [-0.5401918509412711, -0.07307908111430718, -0.20862568142831056],
              [-0.519618808313082, -0.034009984244198155, -0.22490940994970718]]
    plane = get_plane_from_three_points(points[0], points[1], points[2])
    # plane = [-0.0002914140648546244, 0.00014555869084144815, -1.8940833208560068e-05, -0.00015073375194420046]
    # co.check_cross_direction(plane[:3])
    # co.ROBOT.movel(points[2]+[0,0,0], vel=1)
    co.stand_perpendicular(plane)
    # co.COM_PORT.close()
    co.ROBOT.close()
