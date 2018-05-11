import reader as rd
import urx
import time
from utils import *

# -----------=====TRASH=====-----------
# def initialize_robot(rob, file_path):
#
#     tcp = (0, 0, 0.066, 0, 0, 0)
#     rob.set_tcp(tcp)
#     # my_csys = rob.new_csys_from_xpy()
#     # print(my_csys)
#
#     # my_csys = m3d.Transform(
#     #                          7.21748314e-01,  -6.92100474e-01,  -8.73532598e-03,
#     #                          6.92132446e-01,   7.21769916e-01,   9.30157231e-04,
#     #                          5.66113324e-03,  -6.71734195e-03,   9.99961414e-01,
#     #                         -0.50301,         -0.27713,         -0.21144
#     # )
#     # rob.set_csys(my_csys)
#     obj_list = rd.read_file(file_path)
#     pose_processing(rob, obj_list)

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


class Controller:

    ROBOT = None
    TOOL_AXIS = None
    AXIS_DIRECTION = None
    COM_PORT = None
    INDICATOR_ORIGIN = None
    CUBE_EDGE = None
    FILE = None

    def __init__(self, robot, com_port, indicator, tcp, points_filepath, axis='x'):
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
            self.COM_PORT = serial.Serial(com_port,
                                          baudrate=9600,
                                          bytesize=serial.EIGHTBITS,
                                          stopbits=serial.STOPBITS_ONE,
                                          parity=serial.PARITY_NONE)

        self.FILE = rd.read_file(points_filepath)

        if axis.startswith('-'):
            self.AXIS_DIRECTION = -1
            self.TOOL_AXIS = axis[1:]
        else:
            self.AXIS_DIRECTION = 1
            self.TOOL_AXIS = axis

        self.ROBOT.set_tcp(tcp)
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

    def move_to_start(self, pose_to, radius):

        pose_from = self.ROBOT.getl()

        x_via = (pose_from[0] + pose_to[0]) / 2
        y_via = (pose_from[1] + pose_to[1]) / 2
        z_via = math.sqrt(radius ** 2 - x_via ** 2 - y_via ** 2)

        pose_via = (x_via, y_via, z_via, pose_to[3], pose_to[4], pose_to[5])
        self.ROBOT.movec(pose_via, pose_to)

    def pose_processing(self):
        """
        TODO: use vec_to_axis_angle for RX, RY, RZ components
        """
        vectors = self.FILE.copy()
        del vectors['sphere']
        sphere = self.FILE['sphere']

        start_pose = (0, 0, (float(sphere['radius']) + float(sphere['center'][2])), math.pi, 0, 0)
        # start_pose = (0, 0, 0.05, math.pi, 0, 0)

        self.ROBOT.movel(start_pose, vel=0.2)

        input('Press any key to start...')

        for key in vectors:
            x1, y1, z1 = [float(i) for i in vectors[key]['start']]
            x2, y2, z2 = [float(i) for i in vectors[key]['stop']]
            vx, vy, vz = [float(i) for i in vectors[key]['direction']]

            # rx_ang = math.atan2(vz, vy)
            # ry_ang = math.atan2(vx, vz)
            # rz_ang = math.atan2(vy, vx)
            #
            # mx = rotation_matrix(rx_ang, 'x_axis')
            # my = rotation_matrix(ry_ang, 'y_axis')
            # mz = rotation_matrix(rz_ang, 'z_axis')
            # general_rotation_matrix = concatenate_matrices(mx, my, mz)
            #
            # angle, x, y, z = rotation_matrix_to_axis_angle(general_rotation_matrix)
            #
            # rx, ry, rz = x * angle, y * angle, z * angle

            pose_1 = (x1, y1, z1, vx, vy, vz)
            pose_2 = (x2, y2, z2, vx, vy, vz)

            self.move_to_start(pose_1, sphere['radius'])
            # move to stop
            self.ROBOT.movel(pose_2)
            # do_something_useful()
            time.sleep(1)
            # move back to start
            self.ROBOT.movel(pose_1)

        self.ROBOT.movel(start_pose, vel=0.2)


if __name__ == '__main__':
    pass
    # ip = '192.168.88.30'
    # ip = '192.168.0.103'
    ########
    # ip = '192.168.88.31'
    # file_path = './points.txt'
    # rob = urx.Robot(ip)
    # initialize_robot(rob, file_path)
    # rob.close()
    ########

"""

import urx
import math3d as m3d
rob = urx.Robot('192.168.88.31')
tcp = (0, 0, 0.06, 0, 0, 0)
rob.set_tcp(tcp)
my_csys = m3d.Transform(
                             7.21748314e-01,  -6.92100474e-01,  -8.73532598e-03,
                             6.92132446e-01,   7.21769916e-01,   9.30157231e-04,
                             5.66113324e-03,  -6.71734195e-03,   9.99961414e-01,
                            -0.50301,         -0.27713,         -0.21144
    )
rob.set_csys(my_csys)

То что надо:
<Orientation: 
array([[  6.97180551e-01,   7.16880517e-01,  -4.64800784e-03],
       [ -7.16885748e-01,   6.97190296e-01,   7.18386569e-04],
       [  3.75554330e-03,   2.83124543e-03,   9.99988940e-01]])>
<Vector: (-0.64536, -0.11416, -0.21945)>
>


<Transform:
<Orientation: 
array([[ 0.7226265 , -0.6911988 ,  0.00742756],
       [ 0.69123379,  0.72262015, -0.00399496],
       [-0.00260599,  0.00802104,  0.99996444]])>
<Vector: (-0.50305, -0.27659, -0.21213)>
>

"""