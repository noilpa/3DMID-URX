import reader as rd
import math
import urx
import math3d as m3d
from math3d import Vector
from math3d import Orientation
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
    MODEL_HEIGHT = None
    FILE = None

    def __init__(self, robot, com_port, indicator, tcp, points_filepath, axis='x'):
        """
        ==========FIRST OF ALL USE IT==========
        @robot - Robot object or ip to robot
        @axis - indicator direction, one of string values: x, y, -x, -y
        @com_port - Serial object or com number
        @indicator - default indicator value
        """
        if isinstance(robot, urx.Robot):
            self.ROBOT = robot
        else:
            self.ROBOT = urx.Robot(robot)

        if isinstance(com_port, serial.Serial):
            self.COM_PORT = com_port
        elif com_port is None:
            pass
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

    def __get_three_points_from_robot(self):
        input('Set first point on plane and press Enter')
        p1 = self.__correct_point()
        input('Set second point on plane and press Enter')
        p2 = self.__correct_point()
        input('Set third point on plane and press Enter')
        p3 = self.__correct_point()

        return [p1, p2, p3]

    def __get_indicator_data(self):
        if self.COM_PORT.is_open:
            try:
                self.COM_PORT.write(b'1\r\n')
                time.sleep(0.2)
                indicator = self.COM_PORT.read_all().decode('utf-8')[3:]
                print(indicator)
                data = float(indicator)
                return data
            except Exception as e:
                print(e)
                return None
        else:
            raise Exception('Serial port is unreachable')

    def __stand_perpendicular(self, plane):
        # TODO: rework this method without moving robot
        # point = self.ROBOT.getl()
        vector = Vector(plane[:3])
        self.set_tool_orientation(orient_from_vector(vector))
        # pose = point[:3] + rx_ry_rz_from_vector(vector)
        # self.ROBOT.movel(pose, vel=0.1)

    def __check_cross_direction(self, vector: Vector):
        # TODO: dot product of tool vector and plane vector must be > 0 => codirected
        v = vector.normalized
        self.__stand_perpendicular(vector)
        # tool = self.ROBOT.get_orientation()

        if np.dot(v, tool) < 0:
            return inverse_vector(v)
        else:
            return v

    def __correct_point(self):
        epsilon = 0.00005
        pose = [0, 0, 0]
        ind_data = self.__get_indicator_data() / 1000
        delta = self.INDICATOR_ORIGIN - ind_data

        if abs(delta) > epsilon:
            pose[axis_to_number_dict[self.TOOL_AXIS]] = self.AXIS_DIRECTION * delta
            self.ROBOT.translate_tool(pose)

        point = self.ROBOT.getl()[:3]

        return point

    def __move_to_start(self, pose_to, radius):

        pose_from = self.ROBOT.getl()

        x_via = (pose_from[0] + pose_to[0]) / 2
        y_via = (pose_from[1] + pose_to[1]) / 2
        a = radius ** 2 - x_via ** 2 - y_via ** 2
        if a > 0:
            z_via = math.sqrt(a)
        else:
            z_via = -math.sqrt(-a)

        pose_via = (x_via, y_via, z_via, pose_to[3], pose_to[4], pose_to[5])
        # self.ROBOT.movec(pose_via, pose_to)
        self.ROBOT.movep(pose_to, vel=2)

    def __new_csys_from_planes(self):
        """
        first plane define Z axis
        second plane define X axis
        third plane define Y axis

        Z directed outside plane
        X and Y axis directed inside plane

        """
        planes = []
        for i in range(3):
            p1, p2, p3 = self.__get_three_points_from_robot()
            planes.append(get_plane_from_three_points(p1, p2, p3))
            self.__check_cross_direction(planes[i])

        orig = intersection_of_three_planes(planes[0], planes[1], planes[2])

        orig[2] -= self.MODEL_HEIGHT
        vec_x = m3d.Vector(planes[1][:3])
        vec_y = m3d.Vector(planes[2][:3])

        csys = m3d.Transform.new_from_xyp(vec_x, vec_y, orig)

        self.ROBOT.set_csys(csys)

    def pose_processing(self):

        vectors = self.FILE.copy()
        del vectors['sphere']
        sphere = self.FILE['sphere']

        start_pose = (0, 0, (float(sphere['radius']) + float(sphere['center'][2])), math.pi, 0, 0)
        try:
            self.ROBOT.movel(start_pose, vel=2)
        except urx.RobotException:
            pass

        input('Press any key to start...')

        for key in vectors:
            x1, y1, z1 = [float(i) for i in vectors[key]['start']]
            x2, y2, z2 = [float(i) for i in vectors[key]['stop']]
            vx, vy, vz = [float(i) for i in vectors[key]['direction']]

            vec_z = m3d.Vector(vx, vy, vz)
            orient = orient_from_vector(-vec_z)

            pose_1 = (x1, y1, z1, orient.rotation_vector[0], orient.rotation_vector[1], orient.rotation_vector[2])
            pose_2 = (x2, y2, z2, orient.rotation_vector[0], orient.rotation_vector[1], orient.rotation_vector[2])

            try:
                self.__move_to_start(pose_2, sphere['radius'])
            except urx.RobotException:
                pass
            # move to stop
            try:
                self.ROBOT.movel(pose_1, vel=2)
            except urx.RobotException:
                pass
            # do_something_useful()
            time.sleep(1)
            # move back to start
            try:
                self.ROBOT.movel(pose_2, vel=2)
            except urx.RobotException:
                pass

        self.ROBOT.movel(start_pose, vel=2)

    def new_rob_csys(self, mode):
        modes = {
            'points': self.ROBOT.new_csys_from_xpy,
            'planes': self.__new_csys_from_planes
        }
        if mode not in modes.keys():
            raise NameError('wrong mode')
        else:
            modes[mode]()

    def set_tool_orientation(self, vector: Vector):
        self.ROBOT.set_orientation(orient_from_vector(vector))


def init_stage(c: Controller, points=[]):
    """
    Prepare for pose_processing()
    :param c:
    :param points: [ [[p1], [p2], [p3]], [[p1], [p2], [p3]], [[p1], [p2], [p3]] ]
    :return: new csys
    """
    if len(points) == 0:
        for i in range(3):
            points.append(c.__get_three_points_from_robot())

    planes = []
    for i, p in enumerate(points):
        planes.append(get_plane_from_three_points(p[0], p[1], p[2]))
    print(planes)


if __name__ == '__main__':
    # rob = urx.Robot('192.168.0.105')
    rob = urx.Robot('192.168.88.21')
    # edge = 112.71
    y = 50 / 1000
    x = 0
    z = 34.3 / 1000
    com = None  # 'COM3'
    co = Controller(robot=rob,
                    com_port=com,
                    indicator=0,
                    tcp=[x, y, z, 0, 0, 0],
                    points_filepath='./points.txt',
                    axis='z')

    # co.ROBOT.new_csys_from_xpy()

    px = m3d.Vector([-0.43486041640101686, -0.13881540123664282, -0.21053820546042168])
    p0 = m3d.Vector([-0.5198672251276444, -0.21984341325034076, -0.2118583353420153])
    py = m3d.Vector([-0.5981675648960634, -0.13921634276409914, -0.21199963026601087])

    new_csys = m3d.Transform.new_from_xyp(px - p0, py - p0, p0)

    co.ROBOT.set_csys(new_csys)

    co.pose_processing()

