import math
import urx
import reader as rd
import math3d as m3d
import time


def initialize_robot(rob, file_path):

    tcp = (0, 0, 0.066, 0, 0, 0)
    rob.set_tcp(tcp)
    # my_csys = rob.new_csys_from_xpy()
    # print(my_csys)

    my_csys = m3d.Transform(
                             7.21748314e-01,  -6.92100474e-01,  -8.73532598e-03,
                             6.92132446e-01,   7.21769916e-01,   9.30157231e-04,
                             5.66113324e-03,  -6.71734195e-03,   9.99961414e-01,
                            -0.50301,         -0.27713,         -0.21144
    )
    rob.set_csys(my_csys)
    obj_list = rd.read_file(file_path)
    pose_processing(rob, obj_list)


def pose_processing(rob, obj_list):
    vectors = obj_list.copy()
    del vectors['sphere']
    sphere = obj_list['sphere']

    start_pose = (0, 0, (float(sphere['radius']) + float(sphere['center'][2])), math.pi, 0, 0)
    # start_pose = (0, 0, 0.05, math.pi, 0, 0)

    rob.movel(start_pose, vel=0.2)

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

        move_to_start(rob, pose_1, sphere['radius'])
        # move to stop
        rob.movel(pose_2)
        # do_something_useful()
        time.sleep(1)
        # move back to start
        rob.movel(pose_1)

    rob.movel(start_pose, vel=0.1)


def move_to_start(rob, pose_to, radius):

    pose_from = rob.getl()

    x_via = (pose_from[0] + pose_to[0])/2
    y_via = (pose_from[1] + pose_to[1])/2
    z_via = math.sqrt(radius**2 - x_via**2 - y_via**2)

    pose_via = (x_via, y_via, z_via, pose_to[3], pose_to[4], pose_to[5])
    rob.movec(pose_via, pose_to)


if __name__ == '__main__':
    # ip = '192.168.88.30'
    # ip = '192.168.0.103'
    ip = '192.168.88.31'
    file_path = './points.txt'
    rob = urx.Robot(ip)
    initialize_robot(rob, file_path)
    rob.close()

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