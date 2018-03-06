import math
import urx
import reader as rd
from linal import rotation_matrix, concatenate_matrices, rotation_matrix_to_axis_angle


def initialize_robot(rob, file_path):
    # настройка СК
    # настройка тсп для пера !надеюсь в мм!
    tcp = (0, 0, 66, 0, 0, 0)
    rob.set_tcp()
    # rob.set_csys(rob.new_csys_from_xpy())
    obj_list = rd.read_file(file_path)
    pose_processing(rob, obj_list)


def pose_processing(rob, obj_list):
    vectors = obj_list.copy()
    del vectors['sphere']
    sphere = obj_list['sphere']

    start_pose = (0, 0, (float(sphere['radius']) + float(sphere['center'][2])) / 1000 - 2, math.pi, 0, 0)
    rob.movel(start_pose)

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

        move_to_start(rob, pose_1)
        # move to stop
        rob.movel(pose_2)
        # do_something_useful()
        # move back to start
        rob.movel(pose_1)


def move_to_start(rob, pose_to):

    pose_from = rob.getl()
    # r = math.sqrt(pose_to[0] ** 2 + pose_to[1] ** 2 + pose_to[2] ** 2)
    #
    # theta_start = math.atan2(pose_from[1], pose_from[0])
    # phi_start = math.acos(pose_from[2] / r)
    #
    # theta_end = math.atan2(pose_to[1], pose_to[0])
    # phi_end = math.acos(pose_to[2] / r)
    #
    # theta_via = (theta_start + theta_end) / 2
    # phi_via = (phi_start + phi_end) / 2
    #
    # x_via = r * math.sin(theta_via) * math.cos(phi_via)
    # y_via = r * math.sin(theta_via) * math.sin(phi_via)
    # z_via = r * math.cos(theta_via)

    x_via = (pose_from[0] + pose_to[0])/2
    y_via = (pose_from[1] + pose_to[1])/2
    z_via = (pose_from[2] + pose_to[2])/2

    pose_via = (x_via, y_via, z_via, pose_to[3], pose_to[4], pose_to[5])
    rob.movec(pose_via, pose_to)


if __name__ == '__main__':
    # ip = '192.168.88.35'
    ip = '192.168.0.106'
    file_path = './points.txt'
    rob = urx.Robot(ip)
    initialize_robot(rob, file_path)
    rob.close()
