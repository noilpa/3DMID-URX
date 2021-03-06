import pprint


def __read_line(file):
    start = [float(i)/1000 for i in file.readline().strip().split(',')]
    stop = [float(i)/1000 for i in file.readline().strip().split(',')]
    direction = file.readline().strip().split(',')

    obj = {
        'start': start,
        'stop': stop,
        'direction': direction
    }
    return obj


def __read_sphere(file):
    center = [float(i)/1000 for i in file.readline().strip().split(',')]
    radius = float(file.readline().strip())/1000
    obj = {
        'center': center,
        'radius': radius,
    }
    return obj


def read_file(file_path):
    f = open(file_path, "r")
    obj_list = {}
    i = 0
    # как передавать в функцию указатель на строку в файле
    for line in f:
        if line == 'Sphere\n':
            obj_list['sphere'] = __read_sphere(f)
        elif line == 'Vector\n':
            obj_list['vector_{}'.format(i)] = __read_line(f)
            i += 1
        else:
            print('Unknown data')
    f.close()
    return obj_list


