import pprint


def __read_line(file):
    start = file.readline().strip().split(',')
    stop = file.readline().strip().split(',')
    direction = file.readline().strip().split(',')

    obj = {
        'start': start,
        'stop': stop,
        'direction': direction
    }
    return obj


def __read_sphere(file):
    center = file.readline().strip().split(',')
    radius = file.readline().strip()
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
#        else:
#            print('Unknown data')
    f.close()
    return obj_list


if __name__ == '__main__':
    __pp = pprint.PrettyPrinter(indent=4)
    __path = 'C:/Users/ilyan/Desktop/points.txt'
    __pp.pprint(read_file(__path))
