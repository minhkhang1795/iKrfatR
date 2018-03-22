"""
By Khang Vu, 2018

Given transformed coordinates and cubes' size, this scripts tries to
estimate the locations of the cubes
"""
import numpy as np

import cv2

CUBE_SIZE_SMALL = 0.037  # in m
CUBE_SIZE_LARGE = 0.086  # in m


def cube_localization(coords, cube_size=CUBE_SIZE_SMALL):
    r_coords = reduced_coords(coords, cube_size)
    cube_coords = []
    level = 1
    while level <= 5:
        cube_coords.extend(find_cubes_at_height(r_coords, level, cube_size))
        level += 1
    return np.asarray(cube_coords)


def reduced_coords(coords, cube_size):
    r_coords = []
    for coord in coords:
        # TODO: y coordinate to -y
        coord[1] = -coord[1]
        x, y, z = coord
        if cube_size / 2 <= y <= cube_size * 5:
            r_coords.append(coord)

    return np.asarray(r_coords)


def find_cubes_at_height(r_coords, level, cube_size):
    cubes = []
    print "level", level

    # Find coords at y_max level
    coords_at_y = []
    for i, coord in enumerate(r_coords):
        if abs(coord[1] - level * cube_size) <= 0.008:
            coords_at_y.append(coord)
    coords_at_y = np.asarray(coords_at_y)

    # Sort in-place by z
    coords_at_y.view('float64,float64,float64').sort(order=['f2'], axis=0)
    while coords_at_y.size != 0:
        # Find coords from z_min to z_min + cube_size
        coords_at_z = []
        z_min = coords_at_y[0][2]
        i_list = []
        for i, coord in enumerate(coords_at_y):
            x, y, z = coord
            if z - z_min <= cube_size:
                coords_at_z.append(coord)
                i_list.append(i)
            else:
                break
        coords_at_y = np.delete(coords_at_y, i_list, axis=0)
        coords_at_z = np.asarray(coords_at_z)

        # Sort in-place by x
        coords_at_z.view('float64,float64,float64').sort(order=['f0'], axis=0)
        while coords_at_z.size != 0:
            coords_at_x = []
            x_min = coords_at_z[0][0]
            x_max = coords_at_z[-1][0]
            i_list = []
            for i, coord in enumerate(coords_at_z):
                x, y, z = coord
                if x - x_min <= cube_size:
                    coords_at_x.append(coord)
                    i_list.append(i)
                else:
                    break
            coords_at_z = np.delete(coords_at_z, i_list, axis=0)
            cube = check_cube(coords_at_x, level, cube_size)
            if cube is not None:
                cubes.append(cube)

    return cubes


def check_cube(coords, level, cube_size):
    """

    :param coords: these coords should make a cube
    :param level: height level
    :param cube_size: size of the cube
    :return: COM of the cube; None if there is no cube
    """
    cube = min_z = max_z = None
    print "level", level
    min_x = coords[0][0]
    max_x = coords[-1][0]
    for coord in coords:
        x, y, z = coord
        if min_z > z or min_z is None:
            min_z = z
        if max_z <= z or max_z is None:
            max_z = z

    # TODO: Height level
    coord_area = abs(max_x - min_x) * abs(max_z - min_z)
    total_area = cube_size * cube_size
    if coord_area >= 0.8 * total_area:
        cube_x = (min_x + max_x) / 2
        cube_y = level * cube_size / 2
        cube_z = (min_z + max_z) / 2
        cube = np.asarray([cube_x, cube_y, cube_z])
    return cube


if __name__ == '__main__':
    # dc = cv2.imread('image_2.jpg')
    # cv2.imshow('RGB & Depth Image', dc)
    # cv2.waitKey(0)

    coords = np.loadtxt('coords_4.txt', dtype=float)
    cubes = cube_localization(coords)
    print len(cubes)
