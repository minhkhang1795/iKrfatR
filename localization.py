"""
By Khang Vu, 2018

Given transformed coordinates and cubes' size, this scripts tries to
estimate the locations of the cubes
"""
import numpy as np

CUBE_SIZE_SMALL = 0.037  # in m
CUBE_SIZE_LARGE = 0.086  # in m


def cube_localization(coords, cube_size=CUBE_SIZE_SMALL):
    max_y, r_coords = find_boundary(coords, cube_size)
    cube_coords = []
    level = 1
    while level * cube_size <= max_y:
        cube_coords.extend(find_cubes_at_height(r_coords, level, cube_size))
        level += 1
    return cube_coords


def find_boundary(coords, cube_size):
    max_y = None
    reduced_coords = []
    for coord in coords:
        x, y, z = coord
        if y >= cube_size / 2:
            reduced_coords.append(coord)
            if max_y <= y or max_y is None:
                max_y = y

    return max_y, reduced_coords


def find_cubes_at_height(r_coords, level, cube_size):
    cubes = []

    # Find coords at y_max level
    coords_at_y = []
    for coord in r_coords:
        if abs(coord[1] - level * cube_size) < 0.008:
            coords_at_y.append(coord)
            r_coords.remove(coord)

    # Sort in-place by z
    coords_at_y.view('i8,i8,i8').sort(order=['f2'], axis=0)
    while not coords_at_y:
        # Find coords from z_min to z_min + cube_size
        coords_at_z = []
        z_min = coords_at_y[0][2]
        for coord in coords_at_y:
            x, y, z = coord
            if z - z_min <= cube_size:
                coords_at_z.append(coord)
                coords_at_y.remove(coord)
            else:
                break

        # Sort in-place by x
        coords_at_z.view('i8,i8,i8').sort(order=['f0'], axis=0)
        while not coords_at_z:
            coords_at_x = []
            x_min = coords_at_z[0][0]
            for coord in coords_at_z:
                x, y, z = coord
                if x - x_min <= cube_size:
                    coords_at_x.append(coord)
                    coords_at_z.remove(coord)
                else:
                    break
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
    min_x = coords[0]
    max_x = coords[-1]
    for coord in coords:
        x, y, z = coord
        if min_z > z or min_z is None:
            min_z = z
        if max_z <= z or max_z is None:
            max_z = z

    coord_area = abs(max_x - min_x) * abs(min_z - min_z)
    total_area = cube_size * cube_size
    if coord_area >= 0.7 * total_area:
        cube_x = (min_x + max_x) / 2
        cube_y = level * cube_size / 2
        cube_z = (min_z + max_z) / 2
        cube = np.asarray([cube_x, cube_y, cube_z])
    return cube


if __name__ == '__main__':
    pass