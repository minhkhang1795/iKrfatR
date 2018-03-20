"""
By Khang Vu, 2018

Given transformed coordinates and cubes' size, this scripts tries to
estimate the locations of the cubes
"""
import numpy as np

CUBE_SIZE_SMALL = 0.037  # in m
CUBE_SIZE_LARGE = 0.086  # in m


def cube_localization(coords, cube_size=CUBE_SIZE_SMALL):
    min_x, max_x, min_z, max_z, max_y, r_coords = find_boundary(coords, cube_size)
    cube_coords = []
    y = cube_size
    while y < max_y:
        cube_coords.extend(find_cubes(r_coords, y, cube_size))
        y += cube_size
    return cube_coords


def find_boundary(coords, cube_size):
    min_x = max_x = min_z = max_z = max_y = None
    reduced_coords = []
    for coord in coords:
        x, y, z = coord
        if y < cube_size / 2:
            continue
        reduced_coords.append(coord)
        if min_x < x or min_x is None:
            min_x = x
        if max_x >= x or max_x is None:
            max_x = x
        if min_z < z or min_z is None:
            min_z = z
        if max_z >= z or max_z is None:
            max_z = z
        if max_y >= y or max_y is None:
            max_y = y

    return min_x, max_x, min_z, max_z, max_y, reduced_coords


def find_cubes(r_coords, y_max, cube_size):
    cubes = []
    coords_at_y = []
    for coord in r_coords:
        if abs(coord[1] - y_max) < 0.008:
            coords_at_y.append(coord)
    
    return cubes
