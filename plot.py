import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

if __name__ == '__main__':
    c = []
    i = 0
    coords = np.loadtxt('coords_8.txt', dtype=float)
    for coord in coords:
        x, y, z = coord
        i += 1
        if not np.math.isnan(x) and y < 0.5 and z < 1.5:
            if i % 30 != 0:
                continue
            c.append(coord)

    c = np.asarray(c)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(c[:, 0], c[:, 1], c[:, 2], zdir='z', c='red')
    plt.savefig("plot.png")
    plt.show()


def plot_cube2D(cubes):
    xcoordinates = []
    zcoordinates = []
    for cube in cubes:
        x, y, z = cube
        xcoordinates.append(x)
        zcoordinates.append(z)
    plt.plot(xcoordinates, zcoordinates, 'ro')
    plt.show()
