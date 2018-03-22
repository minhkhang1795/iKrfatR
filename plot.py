import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

c = []
i = 0
coords = np.loadtxt('coords_1_special_case.txt', dtype=float)
for coord in coords:
    x, y, z = coord
    i += 1
    if not np.math.isnan(x) and y < 0.5:
        # if i % 50 != 0:
        #     continue
        c.append(coord)

c = np.asarray(c)
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(c[:, 0], c[:, 1], c[:, 2], zdir='z', c='red')
# plt.savefig("plot.png")
plt.show()