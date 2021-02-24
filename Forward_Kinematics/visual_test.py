from mpl_toolkits import mplot3d
#matplotlib inline
import numpy as np
import math as m
import matplotlib.pyplot as plt
from Forward_Kinematics import forward_kin as FK

fig = plt.figure()
ax = plt.axes(projection='3d')
outline_x = [-33.660, -44.919, -11.258, 11.258, 44.919, 33.660, -33.660]
outline_y = [-32.434, -12.934, 45.368, 45.368, -12.934, -32.434, -32.434]
outline_z = [0, 0, 0, 0, 0, 0, 0]
ax.plot3D(outline_x, outline_y, outline_z, 'gray')  # Plot PALM outline
ax.plot3D([0, 10], [0, 0], [0, 0], 'red')  # Plot x axis
ax.plot3D([0, 0], [0, 10], [0, 0], 'green')  # Plot y axis
ax.plot3D([0, 0], [0, 0], [0, 10], 'blue')  # Plot z axis

palm_angle = m.pi/3
loc_INDEX = np.transpose(FK.finger_fk("INDEX", palm_angle, [0, m.pi/2, m.pi/6, m.pi/3]))
loc_INDEX_M = np.transpose(FK.finger_fk("INDEX_M", palm_angle, [0, m.pi/2, m.pi/6, m.pi/3]))
loc_THUMB = np.transpose(FK.finger_fk("THUMB", palm_angle, [0, m.pi/2, m.pi/6, m.pi/3]))

ax.plot3D(loc_INDEX[0, :], loc_INDEX[1, :], loc_INDEX[2, :], 'black')  # Plot INDEX
ax.plot3D(loc_INDEX_M[0, :], loc_INDEX_M[1, :], loc_INDEX_M[2, :], 'black')  # Plot INDEX_M
ax.plot3D(loc_THUMB[0, :], loc_THUMB[1, :], loc_THUMB[2, :], 'black')  # Plot THUMB

plt.show()
