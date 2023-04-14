import numpy as np
import matplotlib.pyplot as plt

waypoints = np.load('waypoints.npy')
vicon_waypoints = np.load('vicon_waypoints.py')
vicon_positions = np.load('vicon_positions.py')
positions = np.load('positions.py')
transform = np.load('transformation.npy')

transform_waypoints = np.matmul(transform,vicon_waypoints)
transform_positions = np.matmul(transform,vicon_positions)

fig = plt.figure()
ax = fig.add_subplot(projection='3d')

ax.scatter(waypoints[0,:],waypoints[1,:],waypoints[2,:])
ax.scatter(transform_waypoints[0,:],transform_waypoints[1,:],transform_waypoints[2,:])
ax.plot(positions[0,:],positions[1,:],positions[2,:])
ax.plot(transform_positions[0,:],transform_positions[1,:],transform_positions[2,:])

plt.show()
