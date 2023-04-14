#!/usr/bin/env python
import numpy as np
from math import *

def rot_from_quat(quat):
    x = quat[0]
    y = quat[1]
    z = quat[2]
    w = quat[3]

    R = [ [1 - 2*y**2 - 2*z**2, 2*x*y - 2*w*z, 2*x*z + 2*w*y],
      [2*x*y + 2*w*z, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*w*x],
      [2*x*z - 2*w*y, 2*y*z + 2*w*x, 1 - 2*x**2 - 2*y**2] ]

    return np.round(R,5)


# Local orientation
orientation = [ 0.400447, 0.0266965, 0.8542868, 0.3303385 ] #xyz (2,4,1)
# orientation = [0, 0, 0, 0] #xyz (2,4,1)
x1 = orientation[0]
y1 = orientation[1]
z1 = orientation[2]
w1 = orientation[3]

# Vicon Orientation
# vicon_orientation = [0,0,0,0]
vicon_orientation = [0,0,0.3826834,0.9238795] # 45 degrees in quaternion
x2 = vicon_orientation[0]
y2 = vicon_orientation[1]
z2 = vicon_orientation[2]
w2 = vicon_orientation[3]

# Local Position
position = [0,1,4]
# position = [1,2,3]
px1 = position[0]
py1 = position[1]
pz1 = position[2]

# Vicon position
vicon_position = [1,0,4]
# vicon_position = [4,5,6]
px2 = vicon_position[0]
py2 = vicon_position[1]
pz2 = vicon_position[2]

pos1 = [px1, py1, pz1]
pos2 = [px2, py2, pz2]

quat1 = [x1, y1, z1, w1]
quat2 = [x2, y2, z2, w2]


r1 = rot_from_quat(quat1)
r2 = rot_from_quat(quat2)

# Get homogenous transformation matrices
T1 = np.eye(4)
T1[:3, :3] = r1
T1[:3, 3] = pos1

T2 = np.eye(4)
T2[:3, :3] = np.transpose(r2)
T2[:3, 3] = -np.matmul(np.transpose(r2),pos2)

# T1 = np.eye(4)
# T1[:3, :3] = np.transpose(r1)
# T1[:3, 3] = -np.matmul(np.transpose(r1),pos1)

# T2 = np.eye(4)
# T2[:3, :3] = r2
# T2[:3, 3] = pos2

print("T1*P1: ", )
print("T2*P2: ", )

# Compute the transformation matrix between the two poses changes from vicon to local frame
T21 = np.matmul(T1, T2)

# self.vicon_transform = T21
# self.vicon_transform_created = True

print("=======Computed Transform=======")     
print("self.position ", position)
print("self.orientation ", orientation)
print("self.vicon_position ", vicon_position)
print("self.vicon_orientation ", vicon_orientation)
print("T1: \n", T1)
print("T2: \n", T2)
print("Final Transform: \n", T21)

print("=======VALIDATION=======")
augmented_point = vicon_position
augmented_point.append(1)
print("Augmented OG point", augmented_point)
print("OG local point", position)
print("Transform vicon point", np.matmul(T21,augmented_point))
