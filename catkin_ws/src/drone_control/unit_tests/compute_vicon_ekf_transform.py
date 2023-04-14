#!/usr/bin/env python
import numpy as np
from math import *



# Local position
x1 = 0
y1 = 0
z1 = 0
w1 = 0

# Vicon Orientation
x2 = 0
y2 = 0
z2 = 0.3826834
w2 = 0.9238795

# Local Position
px1 = 0
py1 = 0
pz1 = 0

# Vicon position
px2 = 0
py2 = 0
pz2 = 0

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

self.t1 = pos1
self.t2 = pos2

# Compute the transformation matrix between the two poses changes from vicon to local frame
T21 = np.matmul(T2, T1)

self.vicon_transform = T21
self.vicon_transform_created = True
print("=======Computed Transform=======")     
print("self.position ", self.position)
print("self.orientation ", self.orientation)
print("self.vicon_position ", self.vicon_position)
print("self.vicon_orientation ", self.vicon_orientation)
print("Final TransformT21: ", T21)
