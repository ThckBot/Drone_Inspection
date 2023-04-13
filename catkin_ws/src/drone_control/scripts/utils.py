
import numpy as np
import math

def rot_from_quat(quat):
    x = quat[0]
    y = quat[1]
    z = quat[2]
    w = quat[3]

    R = [ [1 - 2*y**2 - 2*z**2, 2*x*y - 2*w*z, 2*x*z + 2*w*y],
      [2*x*y + 2*w*z, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*w*x],
      [2*x*z - 2*w*y, 2*y*z + 2*w*x, 1 - 2*x**2 - 2*y**2] ]

    return np.round(R,5)


def quaternion_to_yaw(q):
    '''
    INPUTS: quaternion q.x,y,z,w
    OUTPUTS: yaw calculated from quaternion in radians
    '''
    x, y, z, w = q.x, q.y, q.z, q.w
    yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return yaw_rad
