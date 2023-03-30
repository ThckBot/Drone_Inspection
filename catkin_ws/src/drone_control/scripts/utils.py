
import numpy as np
def rot_from_quat(quat):
    x = quat[0]
    y = quat[1]
    z = quat[2]
    w = quat[3]

    R = [ [1 - 2*y**2 - 2*z**2, 2*x*y - 2*w*z, 2*x*z + 2*w*y],
      [2*x*y + 2*w*z, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*w*x],
      [2*x*z - 2*w*y, 2*y*z + 2*w*x, 1 - 2*x**2 - 2*y**2] ]

    return np.round(R,5)
