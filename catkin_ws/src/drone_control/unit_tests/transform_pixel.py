#!/usr/bin/env python
import numpy as np
from math import *

class Quaternion:
    def __init__(self, x, y, z, w):
        self._x = x
        self._y = y
        self._z = z
        self._w = w

    @property
    def x(self):
        return self._x

    @property
    def y(self):
        return self._y

    @property
    def z(self):
        return self._z

    @property
    def w(self):
        return self._w

    def __repr__(self):
        return f"Quaternion({self._x}, {self._y}, {self._z}, {self._w})"



def quaternion_to_yaw(q):
    '''
    INPUTS: quaternion q.x,y,z,w
    OUTPUTS: yaw calculated from quaternion in radians
    '''
    x, y, z, w = q.x, q.y, q.z, q.w
    yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    yaw_rad = yaw * np.pi / 180
    return yaw_rad

def coord_from_pixel(x_pixel, dist_to_obstacle):
    """
    Inputs: x_pixel - pixel coord in image
            dist_to_obstacle - distance to obstacle
    Parameters: width - camera_pixel width 
    Returns x_pos, y_pos in local coordinate frame
    """

    # Set Parameters
    CAMERA_PIXEL_WIDTH = 500
    CAMERA_SCALE = 30 # TODO Scale should be based on detected obstacle width

    # Get the offset of pixel position from centre of image
    pixel_offset = x_pixel - (CAMERA_PIXEL_WIDTH/2)
    
    # Relative x offset of the obstacle in frame
    coord_offset = pixel_offset / CAMERA_SCALE 

    # Relative orientation in radians of obstacle to robot position
    theta_relative = np.arctan2(coord_offset, dist_to_obstacle) #TODO does order make sense

    # Get Current Yaw
    current_yaw = quaternion_to_yaw(global_orientation)

    # Yaw in drone frame
    yaw = current_yaw + theta_relative

    ## Compute obstacle position ##
    # Get the diagonal distane to obstacle
    diag_distance = sqrt(coord_offset**2 + dist_to_obstacle**2)
    
    x_pos = diag_distance*np.sin(yaw)
    y_pos = diag_distance*np.cos(yaw)

    return x_pos, y_pos


num_tests = 3

# TEST CASE 1: Obstacle at centre 3 meters away
x_pixels = [250,300,350]
dist_to_obstacles = [3,3,3]
global global_orientation 
global_orientation = Quaternion(0,0,0,1)

for i in range(num_tests):
    print("\n====== TEST CASE: ", i+1, " ======")
    print("x_pixels", x_pixels[i])
    print("dist_to_obstacles", dist_to_obstacles[i])
    obstacle_pos = coord_from_pixel(x_pixels[i], dist_to_obstacles[0])
    print("obstacle_pos", obstacle_pos)

    