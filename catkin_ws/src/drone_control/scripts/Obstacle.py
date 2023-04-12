import rospy
import tf
from utils import *
import std_msgs
import numpy as np
from geometry_msgs.msg import PoseStamped,TransformStamped, Point
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State 
from mavros_msgs.srv import CommandBool, SetMode
from tf.transformations import *
from mavros_msgs.msg import Waypoint, WaypointList, WaypointReached
from mavros_msgs.srv import WaypointPush, WaypointPushRequest, WaypointClear, WaypointClearRequest
import message_filters

from math import *
import numpy as np
from numpy.linalg import norm
import time


# Class for Obstacle Avoidance
class Obstacle:
    def __init__(self, x, y, colour=0):
        # fill in
        #red = -1, green = 1
        self.colour = colour
        self.coords = np.array([x, y])


class PathPlanner:
    def __init__(self, ap_obs_list, resolution = 0.4):
        self.obstacles = ap_obs_list
        self.next_obs = 0
        self.min_dist = resolution

    def add_obstacle(self, obstacle):
        self.obstacles[self.next_obs] = obstacle
    
    def increment_obs(self):
        self.next_obs += 1

    def adjust_waypoint(self, old_wp):
        z = np.array([0,0,1])
        adjust_direction = np.cross(old_wp,z)
        norm_direction = (self.obstacles[self.next_obs].colour)*adjust_direction/np.linalg.norm(adjust_direction)
        new_wp = old_wp + (self.min_dist)*norm_direction 
        
        return new_wp

    def check_collision(self, waypoint):
        obst = self.obstacles[self.next_obs].coords
        if np.linalg.norm(waypoint[0:2] - obst) < self.min_dist:
            return True
        return False

    def generate_trajectory(self, next_wp, curr_pos):
        start, end = np.array([curr_pos.x, curr_pos.y, curr_pos.z]), np.array([next_wp.x, next_wp.y, next_wp.z])
        transl = np.array(end - start)
        num_wp = int(np.floor(np.linalg.norm(transl) / self.min_dist))
        print("start", start)
        print("end", end)
        traj = np.array([np.linspace(start[0], end[0], num_wp), np.linspace(start[1], end[1], num_wp), np.linspace(start[2], end[2], num_wp) ])
        print("shape of traj: ", traj.shape)
        print("num_wpts is: ", num_wp)
        for i in range(0, num_wp-1, 1):

            if self.check_collision(traj[:,i]):
                traj[:,i] = self.adjust_waypoint(traj[:,i])

        return traj