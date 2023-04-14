#!/usr/bin/env python
import numpy as np
from math import *

class Obstacle:
    def __init__(self, x, y, colour=0):
        # fill in
        #red = -1, green = 1
        self.colour = colour
        self.coords = np.array([x, y])

obstacles = np.array([[-1.929,-2.994],[2.002,-1.996],[2.434,1.805],[-1.674,1.498]])

# Scan obstacles
obs1 = Obstacle(obstacles[0,0], obstacles[0,1], -1)
obs2 = Obstacle(obstacles[1,0], obstacles[1,1], 1)
obs3 = Obstacle(obstacles[2,0], obstacles[2,1], -1)
obs4 = Obstacle(obstacles[3,0], obstacles[3,1], 1)
print("Obstacles collected")
print("obs1: ", obs1.coords)
print("obs2: ", obs2.coords)
print("obs3: ", obs3.coords)
print("obs4: ", obs4.coords)