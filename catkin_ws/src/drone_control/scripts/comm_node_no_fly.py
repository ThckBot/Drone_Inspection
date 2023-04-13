#!/usr/bin/env python
import numpy as np
import rospy
from drone_fsm import *
from geometry_msgs.msg import PoseArray
from std_srvs.srv import Empty, EmptyResponse
from Obstacle import *

rospy.init_node('rob498_drone_11', anonymous=True)

drone = DroneFSM()



# Main node
def comm_node():
    global STATE, WAYPOINTS, WAYPOINTS_RECEIVED

    # Do not change the node name and service topics!
    name = 'rob498_drone_11'
    print('This is a dummy drone node to test communication with the ground control')


    # TODO check transformation
    # TODO check talking and listening

    computed = False
    done_waypoints = False

    # Create Path Planner Object
    yaw_list = [0, np.pi/2, np.pi, 3*np.pi/2]
    obstacles = np.array([[1,0], [0,1]])
    obs1 = Obstacle(obstacles[0,0], obstacles[0,1], -1)
    obs2 = Obstacle(obstacles[1,0], obstacles[1,1], 1)
    # obs3 = Obstacle(obstacles[2,0], obstacles[2,1], -1)
    # obs4 = Obstacle(obstacles[3,0], obstacles[3,1], 1)
    obs_list = [obs1, obs2]

    PathPlan = PathPlanner(obs_list)

    # Create Test waypoints
    wp1 = np.array([2, 0, 0.50])
    wp2 = np.array([0, 0, 0.50])
    wp3 = np.array([0, 2, 0.50])

    waypoints = [wp1, wp2, wp3]
    
    # start = np.array([drone.position.x, drone.position.y, drone.position.x])
    # start = np.array([0, 0, 0])
    # traj = start.reshape((3,1))
    # for wp in waypoints:
        
    #     sub_traj = PathPlan.generate_trajectory(start, wp)
    #     print('Trajectory shape',traj.shape)
    #     print('Sub trajectory shape',sub_traj.shape)
    #     traj = np.hstack((traj, sub_traj))
    #     start = wp

    # print(np.transpose(traj))





    
    
    
        

if __name__ == '__main__':	
    try:	
        comm_node()	
    except rospy.ROSInterruptException:	
        print("didnt make it in comm_node")	
        pass
    
