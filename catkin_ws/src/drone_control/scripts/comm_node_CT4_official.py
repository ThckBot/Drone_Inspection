#!/usr/bin/env python
import numpy as np
import rospy
from drone_fsm import *
from geometry_msgs.msg import PoseArray
from std_srvs.srv import Empty, EmptyResponse
from Obstacle import *

rospy.init_node('rob498_drone_11', anonymous=True)
STATE = 'Init'
WAYPOINTS = None
WAYPOINTS_RECEIVED = False

drone = DroneFSM()

# Callback handlers
def handle_launch():
    global STATE
    STATE = 'Launch'
    drone.fsm_state = STATE
    print('Launch Requested.')

def handle_test():
    global STATE
    STATE = 'Test'
    drone.fsm_state = STATE
    print('Test Requested.')

def handle_land():
    global STATE
    STATE = 'Land'
    drone.fsm_state = STATE
    print('Land Requested.')

def handle_abort():
    global STATE
    STATE = 'Abort'
    drone.fsm_state = STATE
    print('Abort Requested.')

# Service callbacks
def callback_launch(request):
    handle_launch()
    return EmptyResponse()

def callback_test(request):
    handle_test()
    return EmptyResponse()

def callback_land(request):
    handle_land()
    return EmptyResponse()

def callback_abort(request):
    handle_abort()
    return EmptyResponse()

def callback_waypoints(msg):
    global WAYPOINTS_RECEIVED, WAYPOINTS
    if WAYPOINTS_RECEIVED:
        return
    print('Waypoints Received')
    WAYPOINTS_RECEIVED = True
    WAYPOINTS = np.empty((0,3))
    for pose in msg.poses:
        pos = np.array([pose.position.x, pose.position.y, pose.position.z])
        WAYPOINTS = np.vstack((WAYPOINTS, pos))

# Main node
def comm_node():
    global STATE, WAYPOINTS, WAYPOINTS_RECEIVED

    # Do not change the node name and service topics!
    name = 'rob498_drone_11'
    srv_launch = rospy.Service(name+'/comm/launch', Empty, callback_launch)
    srv_test = rospy.Service(name+'/comm/test', Empty, callback_test)
    srv_land = rospy.Service(name+'/comm/land', Empty, callback_land)
    srv_abort = rospy.Service(name+'/comm/abort', Empty, callback_abort)

    sub_waypoints = rospy.Subscriber(name+'/comm/waypoints', PoseArray, callback_waypoints)

    print('This is a dummy drone node to test communication with the ground control')

    # Stored list of obstacles 
    obs_list = []

    # Create Test waypoints
    wp1 = np.array([2, 0, 0.75])
    wp2 = np.array([0, 0, 0.75])
    wp3 = np.array([0, 2, 0.75])

    waypoints = [wp1, wp2, wp3]

    while not rospy.is_shutdown():
        if WAYPOINTS_RECEIVED:
            STATE = 'Waypoints'
            drone.fsm_state = STATE
            print('Waypoints:\n', WAYPOINTS)

            print("Calling path Planner and generating path")
            PathPlan = PathPlanner(obs_list)

            # Generate the trajectory while in air
            start = np.array([drone.position.x, drone.position.y, drone.position.x])
            traj = start.reshape((3,1))
            print("For each waypoint")
            for wp in WAYPOINTS:
                print("Generate path for wp: ", wp)
                sub_traj = PathPlan.generate_trajectory(start, wp)
                traj = np.hstack((traj, sub_traj))
                start = wp

            traj = np.transpose(traj)

            # Nav to waypoints
            for wp in traj:
                print("=========Navigating to waypoint=======: ", traj[:, i])
                drone.nav_waypoints(wp, vicon_milestones = True, vicon_pose = False)
                drone.hover_test(3) # Hover momentarily b/w waypoints

            # for waypt in WAYPOINTS:
            #     waypt[1] = waypt[1]-1 

            #     drone.nav_waypoints(waypt, vicon_milestones = True, vicon_pose = False) # navigate to waypoint
            #     drone.hover_test(3) # hover after finishing waypoints
            
            # Set to done state to prevent going through again
            STATE = 'Waypoints_Complete'

        # Your code goes here
        if STATE == 'Launch':
            print('Comm node: Launching...')
            drone.arm()
            drone.takeoff(1.5)
            drone.hover()

            # Create Path Planner Object
            yaw_list = [0, np.pi/2, np.pi, 3*np.pi/2]
            obstacles = drone.scan_obstacles(yaw_list)

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
            obs_list = [obs1, obs2, obs3, obs4]
        elif STATE == 'Test':
            print('Comm node: Testing...')
            drone.hover()
        elif STATE == 'Land':
            print('Comm node: Landing...')
            drone.land()
        elif STATE == 'Abort':
            print('Comm node: Aborting...')
            drone.shutdown()
    
    
    # ## We want to emulate the following code
    # # Start with arming
    # drone.arm()
    # drone.takeoff(1.5)
    # drone.hover_test(10)

    # # Create Test waypoints
    # wp1 = np.array([2, 0, 0.75])
    # wp2 = np.array([0, 0, 0.75])
    # wp3 = np.array([0, 2, 0.75])

    # waypoints = [wp1, wp2, wp3]

    # # Create Path Planner Object
    # yaw_list = [0, np.pi/2, np.pi, 3*np.pi/2]

    # print("Scanning OBstacles")
    # obstacles = drone.scan_obstacles(yaw_list)
    # obs1 = Obstacle(obstacles[0,0], obstacles[0,1], -1)
    # obs2 = Obstacle(obstacles[1,0], obstacles[1,1], 1)
    # obs3 = Obstacle(obstacles[2,0], obstacles[2,1], -1)
    # obs4 = Obstacle(obstacles[3,0], obstacles[3,1], 1)
    # print("Obstacles collected")
    # print("obs1: ", obs1.coords)
    # print("obs2: ", obs2.coords)
    # print("obs3: ", obs3.coords)
    # print("obs4: ", obs4.coords)
    # obs_list = [obs1, obs2, obs3, obs4]

    # print("Calling path Planner and generating path")
    # PathPlan = PathPlanner(obs_list)

    # # Generate the trajectory while in air
    # start = np.array([drone.position.x, drone.position.y, drone.position.x])
    # traj = start.reshape((3,1))
    # print("For each waypoint")
    # for wp in waypoints:
    #     print("Generate path for wp: ", wp)
    #     sub_traj = PathPlan.generate_trajectory(start, wp)
    #     traj = np.hstack((traj, sub_traj))
    #     start = wp

    # traj = np.transpose(traj)

    # # Go to the positions
    # drone.fsm_state = "Waypoints"
    # for wp in traj:
    #     print("=========Navigating to waypoint=======: ", traj[:, i])
    #     drone.nav_waypoints(wp)


    # drone.hover_test(2)
    # drone.land()
    # drone.shutdown()
    # rospy.sleep(0.2)
        

if __name__ == '__main__':	
    try:	
        comm_node()	
    except rospy.ROSInterruptException:	
        print("didnt make it in comm_node")	
        pass
    
