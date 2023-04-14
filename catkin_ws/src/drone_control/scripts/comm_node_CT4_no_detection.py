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

    # Create Path Planner Object

    obstacles = np.array([[1,0], [0,1]])
    obs1 = Obstacle(obstacles[0,0], obstacles[0,1], -1)
    obs2 = Obstacle(obstacles[1,0], obstacles[1,1], 1)

    obs_list = [obs1, obs2]

    PathPlan = PathPlanner(obs_list)

    # Create Test waypoints
    wp1 = np.array([2, 0, 0.50])
    wp2 = np.array([0, 0, 0.50])
    wp3 = np.array([0, 2, 0.50])

    waypoints = [wp1, wp2, wp3]

    print('Drone Position')
    print(drone.position)
    print('Vicon Position')
    print(drone.vicon_position)
    #drone.compute_vicon_to_ekf_tf()
    print('Transformation')
    print(drone.vicon_transform)

    drone.arm()
    drone.takeoff(0.75)
    drone.hover_test(2)

    # Generate the trajectory while in air
    start = np.array([drone.position.x, drone.position.y, drone.position.x])
    traj = start.reshape((3,1))
    for wp in waypoints:
        
        sub_traj = PathPlan.generate_trajectory(start, wp)
        traj = np.hstack((traj, sub_traj))
        start = wp

    traj = np.transpose(traj)


    # Navigate waypoints
    drone.fsm_state = "Waypoints"
    for wp in traj:
        drone.nav_waypoints(wp)
        #if wp in waypoints:
            #drone.hover_test(3)

    drone.land()
    np.save('positions.npy',np.array(drone.positions).transpose)
    drone.shutdown()
    rospy.sleep(0.2)
    
    
    
        

if __name__ == '__main__':	
    try:	
        comm_node()	
    except rospy.ROSInterruptException:	
        print("didnt make it in comm_node")	
        pass
    
