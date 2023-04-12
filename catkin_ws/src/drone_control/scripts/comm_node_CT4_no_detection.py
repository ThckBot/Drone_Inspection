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

    computed = False
    done_waypoints = False

    # Create Path Planner Object
    ob1 = [Obstacle(1, 0, 1)]
    PathPlan = PathPlanner(ob1)

    # Create Test waypoints
    pos1 = np.array([2, 0, 0.50])
    wp1 = Point()
    wp1.x = pos1[0]
    wp1.y = pos1[1]
    wp1.z = pos1[2]

    start = Point()
    # Create Trajectory
    print("WP1: ", wp1)


    print("trajectory is: traj")

    # Arm the drone
    drone.arm()
    drone.takeoff(0.50)
    drone.hover_test(5)
    
    #traj = PathPlan.generate_trajectory(wp1, drone.position)
    # Go to the positions
    #for i in range(0, traj.shape[1]):
    #    print("Waypoint is: ", traj[:, i])
    #    drone.nav_waypoints(traj[:, i].T)


    # print("Moving to waypoint 1\n")
    # drone.nav_waypoints(pos1)
    # drone.hover_test(5)
    # print("Moving to waypoint 2\n")
    # drone.nav_waypoints(pos2)
    # drone.hover_test(5)
    # print("Moving to waypoint 3\n")
    # drone.nav_waypoints(pos3)
    drone.hover_test(5)
    drone.land()
    drone.shutdown()
    rospy.sleep(0.2)
    
    
    
        

if __name__ == '__main__':	
    try:	
        comm_node()	
    except rospy.ROSInterruptException:	
        print("didnt make it in comm_node")	
        pass
    