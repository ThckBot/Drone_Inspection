#!/usr/bin/env python
import numpy as np
import rospy
from drone_fsm import *
from geometry_msgs.msg import PoseArray
from std_srvs.srv import Empty, EmptyResponse

rospy.init_node('rob498_drone_11', anonymous=True)
STATE = 'Init'
WAYPOINTS = None
WAYPOINTS_RECEIVED = False

drone = DroneFSM(vicon=True)

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
    while not rospy.is_shutdown():
        if WAYPOINTS_RECEIVED:
            STATE = 'Waypoints'
            drone.compute_vicon_to_ekf_tf()
            drone.fsm_state = STATE
            print('Waypoints:\n', WAYPOINTS)
            for waypt in WAYPOINTS:
                print('Done computing transform')
                drone.nav_waypoints(waypt, vicon_milestones = True, vicon_pose = False) # navigate to waypoint
                drone.hover_test(2)
                drone.update_rotation()
                drone.hover_test(2) # hover after finishing waypoints
            done_waypoints = True
            print("Done waypoints")
            np.save('positions.npy',np.array(drone.positions).transpose)
            np.save('vicon_positions.npy',np.array(drone.vicon_positions).transpose)
            np.save('waypoints.npy',np.array(drone.waypoints).transpose)
            np.save('vicon_waypoints.npy',np.array(drone.vicon_waypoints).transpose)
            np.save('transformation.npy',np.array(drone.vicon_transform))
            
        # Your code goes here
        if STATE == 'Launch':
            print('Comm node: Launching...')
            drone.arm()
            drone.takeoff(.75)
            drone.hover()
        elif STATE == 'Test':
            if computed == False:
                computed = True
                print('Comm node: Testing...')

            drone.hover()
        elif STATE == 'Land':
            print('Comm node: Landing...')
            drone.land()
        elif STATE == 'Abort':
            print('Comm node: Aborting...')
            drone.shutdown()

    
    
        

if __name__ == '__main__':	
    try:	
        comm_node()	
    except rospy.ROSInterruptException:	
        print("didnt make it in comm_node")	
        pass
    
