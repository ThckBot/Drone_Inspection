#!/usr/bin/env python
import rospy
from drone_fsm import *
from std_srvs.srv import Empty, EmptyResponse

rospy.init_node('rob498_drone_11', anonymous=True)
drone = DroneFSM()


# Callback handlers
def handle_launch():
    global drone
    print('Launch Requested. Your drone should take off.')
    # For milestone 2 we are going to set heigh manually as 1.5m as per spec
    drone.fsm_state = 0

def handle_test():
    global drone
    print('Test Requested. Your drone should perform the required tasks. Recording starts now.')
    # Want to hover in place for 30 seconds for milestone 2
    

def handle_land():
    global drone
    print('Land Requested. Your drone should land.')
    drone.fsm_state = 3

def handle_abort():
    global drone
    print('Abort Requested. Your drone should land immediately due to safety considerations')
    drone.fsm_state = 4
    #DronePlanner

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

# Main communication node for ground control
def comm_node():
    global drone
    print('This is a dummy drone node to test communication with the ground control')
    print('node_name should be rob498_drone_TeamID. Service topics should follow the name convention below')
    print('The TAs will test these service calls prior to flight')
    print('Your own code should be integrated into this node')
    
    node_name = 'rob498_drone_11' 
    srv_launch = rospy.Service(node_name + '/comm/launch', Empty, callback_launch)
    srv_test = rospy.Service(node_name + '/comm/test', Empty, callback_test)
    srv_land = rospy.Service(node_name + '/comm/land', Empty, callback_land)
    srv_abort = rospy.Service(node_name + '/comm/abort', Empty, callback_abort)

    # Your code goes below
    drone.arm()
    drone.takeoff(1.4)
    drone.hover_test(10)
    drone.land()
    drone.shutdown()

    ## MILESTONE 2 ##
    while not rospy.is_shutdown():
        if drone.fsm_state == 0:
            drone.arm()
            drone.takeoff(1.4)
            drone.hover()
        if drone.fsm_state == 3:
            drone.land()
        if drone.fsm_state == 4:
            drone.shutdown()


if __name__ == '__main__':
    try:
        comm_node()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("didnt make it in comm_node")
        pass
    
