import rospy
import std_msgs
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State 
from mavros_msgs.srv import CommandBool, SetMode
from tf.transformations import *
import message_filters

from math import *
import numpy as np
from numpy.linalg import norm
import time


#Drone FSM Class
class DroneFSM():
    def __init__(self):
        # fill in
        self.pose = Odometry()
        self.state = State()
        self.hz = 10
        self.rate = rospy.Rate(self.hz) # Hz

        # some required publishers and subscribers
        self.setpoint_publisher = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        rospy.Subscriber('/mavros/state', State, self.state_callback)
        rospy.Subscriber('/mavros/odometry/out', Odometry, self.pose_callback, queue_size=10) # publishes both position and orientation (quaternion)x
        print('here')

    
    # Callback for the state subscriber
    def state_callback(self, state):
        self.state = state
        print("self.state: ", self.state)

    # Callback for the pose subscriber
    def pose_callback(self, pose_msg):
        self.pose = pose_msg
        print("self.pose: ", self.pose)
        # Check if we get the correct pose

    # Arm the drone
    def arm(self):
        # Publish ground set point
        for i in range(self.hz):
            self.publish_setpoint([0,0,0])
            self.rate.sleep()

        while not self.state.connected:
            print('Waiting for FCU Connection')
            self.rate.sleep()

        prev_request_t = rospy.get_time()
        while not rospy.is_shutdown():
            curr_request_t = rospy.get_time()
            request_interval = curr_request_t - prev_request_t
            # First set to offboard mode
            if self.state.mode != "OFFBOARD" and request_interval > 2.:
                self.set_mode_client(base_mode=0, custom_mode="OFFBOARD")
                prev_request_t = curr_request_t
                print("Current mode: %s" % self.current_state.mode)

            # Then arm it
            if not self.state.armed and request_interval > 2.:
                self.arming_client(True)
                prev_request_t = curr_request_t
                print("Vehicle armed: %r" % self.current_state.armed)
            
            if self.state.armed:
                break

            self.publish_setpoint([0,0,0])
            self.rate.sleep()

        return
    
    # De-arm drone and shut down
    def shutdown(self):
        while self.state.armed or self.state.mode == "OFFBOARD":
            if self.state.armed:
                self.arming_client(False)
            if self.state.mode == "OFFBOARD":
                self.set_mode_client(base_mode=0, custom_mode="MANUAL")
            self.rate.sleep()
        return
    
    # Lift drone off ground
    def takeoff(self, height):
        print("Takeoff...")
        self.sp = self.pose
        while self.pose[2] < height:
            self.sp[2] += 0.02
            self.publish_setpoint(self.sp)
            self.rate.sleep()


    # Hover drone in place for a set amount of time
    def hover(self, t_hold):
        print('Position holding...')
        t0 = time.time()
        self.sp = self.pose
        while not rospy.is_shutdown():
            t = time.time()
            if t - t0 > t_hold and t_hold > 0: break
            # Update timestamp and publish sp 
            self.publish_setpoint(self.sp)
            self.rate.sleep()

    
    # Land drone safely
    def land(self):
        print("Landing...")
        self.sp = self.pose
        while self.sp[2] > - 1.0:
            self.sp[2] -= 0.05
            self.publish_setpoint(self.sp)
            self.rate.sleep()
        # self.stop()
    
    # Move drone to milestone
    def move(self, milestone):

        return

    # Publish set point
    def publish_setpoint(self, setpoint, yaw = np.pi/2):
        sp = PoseStamped()
        sp.pose.position.x = setpoint[0]
        sp.pose.position.y = setpoint[1]
        sp.pose.position.z = setpoint[2]
        q = quaternion_from_euler(0, 0, yaw)
        
        sp.pose.orientation.x = q[0]
        sp.pose.orientation.y = q[1]
        sp.pose.orientation.z = q[2]
        sp.pose.orientation.w = q[3]

        # Publish to the rosnode
        sp.header.stamp = rospy.Time.now()
        self.setpoint_publisher.publish(sp)

        return