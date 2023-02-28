import rospy
import std_msgs
from geometry_msgs.msg import PoseStamped
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
        self.pose = None
        self.state = None

        self.rate = rospy.Rate(10) # Hz

        # some required publishers and subscribers
        self.setpoint_publisher = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        rospy.Subscriber('/mavros/state', State, self.state_callback)
        rospy.Subscriber('/mavros/odometry/out', PoseStamped, self.pose_callback) # publishes both position and orientation (quaternion)

    
    # Callback for the state subscriber
    def state_callback(self, state):
        self.state = state

    # Callback for the pose subscriber
    def pose_callback(self, pose_msg):
        self.pose = pose_msg
        # Check if we get the correct pose

    # Arm the drone
    def arm(self):
        
        return
    
    # De-arm drone and shut down
    def shutdown(self):

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