"""
 * File: offb_node.py
 * Stack and tested in Gazebo Classic 9 SITL
"""

#! /usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

# Packages imported previously
import std_msgs
from nav_msgs.msg import Odometry
from tf.transformations import *
import message_filters

from math import *
import numpy as np
from numpy.linalg import norm
import time


class DroneFSM():
    def __init__(self):
        # fill in
        # self.position = None
        # self.orientation = None
        self.state = State()
        # self.sp = Odometry()
        # self.sp_pos = self.sp.pose.pose.position

        self.hz = 10
        self.rate = rospy.Rate(self.hz) # Hz

        # # some required publishers and subscribers
        self.setpoint_publisher = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)

        rospy.wait_for_service("/mavros/cmd/arming")
        self.arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool) 
           
        rospy.wait_for_service("/mavros/set_mode")
        self.set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

        rospy.Subscriber('/mavros/state', State, self.state_callback)
        rospy.Subscriber('/mavros/odometry/out', Odometry, self.pose_callback, queue_size=10) # publishes both position and orientation (quaternion)x

    
    # Callback for the state subscriber
    def state_callback(self, state):
        self.state = state

    # Callback for the pose subscriber
    def pose_callback(self, pose_msg):
        self.position = pose_msg.pose.pose.position
        self.orientation = pose_msg.pose.pose.orientation

    # Publish set point
    def publish_setpoint(self, setpoint, yaw = np.pi/2):
        sp = PoseStamped()
        sp.pose.position.x = setpoint.x
        sp.pose.position.y = setpoint.y
        sp.pose.position.z = setpoint.z

        q = quaternion_from_euler(0, 0, yaw)
        sp.pose.orientation.x = q[0]
        sp.pose.orientation.y = q[1]
        sp.pose.orientation.z = q[2]
        sp.pose.orientation.w = q[3]

        # Publish to the rosnode
        sp.header.stamp = rospy.Time.now()
        self.setpoint_publisher.publish(sp)

        return

    def arm(self):
        # Wait for Flight Controller connection
        while(not rospy.is_shutdown() and not self.state.connected):
            self.rate.sleep()

        pose = PoseStamped()

        pose.pose.position.x = 0
        pose.pose.position.y = 0
        pose.pose.position.z = 0

        # Send a few setpoints before starting
        for i in range(100):   
            if(rospy.is_shutdown()):
                break

            self.publish_setpoint(pose)
            self.rate.sleep()

        offb_set_mode = SetModeRequest()
        offb_set_mode.custom_mode = 'OFFBOARD'

        arm_cmd = CommandBoolRequest()
        arm_cmd.value = True

        last_req = rospy.Time.now()

        while(not rospy.is_shutdown()):
            if(self.state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(self.set_mode_client.call(offb_set_mode).mode_sent == True):
                    rospy.loginfo("OFFBOARD enabled")
                
                last_req = rospy.Time.now()
            else:
                if(not self.state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                    if(self.arming_client.call(arm_cmd).success == True):
                        rospy.loginfo("Vehicle armed")
                
                    last_req = rospy.Time.now()

            self.publish_setpoint(pose)

            self.rate.sleep()

    # De-arm drone and shut down
    def shutdown(self):
        print("Inside Shutdown")
        while self.state.armed or self.state.mode == "OFFBOARD":
            if self.state.armed:
                self.arming_client(False)
            if self.state.mode == "OFFBOARD":
                self.set_mode_client(base_mode=0, custom_mode="MANUAL")
            self.rate.sleep()
        print("Is it armed at shutdown", self.state.armed)
        return
    
    # Lift drone off ground
    def takeoff(self, height):
        print("Takeoff...")
        self.sp_pos = self.position
        while self.position.z < height:
            self.sp_pos.z += 0.02
            self.publish_setpoint(self.sp_pos)
            self.rate.sleep()


    # Hover drone in place for a set amount of time
    def hover(self, t_hold):
        print('Position holding...')
        t0 = time.time()
        self.sp_pos = self.position
        while not rospy.is_shutdown():
            t = time.time()
            if t - t0 > t_hold and t_hold > 0: break
            # Update timestamp and publish sp 
            self.publish_setpoint(self.sp_pos)
            self.rate.sleep()


    # Land drone safely
    def land(self):
        print("Landing...")
        self.sp_pos = self.position
        while self.sp_pos.z > 0.0:
            self.sp_pos.z -= 0.05
            self.publish_setpoint(self.sp_pos)
            self.rate.sleep()
        # self.stop()

if __name__ == "__main__":
    rospy.init_node("offb_node_py")
 
    

    