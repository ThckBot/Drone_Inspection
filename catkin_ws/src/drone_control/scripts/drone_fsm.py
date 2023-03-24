import rospy
import std_msgs
import numpy as np
from geometry_msgs.msg import PoseStamped,TransformStamped, Point
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State 
from mavros_msgs.srv import CommandBool, SetMode
from tf.transformations import *
from mavros_msgs.msg import Waypoint, WaypointList, WaypointReached
from mavros_msgs.srv import WaypointPush, WaypointPushRequest, WaypointClear, WaypointClearRequest
import message_filters

from math import *
import numpy as np
from numpy.linalg import norm
import time


#Drone FSM Class
class DroneFSM():
    def __init__(self, vicon=False):
        # fill in
        self.position = None
        self.orientation = None
        # self.lin_vel = None
        # self.ang_vel = None
        self.state = State()
        
        self.fsm_state = -1

        self.hz = 10
        self.rate = rospy.Rate(self.hz) # Hz

        # some required publishers and subscribers
        self.setpoint_publisher = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        rospy.Subscriber('/mavros/state', State, self.state_callback)
        rospy.Subscriber('/mavros/mission/reached', WaypointReached, self.waypoint_reached_callback)
        
        # Waypoint Subscriber and Publishers
        self.waypoint_client = rospy.ServiceProxy('/mavros/mission/push', WaypointPush)
        self.waypoints = WaypointList() # Initialize list of waypoints
        # TODO Confirm if we are receiving waypoints in format of mavros_msgs/Waypoint Message
        if vicon:
            self.sp = TransformStamped()
            self.sp_pos = self.sp.transform.translation
            rospy.Subscriber("/vicon/ROB498_Drone/ROB498_Drone", TransformStamped, self.vicon_callback, queue_size=10)
        else:
            self.sp = Odometry()
            self.sp_pos = self.sp.pose.pose.position
            rospy.Subscriber('/mavros/local_position/odom', Odometry, self.pose_callback, queue_size=10) # publishes both position and orientation (quaternion)x

    # Callback for the state subscriber
    def state_callback(self, state):
        self.state = state
        #print("self.state: ", self.state)

    # Callback for the pose subscriber
    def pose_callback(self, pose_msg):
        self.position = pose_msg.pose.pose.position
        self.orientation = pose_msg.pose.pose.orientation
        # self.lin_vel = pose_msg.twist.twist.linear
        # self.ang_vel = pose_msg.twist.twist.angular

    def waypoint_reached_callback(self, msg):
        if msg.wp_seq == len(self.waypoints.waypoints) - 1:
            # The last waypoint has been reached
            clear_service = rospy.ServiceProxy('/mavros/mission/clear', WaypointClear)
            clear_service()
            # Maybe add landing here
    # Callback for the pose subscriber
    def vicon_callback(self, pose_msg):
        # has x,y,z
        self.position = pose_msg.transform.translation
        # quaternion
        self.orientation = pose_msg.transform.rotation


    # Arm the drone
    def arm(self):
        self.sp_pos.x = 0
        self.sp_pos.y = 0
        self.sp_pos.z = -1
        # Publish ground set point
        for i in range(self.hz):
            self.publish_setpoint(self.sp_pos)
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
                ret = self.set_mode_client(base_mode=0, custom_mode="OFFBOARD")
                prev_request_t = curr_request_t
                print("return from set_mode: ", ret)
                print("Current mode: %s" % self.state.mode)

            # Then arm it
            if not self.state.armed and request_interval > 2.:
                self.arming_client(True)
                prev_request_t = curr_request_t
                self.publish_setpoint(self.sp_pos)
            
            if self.state.armed and self.state.mode == 'OFFBOARD':
                self.publish_setpoint(self.sp_pos)
                print("System Armed")
                break

            
            self.publish_setpoint(self.sp_pos)
            self.rate.sleep()

        return
    
    # De-arm drone and shut down
    def shutdown(self):
        print("Inside Shutdown")
        while (self.state.armed or self.state.mode == "OFFBOARD"):
            if self.state.armed:
                print("Disarming")
                self.arming_client(False)
                print("Completed Disarming")
                print(self.state.armed)
            if self.state.mode == "OFFBOARD":
                print("Set to manual mode")
                self.set_mode_client(base_mode=0, custom_mode="MANUAL")
            self.rate.sleep()
        print("Is it armed at shutdown", self.state.armed)
        return
    
    # Lift drone off ground
    def takeoff(self, height):
        print("Takeoff...")
        while self.position == None and not rospy.is_shutdown():
            print('Waiting for position...')
            self.rate.sleep()
        self.sp_pos = self.position
        print("Height is: ", height)
        print("self.orientation is: ", self.orientation)
        print("self.position.z is: ", self.position)
        print(rospy.is_shutdown())
        while self.position.z < height -0.02 and not rospy.is_shutdown():
            #print(self.state.armed)
            #print('z_position: ', self.position.z)
            #print('height: ', height)
            self.sp_pos.z = height
            #print('setpoint: ', self.sp_pos)
            #print(self.state.armed)
            #print(self.state.mode)
            self.publish_setpoint(self.sp_pos)
            self.rate.sleep()


    # Hover drone in place for a set amount of time
    def hover(self):
        print('Position holding...')
        t0 = time.time()
        self.sp_pos = self.position
        index = 0
        while (not rospy.is_shutdown()) and self.fsm_state == 'Launch':
            t = time.time()
            if index >= 20:
                print('time: ', t-t0)
                print("orientation is: ", self.orientation)
                print("position is: ", self.position)
                index = 0
            
            index = index + 1

            # Update timestamp and publish sp 
            self.publish_setpoint(self.sp_pos)
            self.rate.sleep()

    def hover_test(self, hover_time):
        print('Position holding...')
        t0 = time.time()
        self.sp_pos = self.position
        index = 0
        t = time.time()
        while (not rospy.is_shutdown()) and hover_time >= t-t0:
            t = time.time()
            if index >= 20:
                print('time: ', t-t0)
                index = 0
            
            index = index + 1

            # Update timestamp and publish sp 
            self.publish_setpoint(self.sp_pos)
            self.rate.sleep()


    # Land drone safely
    def land(self):
        print("Landing...")
        self.sp_pos = self.position
        while (self.position.z > 0.01):
            print(self.position.z)
            self.sp_pos.z = self.position.z - 0.10
            self.publish_setpoint(self.sp_pos)
            self.rate.sleep()
        # self.stop()
    
    # Move drone to milestone
    def move(self, milestone):

        return

    # Publish set point
    def publish_setpoint(self, setpoint, yaw = 0):
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


    def set_waypoints(self, way_points):
        # Inputs: Gets list of waypoints from the command
        # Outputs: Pushes the waypoints into WaypointList of MAVROS to the vehicle
        # Format of Waypoints:

        # for wp in way_points:
        #     waypoint = Waypoint()
        #     waypoint.frame = 3  # Global frame
        #     waypoint.command = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
        #     waypoint.is_current = False
        #     waypoint.autocontinue = True
        #     waypoint.param1 = 0  # Hold time at waypoint in seconds
        #     waypoint.param2 = 0  # Acceptance radius in meters
        #     waypoint.param3 = 0  # Pass through waypoint if set to 1
        #     waypoint.param4 = 0  # Yaw angle in radians
        #     waypoint.x_lat = latitude  # Latitude in degrees
        #     waypoint.y_long = longitude  # Longitude in degrees
        #     waypoint.z_alt = altitude  # Altitude in meters
            
        #     self.waypoint_list.waypoints.append(waypoint)

        # self.waypoint_list.waypoints = [waypoint1, waypoint2, waypoint3]

        # self.waypoint_client(start_index=0, waypoints= self.waypoint_list.waypoints)
        pass


    def set_waypoint_mode(self):
        # TODO should be set to guided or AUTO.mission see MAV_MODE
        self.set_mode_client(custom_mode='AUTO')


    def nav_waypoints(self, wp_next):

        waypoint_pose = Point()
        waypoint_pose.x = wp_next[0]
        waypoint_pose.y = wp_next[1]
        waypoint_pose.z = wp_next[2]

        while not rospy.is_shutdown():
            self.publish_setpoint(waypoint_pose, yaw = 0)
            accum = 0 
            accum += (self.position.x - wp_next[0])**2
            accum += (self.position.y - wp_next[1])**2
            accum += (self.position.z - wp_next[2])**2
            accum = sqrt(accum)
            print("self.position is: ", self.position)
            print("desired waypoint: ", wp_next)
            if accum < 0.1:
                break

            



