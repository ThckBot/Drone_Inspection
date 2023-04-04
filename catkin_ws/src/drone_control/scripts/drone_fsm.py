import rospy
import tf
from utils import *
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

        self.vicon_position = None
        self.vicon_orientation = None
        self.ekf_position = None
        self.ekf_orientation = None
        
        self.vicon_transform = np.identity(4) # Default transform is all 1s

        self.state = State()
        
        self.fsm_state = -1
        self.vicon_milestones = vicon
        self.hz = 10
        self.rate = rospy.Rate(self.hz) # Hz

        # some required publishers and subscribers
        self.setpoint_publisher = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        rospy.Subscriber('/mavros/state', State, self.state_callback)

        self.vicon_transform_created = False
        

        # Subscribe to vicon and local_position

        rospy.Subscriber('/mavros/local_position/odom', Odometry, self.local_position_callback, queue_size=10) # publishes both position and orientation (quaternion)
        #rospy.Subscriber('/mavros/odometry/out', Odometry, self.local_position_callback, queue_size=10) # publishes both position and orientation (quaternion)
        rospy.Subscriber("/vicon/ROB498_Drone/ROB498_Drone", TransformStamped, self.vicon_position_callback, queue_size=10)

        self.sp_pos = Point()

    # Callback for the state subscribers
    def state_callback(self, state):
        self.state = state

    # Callback for the pose subscriber
    def local_position_callback(self, pose_msg):
        self.position = pose_msg.pose.pose.position
        self.orientation = pose_msg.pose.pose.orientation

    # Callback for the pose subscriber
    def vicon_position_callback(self, pose_msg):
        self.vicon_position = pose_msg.transform.translation
        self.vicon_orientation = pose_msg.transform.rotation
        #print("vicon position")
        #print(pose_msg.transform.translation)
        #print("vicon_orientation")
        #print(pose_msg.transform.rotation)

    # Use the transformation matrix to multiply and transform a point 
    def compute_waypoint_transform(self, wp):
        # Put waypoint in augmented form to multiply with vicon transform
        wp_aug = np.array([[wp[0]],[wp[1]],[wp[2]], [1]])
        wp_transformed = np.matmul(self.vicon_transform,wp_aug)
        
        return wp_transformed[0:3]

    def compute_vicon_to_ekf_tf(self):

        x1 = self.orientation.x
        y1 = self.orientation.y
        z1 = self.orientation.z
        w1 = self.orientation.w

        x2 = self.vicon_orientation.x
        y2 = self.vicon_orientation.y
        z2 = self.vicon_orientation.z
        w2 = self.vicon_orientation.w

        px1 = self.position.x
        py1 = self.position.y
        pz1 = self.position.z

        px2 = self.vicon_position.x
        py2 = self.vicon_position.y
        pz2 = self.vicon_position.z

        pos1 = [px1, py1, pz1]
        pos2 = [px2, py2, pz2]

        quat1 = [x1, y1, z1, w1]
        quat2 = [x2, y2, z2, w2]

        
        r1 = rot_from_quat(quat1)
        r2 = rot_from_quat(quat2)

        # Get homogenous transformation matrices
        T1 = np.eye(4)
        T1[:3, :3] = r1
        T1[:3, 3] = pos1

        T2 = np.eye(4)
        T2[:3, :3] = r2
        T2[:3, 3] = pos2

        # Compute the transformation matrix between the two poses changes from vicon to local frame
        T21 = np.matmul(np.linalg.inv(T2), T1)

        return T21

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

        # Create transformation matrix 
        # Assuming we know the vicon and point locations from self.position
        if not self.vicon_transform_created:
            print("Creating transform")     
                   # Create transform objects
            print("self.position")
            print(self.position)
            print("self.orientation")
            print(self.orientation)

            print("self.vicon_position")
            print(self.vicon_position)
            print("self.vicon_orientation")
            print(self.vicon_orientation)
            self.vicon_transform = self.compute_vicon_to_ekf_tf()
            self.vicon_transform_created = True
            print(self.vicon_transform)
            print('Testing transform')
            print('-----------------------------------')
            wp = [1,2,3]
            print('Original point: ', wp)
            trans_wp = self.compute_waypoint_transform(wp)
            print('Transformed point: ', trans_wp)
        else:
            print('Warning: transform already created')
            
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
            self.sp_pos.z = self.position.z - 0.05
            self.publish_setpoint(self.sp_pos)
            self.rate.sleep()
        # self.stop()
    
    # Move drone to milestone
    def move(self, milestone):

        return

    # Publish set point
    def publish_setpoint(self, setpoint, yaw = 0):
        sp = PoseStamped()
        

        #theta = -0.48
        theta = 0
        #theta = 0

        fix_frame = np.array([[np.cos(theta),-np.sin(theta),0],[np.sin(theta),np.cos(theta),0],[0,0,1]])
        
        temp_sp = np.array([[setpoint.x],[setpoint.y],[setpoint.z]])

        setpoint_transformed = np.matmul(fix_frame,temp_sp.reshape((3,1)))

        sp.pose.position.x = setpoint_transformed[0]
        sp.pose.position.y = setpoint_transformed[1]
        sp.pose.position.z = setpoint_transformed[2]

        q = quaternion_from_euler(0, 0, -theta)
        sp.pose.orientation.x = q[0]
        sp.pose.orientation.y = q[1]
        sp.pose.orientation.z = q[2]
        sp.pose.orientation.w = q[3]

        # Publish to the rosnode
        sp.header.stamp = rospy.Time.now()
        self.setpoint_publisher.publish(sp)


        return


    def nav_waypoints(self, wp_next, vicon_milestones = False, vicon_pose = False):

        waypoint_pose = Point()
        if vicon_milestones:
            # Transform milestones from vicon to local frame for publishing
            wp_transformed = self.compute_waypoint_transform(wp_next)
            waypoint_pose.x = wp_transformed[0]
            waypoint_pose.y = wp_transformed[1]
            waypoint_pose.z = wp_transformed[2]
        else:
            # Set up waypoints without transformation
            waypoint_pose.x = wp_next[0]
            waypoint_pose.y = wp_next[1]
            waypoint_pose.z = wp_next[2]

        print('Moving to waypoint:', waypoint_pose)
        # Drive drone to waypoint
        while not rospy.is_shutdown() and self.fsm_state == 'Waypoints':
            self.publish_setpoint(waypoint_pose, yaw = 0)

            if vicon_pose:
                # Directly compare the vicon pose to the milestone
                pose = self.vicon_position
                waypoint = wp_next # if using vicon pose and vicon milestone, compare directly
            else:
                # Otherwise compare the local pose to the milestone
                pose = self.position
                if vicon_milestones:
                    waypoint = wp_transformed # If using local pose and vicon milestone, waypoints need to be transformed to local frame
                else:
                    waypoint = wp_next # if using local pose and local milestone, compare directly

            accum = 0 
            accum += (pose.x - waypoint[0])**2
            accum += (pose.y - waypoint[1])**2
            accum += (pose.z - waypoint[2])**2
            accum = sqrt(accum)
            #print("self.position is: ", self.position)
            #print("desired waypoint: ", wp_next)
            if accum < 0.2:
                print("self.position is: ", pose)
                print("desired waypoint: ", waypoint)
                break

            
