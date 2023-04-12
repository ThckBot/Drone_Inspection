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

from jetcam.csi_camera import CSICamera

from math import *
import numpy as np
from numpy.linalg import norm
import time


#Drone FSM Class
class DroneFSM():
    def __init__(self):
        # fill in
        self.position = None
        self.orientation = None

        self.vicon_position = None
        self.vicon_orientation = None
        self.ekf_position = None
        self.ekf_orientation = None

        self.positions = []
        self.waypoints = []
        self.vicon_waypoints= []
        self.sp_pos = Point()
        self.obstacle_positions =[] #TODO add subscriber to obstacle position

        self.vicon_transform = np.identity(4) # Default transform is all 1s
        self.t1 = []
        self.t2 = []
        
        self.state = State()
        
        self.fsm_state = -1
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

        # Instantiate Camera
        frame_width, frame_height = 640, 480
        self.camera = CSICamera(width=frame_width, height=frame_height, capture_width=frame_width, capture_height=frame_height, capture_fps=30)

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
        
        self.t1 = pos1
        self.t2 = pos2

        # Compute the transformation matrix between the two poses changes from vicon to local frame
        T21 = np.matmul(np.linalg.inv(T2), T1)
        #theta = 0.15
        #fix_frame = np.array([[np.cos(theta),-np.sin(theta),0, 0],[np.sin(theta),np.cos(theta),0,0],[0,0,1,0],[0,0,0,1]])
        #T31 = np.matmul(fix_frame,T21)
        #return T31
        
        self.vicon_transform = T21
        self.vicon_transform_created = True
        print("Computed Transform")        
        return T21

    def update_rotation(self):
        quat1 = [self.orientation.x,self.orientation.y,self.orientation.z,self.orientation.w]
        quat2 = [self.vicon_orientation.x,self.vicon_orientation.y,self.vicon_orientation.z,self.vicon_orientation.w]
        r1 = rot_from_quat(quat1)
        r2 = rot_from_quat(quat2)
        
        T1 = np.eye(4)
        T1[:3,:3] = r1
        T1[:3,3] = self.t1

        T2 = np.eye(4)
        T2[:3,:3] = r2
        T2[:3,3] = self.t2

        self.vicon_transform = np.matmul(np.linalg.inv(T2),T1)


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

        while (not rospy.is_shutdown()) and self.fsm_state == 'Launch':
            t = time.time()

            # Update timestamp and publish sp 
            self.publish_setpoint(self.sp_pos)
            self.rate.sleep()

    def hover_test(self, hover_time):
        print('Position holding...')
        t0 = time.time()
        self.sp_pos = self.position
        t = time.time()
        while (not rospy.is_shutdown()) and hover_time >= t-t0:
            t = time.time()

            # Update timestamp and publish sp 
            self.publish_setpoint(self.sp_pos)
            self.rate.sleep()


    # Land drone safely
    def land(self):
        print("Landing...")
        self.sp_pos = self.position
        while (self.position.z > 0.01):
            print(self.position.z)
            self.sp_pos.z = self.position.z - 0.1
            self.publish_setpoint(self.sp_pos)
            self.rate.sleep()
        # self.stop()
    

    # Publish set point
    def publish_setpoint(self, setpoint, yaw = -np.pi/4):
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

        q = quaternion_from_euler(0, 0, yaw)
        sp.pose.orientation.x = q[0]
        sp.pose.orientation.y = q[1]
        sp.pose.orientation.z = q[2]
        sp.pose.orientation.w = q[3]

        # Publish to the rosnode
        sp.header.stamp = rospy.Time.now()
        self.setpoint_publisher.publish(sp)


        return

    def dance(self):
        # Move around in a square from waypoint
        start_pose = self.position
        dist = 0.3
        new_pose = Point()
        for x in range(-1,2,2):
            for y in range(-1,2,2):
                new_pose.x = start_pose.x + x*dist
                new_pose.y = start_pose.y + y*dist
                new_pose.z = start_pose.z

                curr_pose = self.position
                while np.linalg.norm([curr_pose.x-new_pose.x,curr_pose.y-new_pose.y,curr_pose.z-new_pose.z]) > 0.1:
                    self.publish_setpoint(new_pose)
                    self.rate.sleep()
                    curr_pose = self.position

    def nav_waypoints(self, wp_next, vicon_milestones = False, vicon_pose = False):
        print('Untransformed waypoint:',wp_next)
        waypoint_pose = Point()
        if vicon_milestones:
            # Transform milestones from vicon to local frame for publishing
            self.vicon_waypoints.append(wp_next)
            wp_transformed = self.compute_waypoint_transform(wp_next)
            print('transformation matrix',self.vicon_transform)
            waypoint_pose.x = wp_transformed[0]
            waypoint_pose.y = wp_transformed[1]
            waypoint_pose.z = wp_transformed[2]
            self.waypoints.append(wp_transformed)
        else:
            # Set up waypoints without transformation
            waypoint_pose.x = wp_next[0]
            waypoint_pose.y = wp_next[1]
            waypoint_pose.z = wp_next[2]

        print('Moving to waypoint:', waypoint_pose)
        i = 0
        # Drive drone to waypoint
        while not rospy.is_shutdown() and self.fsm_state == 'Waypoints':
            i += 1
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

            if i%5 == 0:
                self.positions.append([self.position.x,self.position.y,self.position.z])

            if accum < 0.3:
                print("self.position is: ", pose)
                print("desired waypoint: ", waypoint)
                break

    def scan_circle(self, turns):
        '''
        Input: Turns
        Output: NOne
        Based on the current position stored in self.obstacle_positions 
        the drone will turn to face a series of direction, number of 
        points where it will stop depends on the parameter turns
        '''

        pass


    def scan_obstacles(self):
        '''
        Input: None
        Output: 
        Based on the current position stored in self.obstacle_positions 
        the drone will turn to face each obstacle. It will then check the colour 
        using a colour check funciton and finally assing this to the colour property of the function
        '''

        for obs in self.obstacle_positions:
            # Get relative orientation by
            # Get direciton vector
            self.sp_pos.x = self.position.x
            self.sp_pos.y = self.position.y
            self.sp_pos.z = self.position.z

            d = np.array([obs[0]-self.position.x, obs[1]-self.position.y, obs[2]-self.position.z])

            # Get yaw between direction vector in degrees
            theta = np.arctan2(d[1], d[0])
            theta_rad = theta * np.pi / 180

            # Extract current orientation
            x = self.orientation.x
            y = self.orientation.y
            z = self.orientation.z
            w = self.orientation.w

            # Get yaw in degrees
            current_yaw = quaternion_to_yaw(self.orientation)

            # Publish current positions while yaw error > 10 degrees/.17 rad
            while abs(current_yaw - theta) > 0.17:
                self.publish_setpoint(self.sp_pos, yaw = theta_rad)
                current_yaw = quaternion_to_yaw(self.orientation)

            # Get Current image as a numpy array
            frame = image = self.camera.read()

            # Detect Colour
            # TODO
            #color self.detect_colour()
            
            # Assign Colour to Obstacle
            #obs.colour = color
        