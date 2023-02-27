import rospy
import std_msgs
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State 
from mavros_msgs.srv import CommandBool, SetMode

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
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_callback) # publishes both position and orientation (quaternion)

    
    # Callback for the state subscriber
    def state_callback(self, state):
        self.state = state

    # Callback for the pose subscriber
    def pose_callback(self, pose_msg):
        self.pose = pose_msg

    # Arm the drone
    def arm(self):

        return
    
    # De-arm drone and shut down
    def shutdown(self):

        return
    
    # Lift drone off ground
    def takeoff(self, h):

        return
    
    # Hover drone in place for a set amount of time
    def hover(self, t):

        return
    
    # Land drone safely
    def land(self):

        return
    
    # Move drone to milestone
    def move(self, milestone):

        return
